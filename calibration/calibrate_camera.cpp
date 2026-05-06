/*  calibrate_camera_apriltag.cpp  —  Hand-eye calibration (eye-in-hand) for a
 *  wrist-mounted Intel RealSense camera using APRILTAG pattern.
 *
 *  Solves T_ee_camera — the fixed rigid transform from camera frame to the
 *  robot's TCP (tool-centre-point) frame.  Once known:
 *
 *      X_task_camera = X_task_ee  *  T_ee_camera
 *
 *  Algorithm: OpenCV calibrateHandEye (AX = XB, all five methods tried).
 *
 *  Build:
 *      cd calibration && mkdir -p build && cd build
 *      cmake .. && make -j$(sysctl -n hw.ncpu)
 *
 *  Usage:
 *      ./calibrate_camera_apriltag                        # interactive
 * calibration
 *      ./calibrate_camera_apriltag --no-robot             # manual EE input
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <librealsense2/rs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

// AprilTag library
extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag36h11.h>
}

// =========================================================================
//  AprilTag Configuration - ADJUST THESE TO MATCH YOUR TAG
// =========================================================================
static constexpr double TAG_SIZE_M = 0.093; // Tag size in meters (e.g., 93mm)
static constexpr int TARGET_TAG_ID =
    -1; // -1 = auto-lock to first seen tag for session consistency.

static constexpr double MIN_ROTATION_SPREAD_DEG = 10.0;

// =========================================================================
//  Helpers: RealSense intrinsics → OpenCV
// =========================================================================

static void rs_intrinsics_to_cv(const rs2::video_stream_profile &profile,
                                cv::Mat &K, cv::Mat &D) {
  auto intr = profile.get_intrinsics();
  K = (cv::Mat_<double>(3, 3) << intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy,
       0.0, 0.0, 1.0);
  D = cv::Mat(1, 5, CV_64F);
  for (int i = 0; i < 5; ++i)
    D.at<double>(i) = intr.coeffs[i];
}

// =========================================================================
//  Helpers: pose maths
// =========================================================================

static void rotvec_to_Rt(const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                         cv::Mat &R, cv::Mat &t) {
  cv::Rodrigues(cv::Mat(rvec), R);
  t = cv::Mat(tvec).reshape(1, 3).clone();
}

static cv::Mat make_T44(const cv::Mat &R, const cv::Mat &t) {
  cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
  R.copyTo(T(cv::Rect(0, 0, 3, 3)));
  t.reshape(1, 3).copyTo(T(cv::Rect(3, 0, 1, 3)));
  return T;
}

static cv::Mat inv_T44(const cv::Mat &M) {
  cv::Mat R = M(cv::Rect(0, 0, 3, 3));
  cv::Mat t = M(cv::Rect(3, 0, 1, 3));
  cv::Mat out = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat Rt = R.t();
  Rt.copyTo(out(cv::Rect(0, 0, 3, 3)));
  cv::Mat neg_t = -Rt * t;
  neg_t.copyTo(out(cv::Rect(3, 0, 1, 3)));
  return out;
}

// =========================================================================
//  AprilTag detection and pose estimation
// =========================================================================

struct DetectResult {
  bool ok = false;
  int tag_id = -1;
  cv::Vec3d rvec = {0, 0, 0};
  cv::Vec3d tvec = {0, 0, 0};
  cv::Mat vis; // annotated BGR frame
};

static DetectResult detect_apriltag(const cv::Mat &gray, const cv::Mat &K,
                                    double tag_size, int target_id) {
  DetectResult res;
  cv::cvtColor(gray, res.vis, cv::COLOR_GRAY2BGR);

  // Create AprilTag detector
  apriltag_family_t *tf = tag36h11_create();
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  // Configure detector
  td->quad_decimate = 1.0;
  td->quad_sigma = 0.0;
  td->nthreads = 1;
  td->refine_edges = 1;

  // Convert to apriltag image format
  image_u8_t im = {.width = gray.cols,
                   .height = gray.rows,
                   .stride = gray.cols,
                   .buf = gray.data};

  // Detect tags
  zarray_t *detections = apriltag_detector_detect(td, &im);

  if (zarray_size(detections) == 0) {
    // No tags detected
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return res;
  }

  // Find target tag (or first tag if target_id == -1)
  apriltag_detection_t *det = nullptr;
  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t *d;
    zarray_get(detections, i, &d);

    if (target_id == -1 || d->id == target_id) {
      det = d;
      break;
    }
  }

  if (!det) {
    // Target tag not found
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return res;
  }

  // Draw detection
  for (int j = 0; j < 4; ++j) {
    cv::Point p1(det->p[j][0], det->p[j][1]);
    cv::Point p2(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]);
    cv::line(res.vis, p1, p2, cv::Scalar(0, 255, 0), 2);
  }

  // Draw tag ID
  cv::Point center(det->c[0], det->c[1]);
  cv::putText(res.vis, "ID: " + std::to_string(det->id), center,
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

  // Estimate pose
  apriltag_detection_info_t info;
  info.det = det;
  info.tagsize = tag_size;
  info.fx = K.at<double>(0, 0);
  info.fy = K.at<double>(1, 1);
  info.cx = K.at<double>(0, 2);
  info.cy = K.at<double>(1, 2);

  apriltag_pose_t pose;
  estimate_tag_pose(&info, &pose);

  // Convert pose to OpenCV format
  // AprilTag pose: R is 3x3 rotation matrix, t is 3x1 translation
  cv::Mat R_mat(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R_mat.at<double>(i, j) = pose.R->data[i * 3 + j];
    }
  }

  cv::Vec3d t_vec(pose.t->data[0], pose.t->data[1], pose.t->data[2]);

  // Convert rotation matrix to rotation vector
  cv::Mat rvec_mat;
  cv::Rodrigues(R_mat, rvec_mat);
  cv::Vec3d r_vec(rvec_mat.at<double>(0), rvec_mat.at<double>(1),
                  rvec_mat.at<double>(2));

  res.ok = true;
  res.tag_id = det->id;
  res.rvec = r_vec;
  res.tvec = t_vec;

  // Draw coordinate axes
  cv::Mat distCoeffs =
      cv::Mat::zeros(5, 1, CV_64F); // No distortion for RealSense
  cv::drawFrameAxes(res.vis, K, distCoeffs, rvec_mat, cv::Mat(t_vec),
                    tag_size * 0.5);

  // Cleanup
  apriltag_detections_destroy(detections);
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);

  return res;
}

// =========================================================================
//  Residual (AX = XB)
// =========================================================================

static std::pair<double, double> hand_eye_residual(
    const std::vector<cv::Mat> &R_ee, const std::vector<cv::Mat> &t_ee,
    const std::vector<cv::Mat> &R_tgt, const std::vector<cv::Mat> &t_tgt,
    const cv::Mat &R_x, const cv::Mat &t_x) {
  int n = static_cast<int>(R_ee.size());
  cv::Mat X = make_T44(R_x, t_x);
  double rot_sum = 0.0, trans_sum = 0.0;
  int count = 0;

  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      cv::Mat A =
          inv_T44(make_T44(R_ee[i], t_ee[i])) * make_T44(R_ee[j], t_ee[j]);
      cv::Mat B =
          make_T44(R_tgt[i], t_tgt[i]) * inv_T44(make_T44(R_tgt[j], t_tgt[j]));
      cv::Mat LHS = A * X;
      cv::Mat RHS = X * B;

      cv::Mat dR = LHS(cv::Rect(0, 0, 3, 3)) * RHS(cv::Rect(0, 0, 3, 3)).t();
      double cos_a = (cv::trace(dR)[0] - 1.0) / 2.0;
      cos_a = std::max(-1.0, std::min(1.0, cos_a));
      rot_sum += std::acos(cos_a) * 180.0 / CV_PI;

      cv::Mat dt = LHS(cv::Rect(3, 0, 1, 3)) - RHS(cv::Rect(3, 0, 1, 3));
      trans_sum += cv::norm(dt) * 1000.0; // → mm
      ++count;
    }
  }
  return {rot_sum / count, trans_sum / count};
}

// =========================================================================
//  Diversity score
// =========================================================================

static double rotation_spread(const std::vector<cv::Mat> &R_list) {
  if (R_list.size() < 2)
    return 0.0;
  double sum = 0.0;
  int cnt = 0;
  for (size_t i = 0; i < R_list.size(); ++i) {
    for (size_t j = i + 1; j < R_list.size(); ++j) {
      cv::Mat dR = R_list[i].t() * R_list[j];
      double cos_a = (cv::trace(dR)[0] - 1.0) / 2.0;
      cos_a = std::max(-1.0, std::min(1.0, cos_a));
      sum += std::abs(std::acos(cos_a)) * 180.0 / CV_PI;
      ++cnt;
    }
  }
  double mean_angle = sum / cnt;
  return std::min(1.0, mean_angle / 45.0);
}

// =========================================================================
//  EE pose input (manual)
// =========================================================================

static bool prompt_ee_pose(cv::Vec3d &rvec_out, cv::Vec3d &tvec_out) {
  std::cout << "\n  Enter TCP pose [x y z rx ry rz] (metres; rotation as "
               "axis-angle in RADIANS):\n  > "
            << std::flush;
  std::string line;
  std::getline(std::cin, line);
  if (line.empty() || line == "skip")
    return false;

  std::istringstream iss(line);
  double x, y, z, rx, ry, rz;
  if (!(iss >> x >> y >> z >> rx >> ry >> rz)) {
    std::cerr << "  [ERROR] Expected 6 numbers.  Skipping.\n";
    return false;
  }
  tvec_out = {x, y, z};
  rvec_out = {rx, ry, rz};
  return true;
}

// =========================================================================
//  JSON output
// =========================================================================

static void save_json(const std::string &path, const std::string &arm,
                      const std::string &method_name, double rot_err,
                      const cv::Mat &R_result, const cv::Mat &t_result,
                      int n_samples) {
  cv::Mat rvec_result;
  cv::Rodrigues(R_result, rvec_result);

  std::ofstream out(path);
  out << std::fixed << std::setprecision(10);
  out << "{\n";
  out << "  \"description\": \"T_ee_camera: pose of the camera in the robot "
         "TCP (EE) frame. "
         "Produced by hand-eye calibration (eye-in-hand, AX=XB). "
         "Method: "
      << method_name << ", rot_residual_deg: " << std::setprecision(4)
      << rot_err << ".\",\n";
  out << "  \"arm\": \"" << arm << "\",\n";
  out << "  \"method\": \"" << method_name << "\",\n";
  out << "  \"rot_residual_deg\": " << std::setprecision(4) << rot_err << ",\n";

  out << std::setprecision(10);
  out << "  \"R_ee_camera\": [\n";
  for (int r = 0; r < 3; ++r) {
    out << "    [";
    for (int c = 0; c < 3; ++c) {
      out << R_result.at<double>(r, c);
      if (c < 2)
        out << ", ";
    }
    out << "]" << (r < 2 ? "," : "") << "\n";
  }
  out << "  ],\n";

  out << "  \"t_ee_camera_m\": [";
  for (int i = 0; i < 3; ++i) {
    out << t_result.at<double>(i);
    if (i < 2)
      out << ", ";
  }
  out << "],\n";

  out << "  \"rotvec_ee_camera\": [";
  for (int i = 0; i < 3; ++i) {
    out << rvec_result.at<double>(i);
    if (i < 2)
      out << ", ";
  }
  out << "],\n";

  out << "  \"n_samples\": " << n_samples << "\n";
  out << "}\n";
  out.close();

  std::printf("\nT_ee_camera saved to: %s\n", path.c_str());
}

// =========================================================================
//  Main calibration loop
// =========================================================================

static void run_live(const std::string &arm, int min_samples,
                     const std::string &output_path) {
  // --- RealSense camera ---------------------------------------------------
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

  std::cout << "[INFO] Starting RealSense pipeline ...\n";
  auto profile = pipe.start(cfg);

  // Warm-up
  std::cout << "[INFO] Warming up camera (15 frames) ...\n";
  for (int i = 0; i < 15; ++i)
    pipe.wait_for_frames();

  cv::Mat K, D;
  auto color_profile =
      profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  rs_intrinsics_to_cv(color_profile, K, D);

  std::cout << "[INFO] Using RealSense factory intrinsics.\n";
  std::cout << "\nCamera Matrix:\n" << K << "\n";
  std::cout << "Distortion Coeffs:\n" << D << "\n\n";

  rs2::align align(RS2_STREAM_COLOR);

  // --- sample storage -----------------------------------------------------
  std::vector<cv::Mat> R_ee_list, t_ee_list;
  std::vector<cv::Mat> R_tgt_list, t_tgt_list;

  std::cout << "\n" << std::string(60, '=') << "\n";
  std::cout << "  HAND-EYE CALIBRATION (AprilTag)\n";
  std::cout << std::string(60, '=') << "\n";
  std::cout << "  Tag size: " << TAG_SIZE_M << " meters\n";
  if (TARGET_TAG_ID >= 0) {
    std::cout << "  Target tag ID: " << TARGET_TAG_ID << "\n";
  } else {
    std::cout << "  Target: auto-lock first seen tag ID\n";
  }
  std::cout << std::string(60, '=') << "\n";
  std::cout << "  SPACE : capture sample (tag must be detected)\n";
  std::cout << "  Q     : finish and run calibration\n";
  std::cout << "  ESC   : abort\n";
  std::cout << std::string(60, '=') << "\n\n";

  const std::string window = "Hand-Eye Calibration";
  cv::namedWindow(window, cv::WINDOW_NORMAL);
  cv::resizeWindow(window, 960, 540);
  int active_target_id = TARGET_TAG_ID;

  while (true) {
    rs2::frameset frameset = pipe.wait_for_frames();
    rs2::frameset aligned = align.process(frameset);
    rs2::frame color_frame = aligned.get_color_frame();
    if (!color_frame)
      continue;

    cv::Mat bgr(cv::Size(640, 480), CV_8UC3,
                const_cast<void *>(color_frame.get_data()), cv::Mat::AUTO_STEP);
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    auto det = detect_apriltag(gray, K, TAG_SIZE_M, active_target_id);
    if (active_target_id == -1 && det.ok) {
      active_target_id = det.tag_id;
      std::cout << "[INFO] Locked target tag ID to " << active_target_id
                << " for this calibration session.\n";
      // Re-run detection with locked ID so overlay/status are consistent.
      det = detect_apriltag(gray, K, TAG_SIZE_M, active_target_id);
    }

    int n_samp = static_cast<int>(R_ee_list.size());
    double diversity = rotation_spread(R_ee_list);

    // --- status overlay --------------------------------------------------
    auto put_text = [&](const std::string &text, int row,
                        cv::Scalar color = {200, 200, 200}) {
      cv::putText(det.vis, text, {10, 28 + 28 * row}, cv::FONT_HERSHEY_SIMPLEX,
                  0.65, color, 2, cv::LINE_AA);
    };

    put_text("Samples : " + std::to_string(n_samp) + "  (need " +
                 std::to_string(min_samples) + ")",
             0);

    std::string tag_str =
        det.ok ? "FOUND (ID " + std::to_string(det.tag_id) + ")" : "NOT FOUND";
    put_text("Tag     : " + tag_str, 1,
             det.ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));
    if (active_target_id >= 0) {
      put_text("Target  : ID " + std::to_string(active_target_id), 2,
               cv::Scalar(120, 220, 255));
    } else {
      put_text("Target  : waiting to lock first tag", 2, cv::Scalar(0, 200, 255));
    }

    char div_buf[64];
    std::snprintf(div_buf, sizeof(div_buf), "Diversity: %.2f  (aim > 0.5)",
                  diversity);
    put_text(div_buf, 3,
             diversity >= 0.5 ? cv::Scalar(0, 255, 0)
                              : cv::Scalar(0, 200, 255));

    put_text("SPACE=capture  Q=finish  ESC=abort", 5, {180, 180, 180});

    cv::imshow(window, det.vis);
    int key = cv::waitKey(1) & 0xFF;

    if (key == 27) { // ESC
      std::cout << "\n[ABORT] User pressed ESC.\n";
      pipe.stop();
      cv::destroyAllWindows();
      return;
    }

    if (key == 'q' || key == 'Q') {
      if (n_samp < min_samples) {
        std::printf("[WARN] Only %d samples — need at least %d. Keep going.\n",
                    n_samp, min_samples);
      } else {
        break;
      }
    }

    if (key == ' ') {
      if (!det.ok) {
        std::cout << "[SKIP] Tag not detected — reposition.\n";
        continue;
      }

      // --- Get EE pose -------------------------------------------------
      cv::Vec3d ee_rvec, ee_tvec;
      if (!prompt_ee_pose(ee_rvec, ee_tvec)) {
        std::cout << "[SKIP] No pose entered.\n";
        continue;
      }

      cv::Mat R_ee, t_ee;
      rotvec_to_Rt(ee_rvec, ee_tvec, R_ee, t_ee);

      // Reject if too similar to last sample
      if (!R_ee_list.empty()) {
        cv::Mat dR = R_ee_list.back().t() * R_ee;
        double cos_a = (cv::trace(dR)[0] - 1.0) / 2.0;
        cos_a = std::max(-1.0, std::min(1.0, cos_a));
        double angle_deg = std::abs(std::acos(cos_a)) * 180.0 / CV_PI;
        if (angle_deg < MIN_ROTATION_SPREAD_DEG) {
          std::printf("[SKIP] Too similar (Δrot=%.1f° < %.1f°). "
                      "Move to a more different pose.\n",
                      angle_deg, MIN_ROTATION_SPREAD_DEG);
          continue;
        }
      }

      cv::Mat R_tag, t_tag;
      rotvec_to_Rt(det.rvec, det.tvec, R_tag, t_tag);

      R_ee_list.push_back(R_ee);
      t_ee_list.push_back(t_ee);
      R_tgt_list.push_back(R_tag);
      t_tgt_list.push_back(t_tag);

      double dist = cv::norm(cv::Mat(det.tvec));
      std::printf("[CAPTURE #%d] EE xyz=(%.3f, %.3f, %.3f) m  "
                  "tag dist=%.3f m  diversity=%.2f\n",
                  n_samp + 1, ee_tvec[0], ee_tvec[1], ee_tvec[2], dist,
                  rotation_spread(R_ee_list));
    }
  }

  pipe.stop();
  cv::destroyAllWindows();
  std::printf("\n[INFO] Collected %zu samples.  Running calibration ...\n\n",
              R_ee_list.size());

  // --- Hand-eye calibration -----------------------------------------------
  struct Method {
    std::string name;
    cv::HandEyeCalibrationMethod flag;
  };
  std::vector<Method> methods = {
      {"Tsai", cv::CALIB_HAND_EYE_TSAI},
      {"Park", cv::CALIB_HAND_EYE_PARK},
      {"Horaud", cv::CALIB_HAND_EYE_HORAUD},
      {"Andreff", cv::CALIB_HAND_EYE_ANDREFF},
      {"Daniilidis", cv::CALIB_HAND_EYE_DANIILIDIS},
  };

  std::printf("%-14s %12s %15s\n", "Method", "Rot err (°)", "Trans err (mm)");
  std::printf("%s\n", std::string(44, '-').c_str());

  std::string best_name;
  cv::Mat best_R, best_t;
  double best_rot_err = 1e9;

  for (auto &m : methods) {
    try {
      cv::Mat R_x, t_x;
      cv::calibrateHandEye(R_ee_list, t_ee_list, R_tgt_list, t_tgt_list, R_x,
                           t_x, m.flag);

      auto [rot_e, trans_e] = hand_eye_residual(
          R_ee_list, t_ee_list, R_tgt_list, t_tgt_list, R_x, t_x);

      std::printf("%-14s %12.3f %15.2f\n", m.name.c_str(), rot_e, trans_e);

      if (rot_e < best_rot_err) {
        best_rot_err = rot_e;
        best_name = m.name;
        best_R = R_x.clone();
        best_t = t_x.clone();
      }
    } catch (const cv::Exception &e) {
      std::printf("%-14s FAILED: %s\n", m.name.c_str(), e.what());
    }
  }

  std::printf("%s\n", std::string(44, '-').c_str());

  if (best_R.empty()) {
    std::cerr << "All calibration methods failed — check your samples.\n";
    return;
  }

  std::printf("Best: %s  (rot err = %.3f°)\n", best_name.c_str(), best_rot_err);

  // --- Save ---------------------------------------------------------------
  save_json(output_path, arm, best_name, best_rot_err, best_R, best_t,
            static_cast<int>(R_ee_list.size()));

  cv::Mat rvec_out;
  cv::Rodrigues(best_R, rvec_out);

  std::printf("\n  t_ee_camera_m = [%+.4f, %+.4f, %+.4f]\n",
              best_t.at<double>(0), best_t.at<double>(1), best_t.at<double>(2));
  std::printf("  rotvec_ee_cam = [%+.4f, %+.4f, %+.4f]\n",
              rvec_out.at<double>(0), rvec_out.at<double>(1),
              rvec_out.at<double>(2));
  std::printf("\nCalibration complete!\n");
}

// =========================================================================
//  CLI
// =========================================================================

static void print_usage(const char *prog) {
  std::printf(
      "Usage: %s [OPTIONS]\n"
      "\n"
      "Options:\n"
      "  --arm NAME          Arm name for output metadata (default: left)\n"
      "  --min-samples N     Minimum captures before Q accepted (default: 12)\n"
      "  --output PATH       Output JSON path (default: T_ee_camera.json)\n"
      "  --help              Show this help\n"
      "\n"
      "AprilTag configuration (edit source to change):\n"
      "  Tag size:      %.3f m\n"
      "  Target tag ID: %d (-1 = any tag)\n"
      "\n"
      "EE pose format (prompted after each SPACE capture):\n"
      "  x y z rx ry rz  — translation in metres, rotation as axis-angle in "
      "RADIANS\n"
      "  Use the 'base_pose' rotvec field from record_waypoints snapshots.\n",
      prog, TAG_SIZE_M, TARGET_TAG_ID);
}

int main(int argc, char **argv) {

  // Defaults
  std::string arm = "left";
  int min_samples = 12;
  std::string output = "T_ee_camera.json";

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--arm" && i + 1 < argc) {
      arm = argv[++i];
    } else if (arg == "--min-samples" && i + 1 < argc) {
      min_samples = std::atoi(argv[++i]);
    } else if (arg == "--output" && i + 1 < argc) {
      output = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "Unknown option: %s\n", arg.c_str());
      return 1;
    }
  }

  run_live(arm, min_samples, output);
  return 0;
}