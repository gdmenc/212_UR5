/*  calibrate_camera_chessboard.cpp  —  Hand-eye calibration (eye-in-hand) for a
 *  wrist-mounted Intel RealSense camera using CHESSBOARD pattern.
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
 *      ./calibrate_camera_chessboard                        # interactive calibration
 *      ./calibrate_camera_chessboard --ip 192.168.1.101     # specify robot IP
 *      ./calibrate_camera_chessboard --no-robot             # camera-only (manual EE input)
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

// =========================================================================
//  Chessboard Configuration - ADJUST THESE TO MATCH YOUR BOARD
// =========================================================================
static constexpr int    CHESS_WIDTH     = 8;     // interior corners (width)
static constexpr int    CHESS_HEIGHT    = 6;     // interior corners (height)
static constexpr double SQUARE_SIZE_M   = 0.068; // size of each square in meters

static constexpr double MIN_ROTATION_SPREAD_DEG  = 10.0;

// =========================================================================
//  Helpers: RealSense intrinsics → OpenCV
// =========================================================================

static void rs_intrinsics_to_cv(const rs2::video_stream_profile& profile,
                                cv::Mat& K, cv::Mat& D) {
    auto intr = profile.get_intrinsics();
    K = (cv::Mat_<double>(3, 3) <<
         intr.fx, 0.0,     intr.ppx,
         0.0,     intr.fy, intr.ppy,
         0.0,     0.0,     1.0);
    D = cv::Mat(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) D.at<double>(i) = intr.coeffs[i];
}

// =========================================================================
//  Optional: Intrinsic calibration using chessboard (more accurate)
// =========================================================================

static bool calibrate_intrinsics_chessboard(
    rs2::pipeline& pipe,
    cv::Size chessboard_size,
    float square_size,
    cv::Mat& K_out,
    cv::Mat& D_out,
    int min_frames = 15)
{
    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    // Prepare object points: (0,0,0), (1,0,0), (2,0,0), ... in real-world coords
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < chessboard_size.height; ++i) {
        for (int j = 0; j < chessboard_size.width; ++j) {
            objp.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }

    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  INTRINSIC CAMERA CALIBRATION\n";
    std::cout << std::string(60, '=') << "\n";
    std::cout << "  SPACE : capture frame (need " << min_frames << " frames)\n";
    std::cout << "  C     : calibrate with captured frames\n";
    std::cout << "  Q     : skip and use RealSense factory intrinsics\n";
    std::cout << std::string(60, '=') << "\n\n";

    const std::string window = "Intrinsic Calibration";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::resizeWindow(window, 960, 540);

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        if (!color_frame) continue;

        cv::Mat bgr(cv::Size(640, 480), CV_8UC3,
                   const_cast<void*>(color_frame.get_data()),
                   cv::Mat::AUTO_STEP);
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, chessboard_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        cv::Mat display = bgr.clone();
        if (found) {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cv::drawChessboardCorners(display, chessboard_size, corners, found);
        }

        // Status overlay
        std::string status = "Captured: " + std::to_string(objpoints.size()) +
                           "/" + std::to_string(min_frames);
        cv::putText(display, status, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        std::string board_status = found ? "Board: FOUND" : "Board: NOT FOUND";
        cv::Scalar color = found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(display, board_status, cv::Point(10, 65),
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

        cv::putText(display, "SPACE=capture  C=calibrate  Q=skip", cv::Point(10, 100),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 2);

        cv::imshow(window, display);
        int key = cv::waitKey(1) & 0xFF;

        if (key == 'q' || key == 'Q') {
            std::cout << "[INFO] Skipping intrinsic calibration. Using RealSense factory intrinsics.\n";
            cv::destroyWindow(window);
            return false;
        }

        if (key == ' ' && found) {
            objpoints.push_back(objp);
            imgpoints.push_back(corners);
            std::cout << "[CAPTURE] Frame " << objpoints.size() << " saved.\n";
        }

        if (key == 'c' || key == 'C') {
            if (objpoints.size() < static_cast<size_t>(min_frames)) {
                std::cout << "[WARN] Need at least " << min_frames
                         << " frames. Currently have " << objpoints.size() << ".\n";
                continue;
            }

            std::vector<cv::Mat> rvecs, tvecs;
            double rms = cv::calibrateCamera(
                objpoints, imgpoints, gray.size(),
                K_out, D_out, rvecs, tvecs);

            std::cout << "\n--- Intrinsic Calibration Complete ---\n";
            std::cout << "RMS reprojection error: " << rms << " pixels\n";
            std::cout << "Camera Matrix:\n" << K_out << "\n";
            std::cout << "Distortion Coeffs:\n" << D_out << "\n";
            std::cout << "--------------------------------------\n\n";

            // Save to file
            cv::FileStorage fs("camera_intrinsics.yml", cv::FileStorage::WRITE);
            fs << "camera_matrix" << K_out;
            fs << "distortion_coefficients" << D_out;
            fs << "rms_error" << rms;
            fs.release();
            std::cout << "Saved to camera_intrinsics.yml\n\n";

            cv::destroyWindow(window);
            return true;
        }
    }
}

// =========================================================================
//  Helpers: pose maths
// =========================================================================

static void rotvec_to_Rt(const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                         cv::Mat& R, cv::Mat& t) {
    cv::Rodrigues(cv::Mat(rvec), R);
    t = cv::Mat(tvec).reshape(1, 3).clone();
}

static cv::Mat make_T44(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    t.reshape(1, 3).copyTo(T(cv::Rect(3, 0, 1, 3)));
    return T;
}

static cv::Mat inv_T44(const cv::Mat& M) {
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
//  Chessboard detection and pose estimation
// =========================================================================

struct DetectResult {
    bool      ok          = false;
    cv::Vec3d rvec        = {0, 0, 0};
    cv::Vec3d tvec        = {0, 0, 0};
    int       n_corners   = 0;
    cv::Mat   vis;         // annotated BGR frame
};

static DetectResult detect_chessboard(const cv::Mat& gray,
                                      const cv::Mat& K,
                                      const cv::Mat& D,
                                      cv::Size board_size,
                                      float square_size) {
    DetectResult res;
    cv::cvtColor(gray, res.vis, cv::COLOR_GRAY2BGR);

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, board_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!found) {
        return res;
    }

    // Refine corner positions
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);

    // Draw corners
    cv::drawChessboardCorners(res.vis, board_size, corners, found);
    res.n_corners = static_cast<int>(corners.size());

    // Prepare object points (3D points of chessboard in board coordinates)
    std::vector<cv::Point3f> obj_pts;
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            obj_pts.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }

    // Estimate pose using solvePnP
    cv::Mat rvec_m, tvec_m;
    bool ok = cv::solvePnP(obj_pts, corners, K, D, rvec_m, tvec_m);

    if (!ok) return res;

    res.ok = true;
    res.rvec = cv::Vec3d(rvec_m.at<double>(0),
                         rvec_m.at<double>(1),
                         rvec_m.at<double>(2));
    res.tvec = cv::Vec3d(tvec_m.at<double>(0),
                         tvec_m.at<double>(1),
                         tvec_m.at<double>(2));

    // Draw coordinate axes on the board
    cv::drawFrameAxes(res.vis, K, D, rvec_m, tvec_m, square_size * 2);

    return res;
}

// =========================================================================
//  Residual (AX = XB)
// =========================================================================

static std::pair<double, double> hand_eye_residual(
        const std::vector<cv::Mat>& R_ee,  const std::vector<cv::Mat>& t_ee,
        const std::vector<cv::Mat>& R_tgt, const std::vector<cv::Mat>& t_tgt,
        const cv::Mat& R_x, const cv::Mat& t_x) {
    int n = static_cast<int>(R_ee.size());
    cv::Mat X = make_T44(R_x, t_x);
    double rot_sum = 0.0, trans_sum = 0.0;
    int count = 0;

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            cv::Mat A = inv_T44(make_T44(R_ee[i], t_ee[i]))
                      * make_T44(R_ee[j], t_ee[j]);
            cv::Mat B = make_T44(R_tgt[i], t_tgt[i])
                      * inv_T44(make_T44(R_tgt[j], t_tgt[j]));
            cv::Mat LHS = A * X;
            cv::Mat RHS = X * B;

            cv::Mat dR = LHS(cv::Rect(0, 0, 3, 3))
                       * RHS(cv::Rect(0, 0, 3, 3)).t();
            double cos_a = (cv::trace(dR)[0] - 1.0) / 2.0;
            cos_a = std::max(-1.0, std::min(1.0, cos_a));
            rot_sum += std::acos(cos_a) * 180.0 / CV_PI;

            cv::Mat dt = LHS(cv::Rect(3, 0, 1, 3))
                       - RHS(cv::Rect(3, 0, 1, 3));
            trans_sum += cv::norm(dt) * 1000.0;  // → mm
            ++count;
        }
    }
    return {rot_sum / count, trans_sum / count};
}

// =========================================================================
//  Diversity score
// =========================================================================

static double rotation_spread(const std::vector<cv::Mat>& R_list) {
    if (R_list.size() < 2) return 0.0;
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
//  EE pose input (manual — no ur_rtde C++ SDK)
// =========================================================================

static bool prompt_ee_pose(cv::Vec3d& rvec_out, cv::Vec3d& tvec_out) {
    std::cout << "\n  Enter TCP pose [x y z rx ry rz] (metres; rotation as axis-angle in RADIANS):\n  > " << std::flush;
    std::string line;
    std::getline(std::cin, line);
    if (line.empty() || line == "skip") return false;

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

static void save_json(const std::string& path,
                      const std::string& arm,
                      const std::string& method_name,
                      double rot_err,
                      const cv::Mat& R_result,
                      const cv::Mat& t_result,
                      int n_samples) {
    cv::Mat rvec_result;
    cv::Rodrigues(R_result, rvec_result);

    std::ofstream out(path);
    out << std::fixed << std::setprecision(10);
    out << "{\n";
    out << "  \"description\": \"T_ee_camera: pose of the camera in the robot TCP (EE) frame. "
           "Produced by hand-eye calibration (eye-in-hand, AX=XB). "
           "Method: " << method_name << ", rot_residual_deg: "
        << std::setprecision(4) << rot_err << ".\",\n";
    out << "  \"arm\": \"" << arm << "\",\n";
    out << "  \"method\": \"" << method_name << "\",\n";
    out << "  \"rot_residual_deg\": " << std::setprecision(4) << rot_err << ",\n";

    out << std::setprecision(10);
    out << "  \"R_ee_camera\": [\n";
    for (int r = 0; r < 3; ++r) {
        out << "    [";
        for (int c = 0; c < 3; ++c) {
            out << R_result.at<double>(r, c);
            if (c < 2) out << ", ";
        }
        out << "]" << (r < 2 ? "," : "") << "\n";
    }
    out << "  ],\n";

    out << "  \"t_ee_camera_m\": [";
    for (int i = 0; i < 3; ++i) {
        out << t_result.at<double>(i);
        if (i < 2) out << ", ";
    }
    out << "],\n";

    out << "  \"rotvec_ee_camera\": [";
    for (int i = 0; i < 3; ++i) {
        out << rvec_result.at<double>(i);
        if (i < 2) out << ", ";
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

static void run_live(const std::string& arm,
                     int min_samples,
                     const std::string& output_path,
                     bool no_robot,
                     bool use_intrinsic_cal) {
    // --- RealSense camera ---------------------------------------------------
    rs2::pipeline pipe;
    rs2::config   cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    std::cout << "[INFO] Starting RealSense pipeline ...\n";
    auto profile = pipe.start(cfg);

    // Warm-up
    std::cout << "[INFO] Warming up camera (15 frames) ...\n";
    for (int i = 0; i < 15; ++i) pipe.wait_for_frames();

    cv::Mat K, D;

    // Get camera intrinsics
    if (use_intrinsic_cal) {
        bool success = calibrate_intrinsics_chessboard(
            pipe,
            cv::Size(CHESS_WIDTH, CHESS_HEIGHT),
            SQUARE_SIZE_M,
            K, D,
            15  // minimum frames
        );

        if (!success) {
            // Fall back to RealSense factory intrinsics
            auto color_profile = profile.get_stream(RS2_STREAM_COLOR)
                                       .as<rs2::video_stream_profile>();
            rs_intrinsics_to_cv(color_profile, K, D);
        }
    } else {
        // Use RealSense factory intrinsics
        auto color_profile = profile.get_stream(RS2_STREAM_COLOR)
                                   .as<rs2::video_stream_profile>();
        rs_intrinsics_to_cv(color_profile, K, D);
        std::cout << "[INFO] Using RealSense factory intrinsics.\n";
    }

    std::cout << "\nCamera Matrix:\n" << K << "\n";
    std::cout << "Distortion Coeffs:\n" << D << "\n\n";

    rs2::align align(RS2_STREAM_COLOR);

    // --- sample storage -----------------------------------------------------
    std::vector<cv::Mat> R_ee_list, t_ee_list;
    std::vector<cv::Mat> R_tgt_list, t_tgt_list;

    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "  HAND-EYE CALIBRATION (Chessboard)\n";
    std::cout << std::string(60, '=') << "\n";
    std::cout << "  SPACE : capture sample (board must be detected)\n";
    std::cout << "  Q     : finish and run calibration\n";
    std::cout << "  ESC   : abort\n";
    if (no_robot) {
        std::cout << "  (no-robot mode: you will type EE pose after each capture)\n";
    }
    std::cout << std::string(60, '=') << "\n\n";

    const std::string window = "Hand-Eye Calibration";
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::resizeWindow(window, 960, 540);

    while (true) {
        rs2::frameset frameset = pipe.wait_for_frames();
        rs2::frameset aligned = align.process(frameset);
        rs2::frame color_frame = aligned.get_color_frame();
        if (!color_frame) continue;

        cv::Mat bgr(cv::Size(640, 480), CV_8UC3,
                    const_cast<void*>(color_frame.get_data()),
                    cv::Mat::AUTO_STEP);
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

        auto det = detect_chessboard(gray, K, D,
                                     cv::Size(CHESS_WIDTH, CHESS_HEIGHT),
                                     SQUARE_SIZE_M);

        int n_samp = static_cast<int>(R_ee_list.size());
        double diversity = rotation_spread(R_ee_list);

        // --- status overlay --------------------------------------------------
        auto put_text = [&](const std::string& text, int row,
                            cv::Scalar color = {200, 200, 200}) {
            cv::putText(det.vis, text, {10, 28 + 28 * row},
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv::LINE_AA);
        };
        put_text("Samples : " + std::to_string(n_samp) +
                 "  (need " + std::to_string(min_samples) + ")", 0);

        std::string board_str = det.ok
            ? "FOUND (" + std::to_string(det.n_corners) + " corners)"
            : "NOT FOUND";
        put_text("Board   : " + board_str, 1,
                 det.ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

        char div_buf[64];
        std::snprintf(div_buf, sizeof(div_buf), "Diversity: %.2f  (aim > 0.5)", diversity);
        put_text(div_buf, 2, diversity >= 0.5
                 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255));

        put_text("SPACE=capture  Q=finish  ESC=abort", 4, {180, 180, 180});

        cv::imshow(window, det.vis);
        int key = cv::waitKey(1) & 0xFF;

        if (key == 27) {  // ESC
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
                std::cout << "[SKIP] Board not detected — reposition.\n";
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

            cv::Mat R_board, t_board;
            rotvec_to_Rt(det.rvec, det.tvec, R_board, t_board);

            R_ee_list.push_back(R_ee);
            t_ee_list.push_back(t_ee);
            R_tgt_list.push_back(R_board);
            t_tgt_list.push_back(t_board);

            double dist = cv::norm(cv::Mat(det.tvec));
            std::printf("[CAPTURE #%d] EE xyz=(%.3f, %.3f, %.3f) m  "
                        "board dist=%.3f m  diversity=%.2f\n",
                        n_samp + 1,
                        ee_tvec[0], ee_tvec[1], ee_tvec[2],
                        dist,
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
        {"Tsai",       cv::CALIB_HAND_EYE_TSAI},
        {"Park",       cv::CALIB_HAND_EYE_PARK},
        {"Horaud",     cv::CALIB_HAND_EYE_HORAUD},
        {"Andreff",    cv::CALIB_HAND_EYE_ANDREFF},
        {"Daniilidis", cv::CALIB_HAND_EYE_DANIILIDIS},
    };

    std::printf("%-14s %12s %15s\n", "Method", "Rot err (°)", "Trans err (mm)");
    std::printf("%s\n", std::string(44, '-').c_str());

    std::string best_name;
    cv::Mat best_R, best_t;
    double best_rot_err = 1e9;

    for (auto& m : methods) {
        try {
            cv::Mat R_x, t_x;
            cv::calibrateHandEye(
                R_ee_list, t_ee_list,
                R_tgt_list, t_tgt_list,
                R_x, t_x, m.flag);

            auto [rot_e, trans_e] = hand_eye_residual(
                R_ee_list, t_ee_list, R_tgt_list, t_tgt_list, R_x, t_x);

            std::printf("%-14s %12.3f %15.2f\n",
                        m.name.c_str(), rot_e, trans_e);

            if (rot_e < best_rot_err) {
                best_rot_err = rot_e;
                best_name    = m.name;
                best_R       = R_x.clone();
                best_t       = t_x.clone();
            }
        } catch (const cv::Exception& e) {
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
    save_json(output_path, arm, best_name, best_rot_err,
              best_R, best_t, static_cast<int>(R_ee_list.size()));

    cv::Mat rvec_out;
    cv::Rodrigues(best_R, rvec_out);

    std::printf("\n  t_ee_camera_m = [%+.4f, %+.4f, %+.4f]\n",
                best_t.at<double>(0), best_t.at<double>(1), best_t.at<double>(2));
    std::printf("  rotvec_ee_cam = [%+.4f, %+.4f, %+.4f]\n",
                rvec_out.at<double>(0), rvec_out.at<double>(1), rvec_out.at<double>(2));
    std::printf("\nCalibration complete!\n");
}

// =========================================================================
//  CLI
// =========================================================================

static void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [OPTIONS]\n"
        "\n"
        "Options:\n"
        "  --arm NAME          Arm name for output metadata (default: left)\n"
        "  --ip IP             Robot IP (default: 192.168.1.101)\n"
        "  --no-robot          Skip robot connection (manual EE pose entry)\n"
        "  --calibrate-intrinsics  Run intrinsic calibration before hand-eye\n"
        "  --min-samples N     Minimum captures before Q accepted (default: 12)\n"
        "  --output PATH       Output JSON path (default: T_ee_camera.json)\n"
        "  --help              Show this help\n"
        "\n"
        "Chessboard configuration (edit source to change):\n"
        "  Interior corners: %d x %d\n"
        "  Square size:      %.3f m\n"
        "\n"
        "EE pose format (prompted after each SPACE capture):\n"
        "  x y z rx ry rz  — translation in metres, rotation as axis-angle in RADIANS\n"
        "  Use the 'rotvec' field from record_waypoints snapshots (already in radians).\n",
        prog, CHESS_WIDTH, CHESS_HEIGHT, SQUARE_SIZE_M);
}

int main(int argc, char** argv) {

    // Defaults
    std::string arm               = "left";
    std::string ip                = "192.168.1.101";
    bool        no_robot          = false;
    bool        use_intrinsic_cal = false;
    int         min_samples       = 12;
    std::string output            = "T_ee_camera.json";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--arm" && i+1<argc)           { arm = argv[++i]; }
        else if (arg == "--ip" && i+1<argc)       { ip = argv[++i]; }
        else if (arg == "--no-robot")             { no_robot = true; }
        else if (arg == "--calibrate-intrinsics") { use_intrinsic_cal = true; }
        else if (arg == "--min-samples" && i+1<argc) { min_samples = std::atoi(argv[++i]); }
        else if (arg == "--output" && i+1<argc)      { output = argv[++i]; }
        else if (arg == "--help" || arg == "-h")  { print_usage(argv[0]); return 0; }
        else { std::fprintf(stderr, "Unknown option: %s\n", arg.c_str()); return 1; }
    }

    run_live(arm, min_samples, output, no_robot, use_intrinsic_cal);
    return 0;
}
