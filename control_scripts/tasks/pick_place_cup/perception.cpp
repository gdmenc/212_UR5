/**
 * perception.cpp — Cup detection + grasp candidates
 * Task: pick_place_cup
 *
 * Detects a cup (COCO class 41) with YOLOv8 via a RealSense D435i,
 * back-projects the bbox centre to a 3D camera-frame point, transforms
 * to task frame, and prints 8 top-down rim grasp candidates as JSON.
 *
 * Geometry mirrors control_scripts/grasps/cup.py exactly.
 *
 * Dependencies
 * ------------
 *   librealsense2  (Intel RealSense SDK ≥ 2.50)
 *   OpenCV 4.x     (DNN module)
 *   Eigen 3.x
 *
 * Export the YOLO model first (one-time):
 *   python3 -c "from ultralytics import YOLO; \
 *     YOLO('../../../../perception/yolov8n.pt').export(format='onnx')"
 *
 * Build (from this directory):
 *   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
 *
 * Run:
 *   ./build/perception_cup
 *   ./build/perception_cup --model /path/to/yolov8n.onnx --conf 0.4
 *
 * Output: JSON on stdout, progress messages on stderr.
 * The calling Python task script can read stdout via subprocess.
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

// ---------------------------------------------------------------------------
// Geometry constants — mirror control_scripts/grasps/cup.py
// ---------------------------------------------------------------------------

static constexpr double CUP_RIM_OUTER_RADIUS_M = 0.044; // 4.4 cm rim radius
static constexpr double CUP_HEIGHT_M            = 0.154; // 15.4 cm total height
static constexpr double CUP_PREGRASP_OFFSET     = 0.050; // 5 cm along tool -Z
static constexpr double CUP_GRASP_FORCE         = 15.0;  // N — light: thin rim

static constexpr int COCO_CUP   = 41;
static constexpr int N_ANGLES   = 8;
static constexpr int N_WARMUP   = 15;
static constexpr int N_FRAMES   = 5;

// Camera-to-task homogeneous transform.
// Replace with calibration values once hand-eye calibration is done.
// Columns: [R | t; 0 0 0 1] — transform from camera frame to task frame.
static Eigen::Matrix4d make_T_task_camera() {
    return Eigen::Matrix4d::Identity(); // TODO: load from calibration file
}

// ---------------------------------------------------------------------------
// Rotation: top-down rim grasp at angle_rad
// Mirrors cup.py::_top_down_rim_rotation:
//   (R_z(angle+π) * R_y(π)) * R_z(π/2)
// ---------------------------------------------------------------------------

static Eigen::Matrix3d top_down_rim_rotation(double angle_rad) {
    Eigen::Matrix3d Rz_yaw = Eigen::AngleAxisd(angle_rad + M_PI,
                                               Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry_pi  = Eigen::AngleAxisd(M_PI,
                                               Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz_90  = Eigen::AngleAxisd(M_PI / 2.0,
                                               Eigen::Vector3d::UnitZ()).matrix();
    return (Rz_yaw * Ry_pi) * Rz_90;
}

// ---------------------------------------------------------------------------
// Rotation matrix → axis-angle vector (for JSON)
// ---------------------------------------------------------------------------

static Eigen::Vector3d to_rotvec(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.axis() * aa.angle();
}

// ---------------------------------------------------------------------------
// Detection helpers
// ---------------------------------------------------------------------------

struct BBox {
    int x1, y1, x2, y2;
    float conf;
};

// Run YOLOv8 ONNX model (output shape: 1×84×8400) via OpenCV DNN.
// Returns detections for target_class_id sorted by descending confidence.
static std::vector<BBox> yolo_detect(
    cv::dnn::Net& net,
    const cv::Mat& bgr,
    int target_class_id,
    float conf_threshold)
{
    const int INPUT = 640;
    float sx = (float)bgr.cols / INPUT;
    float sy = (float)bgr.rows / INPUT;

    cv::Mat blob;
    cv::dnn::blobFromImage(bgr, blob, 1.0 / 255.0,
                           cv::Size(INPUT, INPUT), cv::Scalar(), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outs;
    net.forward(outs, net.getUnconnectedOutLayersNames());

    // outs[0]: (1, 84, 8400) → reshape to (84, 8400) → transpose to (8400, 84)
    cv::Mat out = outs[0].reshape(1, {84, 8400});
    cv::transpose(out, out);

    std::vector<cv::Rect> rects;
    std::vector<float>    scores;

    for (int i = 0; i < 8400; i++) {
        cv::Mat cls_scores = out.row(i).colRange(4, 84);
        double max_score;
        cv::Point max_loc;
        cv::minMaxLoc(cls_scores, nullptr, &max_score, nullptr, &max_loc);
        if (max_loc.x != target_class_id || (float)max_score < conf_threshold)
            continue;

        float cx = out.at<float>(i, 0) * sx;
        float cy = out.at<float>(i, 1) * sy;
        float w  = out.at<float>(i, 2) * sx;
        float h  = out.at<float>(i, 3) * sy;
        int x1 = std::max(0, (int)(cx - w / 2));
        int y1 = std::max(0, (int)(cy - h / 2));
        rects.emplace_back(x1, y1,
                           std::min((int)w, bgr.cols - x1),
                           std::min((int)h, bgr.rows - y1));
        scores.push_back((float)max_score);
    }
    if (rects.empty()) return {};

    std::vector<int> idx;
    cv::dnn::NMSBoxes(rects, scores, conf_threshold, 0.45f, idx);

    std::vector<BBox> result;
    for (int i : idx) {
        auto& r = rects[i];
        result.push_back({r.x, r.y, r.x + r.width, r.y + r.height, scores[i]});
    }
    std::sort(result.begin(), result.end(),
              [](const BBox& a, const BBox& b){ return a.conf > b.conf; });
    return result;
}

// Median depth of the central 50% of bbox → deproject to 3D camera-frame point.
static bool bbox_centre_3d(
    const rs2::depth_frame& depth,
    const rs2_intrinsics& intr,
    const BBox& bbox,
    Eigen::Vector3d& out)
{
    int cx = (bbox.x1 + bbox.x2) / 2, cy = (bbox.y1 + bbox.y2) / 2;
    int hw = std::max(1, (bbox.x2 - bbox.x1) / 4);
    int hh = std::max(1, (bbox.y2 - bbox.y1) / 4);
    int W = depth.get_width(), H = depth.get_height();

    std::vector<float> ds;
    for (int v = std::max(0, cy - hh); v < std::min(H, cy + hh); v++)
        for (int u = std::max(0, cx - hw); u < std::min(W, cx + hw); u++) {
            float d = depth.get_distance(u, v);
            if (d > 0.05f && d < 4.0f) ds.push_back(d);
        }
    if (ds.empty()) return false;

    std::sort(ds.begin(), ds.end());
    float d_med = ds[ds.size() / 2];

    float pix[2] = {(float)cx, (float)cy}, pt[3];
    rs2_deproject_pixel_to_point(pt, &intr, pix, d_med);
    out = Eigen::Vector3d(pt[0], pt[1], pt[2]);
    return true;
}

// ---------------------------------------------------------------------------
// Grasp candidate computation
// ---------------------------------------------------------------------------

struct GraspCandidate {
    Eigen::Vector3d translation;
    Eigen::Vector3d rotvec;
    Eigen::Vector3d pre_translation;
    Eigen::Vector3d pre_rotvec;
    double pregrasp_offset_m;
    double grasp_force_n;
    std::string description;
};

// cup_base_task: 3D position of the cup's resting base in task frame.
static std::vector<GraspCandidate> cup_rim_candidates(
    const Eigen::Vector3d& cup_base_task,
    int n = N_ANGLES)
{
    std::vector<GraspCandidate> out;
    out.reserve(n);
    for (int i = 0; i < n; i++) {
        double angle = 2.0 * M_PI * i / n;
        Eigen::Vector3d rim(
            CUP_RIM_OUTER_RADIUS_M * std::cos(angle),
            CUP_RIM_OUTER_RADIUS_M * std::sin(angle),
            CUP_HEIGHT_M);
        Eigen::Vector3d grasp_t = cup_base_task + rim;
        Eigen::Matrix3d R       = top_down_rim_rotation(angle);

        // Pregrasp: CUP_PREGRASP_OFFSET along tool -Z (= -R.col(2))
        Eigen::Vector3d pre_t = grasp_t - CUP_PREGRASP_OFFSET * R.col(2);

        std::ostringstream desc;
        desc.precision(0);
        desc << std::fixed << "cup rim grasp @ "
             << (angle * 180.0 / M_PI) << "deg";

        out.push_back({
            grasp_t,
            to_rotvec(R),
            pre_t,
            to_rotvec(R),
            CUP_PREGRASP_OFFSET,
            CUP_GRASP_FORCE,
            desc.str(),
        });
    }
    return out;
}

// ---------------------------------------------------------------------------
// JSON helpers
// ---------------------------------------------------------------------------

static std::string v3(const Eigen::Vector3d& v) {
    std::ostringstream ss;
    ss << std::fixed;
    ss.precision(6);
    ss << "[" << v[0] << "," << v[1] << "," << v[2] << "]";
    return ss.str();
}

static void print_json(
    bool detected,
    float conf,
    const Eigen::Vector3d& pos_cam,
    const Eigen::Vector3d& pos_task,
    const Eigen::Vector3d& cup_base,
    const std::vector<GraspCandidate>& cands)
{
    std::cout << "{\n";
    std::cout << "  \"detected\": " << (detected ? "true" : "false") << ",\n";
    if (!detected) { std::cout << "  \"candidates\": []\n}\n"; return; }
    std::cout << "  \"confidence\": " << conf << ",\n";
    std::cout << "  \"position_camera_m\": " << v3(pos_cam) << ",\n";
    std::cout << "  \"position_task_m\": "   << v3(pos_task) << ",\n";
    std::cout << "  \"cup_base_task_m\": "   << v3(cup_base) << ",\n";
    std::cout << "  \"candidates\": [\n";
    for (int i = 0; i < (int)cands.size(); i++) {
        const auto& g = cands[i];
        std::cout << "    {\n";
        std::cout << "      \"index\": "            << i              << ",\n";
        std::cout << "      \"description\": \""    << g.description  << "\",\n";
        std::cout << "      \"translation\": "      << v3(g.translation)    << ",\n";
        std::cout << "      \"rotvec\": "           << v3(g.rotvec)         << ",\n";
        std::cout << "      \"pre_translation\": "  << v3(g.pre_translation)<< ",\n";
        std::cout << "      \"pre_rotvec\": "       << v3(g.pre_rotvec)     << ",\n";
        std::cout << "      \"pregrasp_offset_m\": "<< g.pregrasp_offset_m  << ",\n";
        std::cout << "      \"grasp_force_n\": "    << g.grasp_force_n      << "\n";
        std::cout << "    }" << (i + 1 < (int)cands.size() ? "," : "") << "\n";
    }
    std::cout << "  ]\n}\n";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::string model_path = "../../../../perception/yolov8n.onnx";
    float conf_threshold = 0.40f;

    for (int i = 1; i < argc; i++) {
        std::string a(argv[i]);
        if (a == "--model" && i + 1 < argc)  model_path      = argv[++i];
        else if (a == "--conf" && i + 1 < argc) conf_threshold = std::stof(argv[++i]);
    }

    // Load YOLO model
    cv::dnn::Net net;
    try {
        net = cv::dnn::readNetFromONNX(model_path);
    } catch (const cv::Exception&) {
        std::cerr << "[ERROR] Cannot load model: " << model_path << "\n";
        std::cerr << "  Export: python3 -c \"from ultralytics import YOLO; "
                     "YOLO('../../../../perception/yolov8n.pt').export(format='onnx')\"\n";
        return 1;
    }
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // Open RealSense
    rs2::pipeline pipe;
    rs2::config   cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16,  30);
    rs2::pipeline_profile profile;
    try {
        profile = pipe.start(cfg);
    } catch (const rs2::error& e) {
        std::cerr << "[ERROR] RealSense: " << e.what() << "\n";
        return 1;
    }

    rs2_intrinsics intr = profile.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>()
                                 .get_intrinsics();
    rs2::align align_to_color(RS2_STREAM_COLOR);

    Eigen::Matrix4d T_task_cam = make_T_task_camera();

    // Warmup
    std::cerr << "[INFO] Warming up (" << N_WARMUP << " frames)...\n";
    for (int i = 0; i < N_WARMUP; i++) pipe.wait_for_frames();

    // Aggregate detections over N_FRAMES
    std::cerr << "[INFO] Detecting cup (COCO " << COCO_CUP << ") over "
              << N_FRAMES << " frames...\n";
    bool found = false;
    float best_conf = 0.0f;
    BBox best_bbox  = {};
    Eigen::Vector3d pos_camera;

    for (int fi = 0; fi < N_FRAMES; fi++) {
        rs2::frameset fs  = pipe.wait_for_frames();
        rs2::frameset afs = align_to_color.process(fs);
        auto color_f = afs.get_color_frame();
        auto depth_f = afs.get_depth_frame();

        cv::Mat bgr(cv::Size(color_f.get_width(), color_f.get_height()),
                    CV_8UC3, (void*)color_f.get_data(), cv::Mat::AUTO_STEP);

        auto dets = yolo_detect(net, bgr, COCO_CUP, conf_threshold);
        if (dets.empty()) continue;

        const BBox& d = dets[0];
        if (!found || d.conf > best_conf) {
            Eigen::Vector3d p;
            if (bbox_centre_3d(depth_f, intr, d, p)) {
                best_bbox  = d;
                best_conf  = d.conf;
                pos_camera = p;
                found      = true;
            }
        }
    }
    pipe.stop();

    if (!found) {
        std::cerr << "[WARN] No cup detected (conf≥" << conf_threshold << ").\n";
        print_json(false, 0, {}, {}, {}, {});
        return 0;
    }

    std::cerr << "[INFO] Cup at camera ("
              << pos_camera.transpose() << ") m, conf=" << best_conf << "\n";

    // Transform to task frame
    Eigen::Vector4d h(pos_camera[0], pos_camera[1], pos_camera[2], 1.0);
    Eigen::Vector3d pos_task = (T_task_cam * h).head<3>();

    // Estimate cup base: shift detected centre down by half the cup height.
    // The camera typically sees the cup's midsection, so centre ≈ midpoint.
    // Refine using a full point cloud for better accuracy.
    Eigen::Vector3d cup_base = pos_task;
    cup_base[2] -= CUP_HEIGHT_M / 2.0;

    auto cands = cup_rim_candidates(cup_base);
    print_json(true, best_conf, pos_camera, pos_task, cup_base, cands);
    return 0;
}
