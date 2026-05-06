/**
 * perception_hook_testing.cpp — Bowl detection + depth diagnostics for hook testing
 * Task: hook_grasp_from_vision (standalone binary; do not merge into pick_place_bowl_hook_full)
 *
 * Modes
 * -----
 *   default (preview)   detect bowl, save annotated JPEG to --save-image path,
 *                       wait for user to press Enter in the terminal before exiting.
 *                       cv::imshow is NOT used because this binary runs under sudo
 *                       which blocks macOS window-server access.
 *   --headless          no image save, no pause, pure JSON stdout.  Used for
 *                       fully autonomous runs (--auto in Python).
 *
 * Args
 * ----
 *   --model   <path>          YOLOv11 ONNX
 *   --classes <path>          coco-classes.txt
 *   --conf    <float>         detection threshold (default 0.40)
 *   --T-task-camera "f0…f15"  row-major 4×4 float string from Python
 *   --depth-color-offset-pix <du> <dv>  optional shift for residual RGB–depth after align_to_color
 *   --save-image <path>       where to write the annotated JPEG (preview mode)
 *   --headless                skip image save and terminal pause
 *   --sam-encoder <path>      SAM2 encoder ONNX (optional but recommended)
 *   --sam-decoder <path>      SAM2 decoder ONNX
 *   --no-sam                  disable SAM2 even if paths are passed (YOLO-only XY)
 *
 * Pipeline (when built with ONNX Runtime and both SAM paths are passed):
 *   YOLO → bowl bbox (prompt) → SAM2 mask + centroid → depth at SAM xy (fallback: bbox).
 *
 * stdout — JSON:
 *   { "detected": bool, "confidence": float,
 *     "bowl_pos_camera_m": [...], "bowl_base_task_m": [...],
 *     "bowl_rim_task_m": [...],
 *     optional when SAM mask ok: "bowl_center_method", "bowl_base_before_mask_refine_m",
 *       "mask_planar_samples_n", "mask_planar_pca_major_rad_task",
 *       "mask_planar_eccentricity" (≈0 → circular in task XY; axis angle ill-defined),
 *       circle + ellipse contour fits (task XY metres); "planar_oblique_view_hint" when oval,
 *     optional when detected + depth metrics ok:
 *       "table_median_depth_m", "object_median_depth_m",
 *       "depth_delta_table_minus_object_m", "depth_edge_metric_mean"
 *     "candidates": [...] }
 *
 * Build:
 *   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
 *
 * Camera / segmentation note
 * ----------------------------
 * Capture and depth are librealsense2 (C++) only — not pyrealsense — for
 * consistent behaviour on macOS and the lab machines.  Finer masks (e.g.
 * SAM2) belong in this binary too once exported to ONNX (OpenCV dnn /
 * ONNX Runtime); do not route live RGB-D through Python.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#if defined(HAVE_ONNXRUNTIME)
#include <onnxruntime_cxx_api.h>
#endif

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr float  INPUT_WIDTH          = 640.0f;
static constexpr float  INPUT_HEIGHT         = 640.0f;
static constexpr float  SCORE_THRESHOLD      = 0.25f;
static constexpr float  NMS_THRESHOLD        = 0.45f;
static constexpr float  CONFIDENCE_THRESHOLD = 0.25f;

static constexpr int    COCO_BOWL            = 45;
static constexpr int    DEFAULT_N_WARMUP     = 15;
static constexpr int    DEFAULT_N_FRAMES     = 10;  // accumulate over more frames for reliability

static constexpr double BOWL_RIM_OUTER_RADIUS_M = 0.075;  // measured 2026-05-03: 150mm dia
static constexpr double BOWL_RIM_Z_OFFSET_M     = 0.075;  // measured 2026-05-03: 75mm height
static constexpr double HOOK_RIM_PREGRASP_OFFSET = 0.050;
static constexpr double BOWL_GRASP_FORCE         = 15.0;
static constexpr int    N_ANGLES                 = 8;
static constexpr int    SAM_INPUT_SIZE           = 1024;
static constexpr int    SAM_MASK_SIZE            = 256;

/**
 * Added to every (u,v) before ``get_distance`` / deproject when using depth aligned to color
 * with **color** intrinsics. ``align_to_color`` removes most RGB–depth parallax; a few pixels
 * of residual (or bad device calibration) show up as grasp/geometry shifted on the JPEG
 * relative to the bowl — tune with RealSense Viewer depth-on-RGB overlay, then set e.g.
 * ``--depth-color-offset-pix -3 0``.
 */
static int g_depth_color_off_u = 0;
static int g_depth_color_off_v = 0;

static inline void offsetUvClamped(int u, int v, int W, int H, int& uo, int& vo)
{
    uo = std::clamp(u + g_depth_color_off_u, 0, W - 1);
    vo = std::clamp(v + g_depth_color_off_v, 0, H - 1);
}

static inline float depthDistOfs(const rs2::depth_frame& depth, int u, int v)
{
    const int W = depth.get_width(), H = depth.get_height();
    int uo, vo;
    offsetUvClamped(u, v, W, H, uo, vo);
    return depth.get_distance(uo, vo);
}

static inline void deprojectOfs(
    const rs2_intrinsics& intr,
    int u,
    int v,
    float z_m,
    float pt_cam[3],
    const rs2::depth_frame& depth)
{
    const int W = depth.get_width(), H = depth.get_height();
    int uo, vo;
    offsetUvClamped(u, v, W, H, uo, vo);
    float pix[2] = {(float)uo, (float)vo};
    rs2_deproject_pixel_to_point(pt_cam, &intr, pix, z_m);
}

#if defined(HAVE_ONNXRUNTIME)
class Sam2Segmenter {
public:
    bool load(const std::string& encoder_path, const std::string& decoder_path) {
        Ort::SessionOptions opts;
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        encoder_ = std::make_unique<Ort::Session>(env_, encoder_path.c_str(), opts);
        decoder_ = std::make_unique<Ort::Session>(env_, decoder_path.c_str(), opts);
        return true;
    }

    bool refineByBox(
        const cv::Mat& bgr,
        const cv::Rect& box,
        cv::Mat& out_mask_u8,
        cv::Point2f& out_centroid)
    {
        if (!encoder_ || !decoder_ || bgr.empty()) return false;

        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        cv::Mat resized;
        cv::resize(rgb, resized, cv::Size(SAM_INPUT_SIZE, SAM_INPUT_SIZE), 0, 0, cv::INTER_LINEAR);

        std::vector<float> img(1 * 3 * SAM_INPUT_SIZE * SAM_INPUT_SIZE);
        static constexpr float mean[3] = {123.675f, 116.28f, 103.53f};
        static constexpr float stdv[3] = {58.395f, 57.12f, 57.375f};
        for (int y = 0; y < SAM_INPUT_SIZE; ++y) {
            const cv::Vec3b* row = resized.ptr<cv::Vec3b>(y);
            for (int x = 0; x < SAM_INPUT_SIZE; ++x) {
                for (int c = 0; c < 3; ++c) {
                    const float v = (float)row[x][c];
                    img[c * SAM_INPUT_SIZE * SAM_INPUT_SIZE + y * SAM_INPUT_SIZE + x] = (v - mean[c]) / stdv[c];
                }
            }
        }

        std::array<int64_t, 4> img_shape{1, 3, SAM_INPUT_SIZE, SAM_INPUT_SIZE};
        auto mem = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeDefault);
        Ort::Value image_tensor = Ort::Value::CreateTensor<float>(
            mem, img.data(), img.size(), img_shape.data(), img_shape.size());

        const char* enc_in_names[] = {"image"};
        const char* enc_out_names[] = {"high_res_feats_0", "high_res_feats_1", "image_embed"};
        auto enc_out = encoder_->Run(
            Ort::RunOptions{nullptr},
            enc_in_names, &image_tensor, 1,
            enc_out_names, 3);

        const float sx = (float)SAM_INPUT_SIZE / (float)bgr.cols;
        const float sy = (float)SAM_INPUT_SIZE / (float)bgr.rows;
        const float x1 = (float)box.x * sx;
        const float y1 = (float)box.y * sy;
        const float x2 = (float)(box.x + box.width) * sx;
        const float y2 = (float)(box.y + box.height) * sy;
        std::array<float, 4> point_coords_buf{x1, y1, x2, y2};
        std::array<float, 2> point_labels_buf{2.0f, 3.0f};
        std::vector<float> mask_input(1 * 1 * SAM_MASK_SIZE * SAM_MASK_SIZE, 0.0f);
        std::array<float, 1> has_mask_input{0.0f};

        std::array<int64_t, 3> point_coords_shape{1, 2, 2};
        std::array<int64_t, 2> point_labels_shape{1, 2};
        std::array<int64_t, 4> mask_input_shape{1, 1, SAM_MASK_SIZE, SAM_MASK_SIZE};
        std::array<int64_t, 1> has_mask_shape{1};

        Ort::Value point_coords = Ort::Value::CreateTensor<float>(
            mem, point_coords_buf.data(), point_coords_buf.size(),
            point_coords_shape.data(), point_coords_shape.size());
        Ort::Value point_labels = Ort::Value::CreateTensor<float>(
            mem, point_labels_buf.data(), point_labels_buf.size(),
            point_labels_shape.data(), point_labels_shape.size());
        Ort::Value mask_in = Ort::Value::CreateTensor<float>(
            mem, mask_input.data(), mask_input.size(),
            mask_input_shape.data(), mask_input_shape.size());
        Ort::Value has_mask = Ort::Value::CreateTensor<float>(
            mem, has_mask_input.data(), has_mask_input.size(),
            has_mask_shape.data(), has_mask_shape.size());

        std::array<const char*, 7> dec_in_names{
            "image_embed", "high_res_feats_0", "high_res_feats_1",
            "point_coords", "point_labels", "mask_input", "has_mask_input"};
        std::array<Ort::Value, 7> dec_inputs{
            std::move(enc_out[2]), std::move(enc_out[0]), std::move(enc_out[1]),
            std::move(point_coords), std::move(point_labels), std::move(mask_in), std::move(has_mask)};
        std::array<const char*, 2> dec_out_names{"masks", "iou_predictions"};
        auto dec_out = decoder_->Run(
            Ort::RunOptions{nullptr},
            dec_in_names.data(), dec_inputs.data(), dec_inputs.size(),
            dec_out_names.data(), dec_out_names.size());

        const float* iou = dec_out[1].GetTensorData<float>();
        int best_idx = 0;
        if (iou[1] > iou[best_idx]) best_idx = 1;
        if (iou[2] > iou[best_idx]) best_idx = 2;

        const float* masks = dec_out[0].GetTensorData<float>();
        const int plane_stride = SAM_MASK_SIZE * SAM_MASK_SIZE;
        const float* logits = masks + best_idx * plane_stride;

        cv::Mat mask_small(SAM_MASK_SIZE, SAM_MASK_SIZE, CV_8UC1, cv::Scalar(0));
        for (int y = 0; y < SAM_MASK_SIZE; ++y) {
            uint8_t* row = mask_small.ptr<uint8_t>(y);
            for (int x = 0; x < SAM_MASK_SIZE; ++x) {
                const float logit = logits[y * SAM_MASK_SIZE + x];
                row[x] = (logit > 0.0f) ? 255 : 0;
            }
        }

        cv::resize(mask_small, out_mask_u8, bgr.size(), 0, 0, cv::INTER_LINEAR);
        cv::threshold(out_mask_u8, out_mask_u8, 127, 255, cv::THRESH_BINARY);
        const cv::Moments m = cv::moments(out_mask_u8, true);
        if (m.m00 <= 1.0) return false;
        out_centroid = cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));
        return true;
    }

private:
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "sam2"};
    std::unique_ptr<Ort::Session> encoder_;
    std::unique_ptr<Ort::Session> decoder_;
};
#endif

// ---------------------------------------------------------------------------
// YOLO detector (bowl-only, headless)
// ---------------------------------------------------------------------------

enum class YOLOVersion { UNKNOWN, YOLOV5, YOLOV8_11 };

class YOLODetector {
public:
    cv::dnn::Net             net;
    std::vector<std::string> class_list;
    YOLOVersion              version        = YOLOVersion::UNKNOWN;
    int                      num_classes    = 0;
    int                      num_detections = 0;

    bool loadModel(const std::string& model_path, const std::string& classes_path) {
        std::ifstream ifs(classes_path);
        if (!ifs.is_open()) {
            std::cerr << "[ERROR] Cannot open classes file: " << classes_path << "\n";
            return false;
        }
        std::string line;
        while (std::getline(ifs, line)) class_list.push_back(line);
        num_classes = (int)class_list.size();
        std::cerr << "[INFO] " << num_classes << " classes loaded\n";

        try {
            net = cv::dnn::readNet(model_path);
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            std::cerr << "[INFO] Model loaded: " << model_path << "\n";
        } catch (const cv::Exception& e) {
            std::cerr << "[ERROR] " << e.what() << "\n";
            return false;
        }
        return detectVersion();
    }

    // Returns true + fills best_conf/best_box if bowl found in this frame.
    bool runFrame(const cv::Mat& bgr, float conf_thr,
                  float& best_conf, cv::Rect& best_box)
    {
        if (version == YOLOVersion::UNKNOWN || bgr.empty()) return false;

        cv::Mat img = bgr.clone();
        int _max = std::max(img.cols, img.rows);
        cv::Mat formatted = cv::Mat::zeros(_max, _max, CV_8UC3);
        img.copyTo(formatted(cv::Rect(0, 0, img.cols, img.rows)));

        float xf = (float)formatted.cols / INPUT_WIDTH;
        float yf = (float)formatted.rows / INPUT_HEIGHT;

        cv::Mat blob;
        cv::dnn::blobFromImage(formatted, blob, 1.0 / 255.0,
                               cv::Size(INPUT_WIDTH, INPUT_HEIGHT),
                               cv::Scalar(), true, false);
        net.setInput(blob);
        std::vector<cv::Mat> outs;
        net.forward(outs, net.getUnconnectedOutLayersNames());

        std::vector<cv::Rect> boxes;
        std::vector<float>    confs;

        if (!outs.empty() && !outs[0].empty()) {
            if (version == YOLOVersion::YOLOV5) parseV5(outs[0], xf, yf, conf_thr, boxes, confs);
            else                                parseV8V11(outs[0], xf, yf, conf_thr, boxes, confs);
        }
        outs.clear(); blob.release(); formatted.release(); img.release();

        if (boxes.empty()) return false;
        std::vector<int> idx;
        cv::dnn::NMSBoxes(boxes, confs, conf_thr, NMS_THRESHOLD, idx);
        if (idx.empty()) return false;

        int bi = idx[0];
        for (int i : idx) if (confs[i] > confs[bi]) bi = i;
        best_conf = confs[bi];
        best_box  = boxes[bi];
        return true;
    }

    // Verify the bounding-box has valid depth in a plausible range.
    // Used only as a sanity / false-positive filter — NOT for position.
    bool depthSanity(const cv::Rect& box, const rs2::depth_frame& depth,
                     float min_m = 0.20f, float max_m = 3.0f) const
    {
        int cx = box.x + box.width / 2, cy = box.y + box.height / 2;
        int hw = std::max(1, box.width / 4), hh = std::max(1, box.height / 4);
        int W = depth.get_width(), H = depth.get_height();
        std::vector<float> ds;
        for (int v = std::max(0, cy - hh); v < std::min(H, cy + hh); v++)
            for (int u = std::max(0, cx - hw); u < std::min(W, cx + hw); u++) {
                float d = depthDistOfs(depth, u, v);
                if (d > 0.05f && d < 4.0f) ds.push_back(d);
            }
        if (ds.empty()) return false;
        std::sort(ds.begin(), ds.end());
        float d_med = ds[ds.size() / 2];
        return d_med >= min_m && d_med <= max_m;
    }

private:
    bool detectVersion() {
        cv::Mat dummy = cv::Mat::zeros(640, 640, CV_8UC3);
        cv::Mat blob;
        cv::dnn::blobFromImage(dummy, blob, 1.0/255.0,
                               cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
        net.setInput(blob);
        std::vector<cv::Mat> outs;
        net.forward(outs, net.getUnconnectedOutLayersNames());
        if (outs.empty()) { std::cerr << "[ERROR] No model output\n"; return false; }
        int d1 = outs[0].size[1], d2 = outs[0].size[2];
        std::cerr << "[INFO] Output shape [1, " << d1 << ", " << d2 << "]\n";
        if      (d1 > 1000 && d2 < 200) { version = YOLOVersion::YOLOV5;    num_detections = d1; std::cerr << "[INFO] YOLOv5\n"; }
        else if (d1 < 200 && d2 > 1000) { version = YOLOVersion::YOLOV8_11; num_detections = d2; std::cerr << "[INFO] YOLOv8/v11\n"; }
        else { std::cerr << "[ERROR] Unknown shape\n"; return false; }
        return true;
    }

    void parseV5(cv::Mat& out, float xf, float yf, float thr,
                 std::vector<cv::Rect>& boxes, std::vector<float>& confs)
    {
        float* data = (float*)out.data;
        int dims = 5 + num_classes;
        for (int i = 0; i < num_detections; i++, data += dims) {
            float obj = data[4];
            if (obj < CONFIDENCE_THRESHOLD) continue;
            cv::Mat sc(1, num_classes, CV_32FC1, data + 5);
            cv::Point cl; double ms;
            cv::minMaxLoc(sc, nullptr, &ms, nullptr, &cl);
            float conf = obj * (float)ms;
            if (cl.x != COCO_BOWL || conf < thr) continue;
            float x=data[0],y=data[1],w=data[2],h=data[3];
            boxes.emplace_back((int)((x-w/2)*xf),(int)((y-h/2)*yf),(int)(w*xf),(int)(h*yf));
            confs.push_back(conf);
        }
    }

    void parseV8V11(cv::Mat& out, float xf, float yf, float thr,
                    std::vector<cv::Rect>& boxes, std::vector<float>& confs)
    {
        int nf = out.size[out.dims==3?1:0], nd = out.size[out.dims==3?2:1];
        cv::Mat m2d(nf, nd, CV_32F, out.data);
        cv::Mat tr; cv::transpose(m2d, tr);
        float* data = (float*)tr.data;
        for (int i = 0; i < num_detections; i++, data += (4+num_classes)) {
            cv::Mat sc(1, num_classes, CV_32FC1, data+4);
            cv::Point cl; double ms;
            cv::minMaxLoc(sc, nullptr, &ms, nullptr, &cl);
            float conf = (float)ms;
            if (cl.x != COCO_BOWL || conf < thr) continue;
            float x=data[0],y=data[1],w=data[2],h=data[3];
            boxes.emplace_back((int)((x-w/2)*xf),(int)((y-h/2)*yf),(int)(w*xf),(int)(h*yf));
            confs.push_back(conf);
        }
        tr.release();
    }
};

// ---------------------------------------------------------------------------
// Depth-to-3D
// ---------------------------------------------------------------------------

// Sample median depth in a patch centred at (cx, cy) within the bbox and
// deproject directly to a task-frame 3D point.  This uses the depth sensor's
// radial measurement rather than tracing an angular ray to an assumed z=0
// plane, so small calibration angular errors don't get amplified by the
// camera-to-table distance.
static bool depthTo3DTask(
    const cv::Rect& box,
    int cx, int cy,
    const rs2::depth_frame& depth,
    const rs2_intrinsics& intr,
    const Eigen::Matrix4d& T_task_cam,
    Eigen::Vector3d& out_base_task)
{
    // Match bottle detector: wide horizontal patch, tight vertical band —
    // reduces mixing rim/background depth when the camera is oblique.
    int hw = std::max(4, box.width / 4);
    int hh = std::max(2, box.height / 8);
    int W = depth.get_width(), H = depth.get_height();

    std::vector<float> ds;
    for (int v = std::max(0, cy - hh); v <= std::min(H - 1, cy + hh); v++)
        for (int u = std::max(0, cx - hw); u <= std::min(W - 1, cx + hw); u++) {
            float d = depthDistOfs(depth, u, v);
            if (d >= 0.10f && d <= 2.0f) ds.push_back(d);
        }
    if (ds.empty()) return false;
    std::sort(ds.begin(), ds.end());
    float d_med = ds[ds.size() / 2];

    float pt[3];
    deprojectOfs(intr, cx, cy, d_med, pt, depth);

    Eigen::Vector4d pt_cam(pt[0], pt[1], pt[2], 1.0);
    Eigen::Vector4d pt_task = T_task_cam * pt_cam;

    out_base_task = pt_task.head<3>();
    // out_base_task.y() = out_base_task.y() + 0.05;
    out_base_task.z() = 0.0;  // snap to table surface
    return true;
}

/**
 * Where the viewing ray through (cx,cy) meets the task horizontal plane z = plane_z_task_m.
 * Uses the **same median-depth patch** as ``depthTo3DTask`` so range matches the centroid sample.
 * Depth fixes distance along the ray; this intersection fixes **XY on the table** implied by that
 * ray — compare to contour/circle center to see lateral (hand-eye / segmentation) residual.
 */
static bool tablePlaneXYFromCentroidRay(
    const cv::Rect& box,
    int cx,
    int cy,
    const rs2::depth_frame& depth,
    const rs2_intrinsics& intr,
    const Eigen::Matrix4d& T_task_cam,
    double plane_z_task_m,
    Eigen::Vector2d& out_xy)
{
    int hw = std::max(4, box.width / 4);
    int hh = std::max(2, box.height / 8);
    int W = depth.get_width(), H = depth.get_height();
    std::vector<float> ds;
    for (int v = std::max(0, cy - hh); v <= std::min(H - 1, cy + hh); v++)
        for (int u = std::max(0, cx - hw); u <= std::min(W - 1, cx + hw); u++) {
            float d = depthDistOfs(depth, u, v);
            if (d >= 0.10f && d <= 2.0f) ds.push_back(d);
        }
    if (ds.empty()) return false;
    std::sort(ds.begin(), ds.end());
    float d_med = ds[ds.size() / 2];

    float pt_cam[3];
    deprojectOfs(intr, cx, cy, d_med, pt_cam, depth);

    Eigen::Vector4d O_cam(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector4d P_cam(pt_cam[0], pt_cam[1], pt_cam[2], 1.0);
    Eigen::Vector4d O_h = T_task_cam * O_cam;
    Eigen::Vector4d P_h = T_task_cam * P_cam;
    Eigen::Vector3d O_t = O_h.head<3>() / O_h.w();
    Eigen::Vector3d P_t = P_h.head<3>() / P_h.w();
    Eigen::Vector3d V = P_t - O_t;
    if (std::abs(V.z()) < 1e-9) return false;
    const double lam = (plane_z_task_m - O_t.z()) / V.z();
    out_xy = O_t.head<2>() + lam * V.head<2>();
    return true;
}

/** Per-pixel depth → task XY (table snap z handled by caller's bowl pose). */
static bool pixelDepthToTaskXY(
    int u, int v,
    const rs2::depth_frame& depth,
    const rs2_intrinsics& intr,
    const Eigen::Matrix4d& T_task_cam,
    Eigen::Vector2d& out_xy)
{
    float d = depthDistOfs(depth, u, v);
    if (d < 0.10f || d > 2.0f) return false;
    float pt[3];
    deprojectOfs(intr, u, v, d, pt, depth);
    Eigen::Vector4d pt_cam(pt[0], pt[1], pt[2], 1.0);
    Eigen::Vector4d pt_task = T_task_cam * pt_cam;
    out_xy = pt_task.head<2>();
    return true;
}

/**
 * Refine bowl base XY using many SAM mask pixels: independent coordinate medians in task
 * frame, plus planar PCA (eccentricity / major-axis angle) for debugging.  Reduces the
 * single-sample bias that looks like a diagonal offset when the camera is oblique.
 */
static bool refineBowlBaseFromMaskPlanar(
    const cv::Mat& mask_u8,
    const rs2::depth_frame& depth,
    const rs2_intrinsics& intr,
    const Eigen::Matrix4d& T_task_cam,
    Eigen::Vector3d& inout_bowl_task,
    std::string& out_method,
    int& out_n_samples,
    double& out_pca_major_rad,
    double& out_pca_ecc,
    bool& out_have_pca)
{
    out_have_pca = false;
    out_n_samples = 0;
    out_pca_major_rad = 0.0;
    out_pca_ecc = 0.0;
    if (mask_u8.empty() || mask_u8.type() != CV_8UC1) return false;

    std::vector<double> xs, ys;
    xs.reserve(12000);
    ys.reserve(12000);
    const int stride = 2;
    for (int v = 0; v < mask_u8.rows; v += stride) {
        const uint8_t* row = mask_u8.ptr<uint8_t>(v);
        for (int u = 0; u < mask_u8.cols; u += stride) {
            if (row[u] < 128) continue;
            Eigen::Vector2d xy;
            if (!pixelDepthToTaskXY(u, v, depth, intr, T_task_cam, xy)) continue;
            xs.push_back(xy.x());
            ys.push_back(xy.y());
        }
    }
    out_n_samples = (int)xs.size();
    static constexpr int kMinPts = 48;
    if (out_n_samples < kMinPts) return false;

    auto median_inplace = [](std::vector<double>& a) -> double {
        const size_t mid = a.size() / 2;
        std::nth_element(a.begin(), a.begin() + mid, a.end());
        return a[mid];
    };
    std::vector<double> xs_copy = xs;
    std::vector<double> ys_copy = ys;
    const double mx = median_inplace(xs_copy);
    const double my = median_inplace(ys_copy);
    inout_bowl_task.x() = mx;
    inout_bowl_task.y() = my;
    inout_bowl_task.z() = 0.0;
    out_method = "mask_planar_median_xy";

    double meanx = 0.0, meany = 0.0;
    for (size_t i = 0; i < xs.size(); ++i) {
        meanx += xs[i];
        meany += ys[i];
    }
    meanx /= (double)xs.size();
    meany /= (double)xs.size();
    double cxx = 0.0, cxy = 0.0, cyy = 0.0;
    for (size_t i = 0; i < xs.size(); ++i) {
        const double dx = xs[i] - meanx;
        const double dy = ys[i] - meany;
        cxx += dx * dx;
        cxy += dx * dy;
        cyy += dy * dy;
    }
    const double invn = 1.0 / (double)xs.size();
    cxx *= invn;
    cxy *= invn;
    cyy *= invn;

    Eigen::Matrix2d C;
    C << cxx, cxy, cxy, cyy;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
    if (es.info() != Eigen::Success) return true;

    const double l0 = es.eigenvalues()(0);
    const double l1 = es.eigenvalues()(1);
    const double lmin = std::min(l0, l1);
    const double lmax = std::max(l0, l1);
    Eigen::Vector2d major = es.eigenvectors().col(1);
    out_pca_major_rad = std::atan2(major.y(), major.x());
    if (lmax > 1e-18)
        out_pca_ecc = std::sqrt(std::max(0.0, 1.0 - lmin / lmax));
    else
        out_pca_ecc = 0.0;
    out_have_pca = true;
    return true;
}

/** Algebraic circle fit (Kåsa / linear least squares on x²+y² + d x + e y + f = 0). */
static bool kasaCircleFit(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    double& out_cx,
    double& out_cy,
    double& out_r,
    double& out_rmse)
{
    const size_t n = xs.size();
    if (n < 3) return false;
    Eigen::MatrixXd A((int)n, 3);
    Eigen::VectorXd b((int)n);
    for (size_t i = 0; i < n; ++i) {
        A((int)i, 0) = xs[i];
        A((int)i, 1) = ys[i];
        A((int)i, 2) = 1.0;
        b((int)i) = -(xs[i] * xs[i] + ys[i] * ys[i]);
    }
    Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
    const double d = sol(0), e = sol(1), f = sol(2);
    out_cx = -0.5 * d;
    out_cy = -0.5 * e;
    const double inner = out_cx * out_cx + out_cy * out_cy - f;
    if (inner <= 1e-12) return false;
    out_r = std::sqrt(inner);
    double sum_sq = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double ri = std::hypot(xs[i] - out_cx, ys[i] - out_cy);
        const double dv = ri - out_r;
        sum_sq += dv * dv;
    }
    out_rmse = std::sqrt(sum_sq / (double)n);
    return std::isfinite(out_r) && out_r > 1e-4;
}

/** Fixed-radius circle center fit in task XY using Gauss-Newton on contour points. */
static bool fixedRadiusCenterFit(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    double known_r_m,
    double& out_cx,
    double& out_cy,
    double& out_rmse)
{
    const size_t n = xs.size();
    if (n < 6 || known_r_m < 1e-4) return false;

    // Initialize from contour centroid.
    double cx = 0.0, cy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        cx += xs[i];
        cy += ys[i];
    }
    cx /= (double)n;
    cy /= (double)n;

    static constexpr int kMaxIters = 20;
    for (int it = 0; it < kMaxIters; ++it) {
        double h11 = 0.0, h12 = 0.0, h22 = 0.0;
        double g1 = 0.0, g2 = 0.0;
        int used = 0;
        for (size_t i = 0; i < n; ++i) {
            const double dx = cx - xs[i];
            const double dy = cy - ys[i];
            const double ri = std::hypot(dx, dy);
            if (ri < 1e-9) continue;
            const double fi = ri - known_r_m;
            const double jx = dx / ri;
            const double jy = dy / ri;
            h11 += jx * jx;
            h12 += jx * jy;
            h22 += jy * jy;
            g1 += jx * fi;
            g2 += jy * fi;
            ++used;
        }
        if (used < 6) return false;
        const double det = h11 * h22 - h12 * h12;
        if (std::abs(det) < 1e-12) break;
        const double inv11 =  h22 / det;
        const double inv12 = -h12 / det;
        const double inv22 =  h11 / det;
        const double dcx = -(inv11 * g1 + inv12 * g2);
        const double dcy = -(inv12 * g1 + inv22 * g2);
        cx += dcx;
        cy += dcy;
        if (std::hypot(dcx, dcy) < 1e-6) break;
    }

    double sum_sq = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double ri = std::hypot(xs[i] - cx, ys[i] - cy);
        const double dv = ri - known_r_m;
        sum_sq += dv * dv;
    }
    out_cx = cx;
    out_cy = cy;
    out_rmse = std::sqrt(sum_sq / (double)n);
    return std::isfinite(out_cx) && std::isfinite(out_cy) && std::isfinite(out_rmse);
}

/** Largest SAM mask contour lifted to task XY (oblique views → ellipse, not a circle). */
static bool collectMaskContourTaskXY(
    const cv::Mat& mask_u8,
    const rs2::depth_frame& depth,
    const rs2_intrinsics& intr,
    const Eigen::Matrix4d& T_task_cam,
    std::vector<double>& out_xs,
    std::vector<double>& out_ys,
    int& out_n)
{
    out_xs.clear();
    out_ys.clear();
    out_n = 0;
    if (mask_u8.empty() || mask_u8.type() != CV_8UC1) return false;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_u8, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return false;
    size_t bi = 0;
    double best_a = 0.0;
    for (size_t i = 0; i < contours.size(); ++i) {
        const double a = std::abs(cv::contourArea(contours[i]));
        if (a > best_a) {
            best_a = a;
            bi = i;
        }
    }
    const auto& cnt = contours[bi];
    out_xs.reserve(cnt.size());
    out_ys.reserve(cnt.size());
    const int step = std::max(1, (int)cnt.size() / 400);
    for (size_t i = 0; i < cnt.size(); i += (size_t)step) {
        const int u = cnt[i].x;
        const int v = cnt[i].y;
        Eigen::Vector2d xy;
        if (!pixelDepthToTaskXY(u, v, depth, intr, T_task_cam, xy)) continue;
        out_xs.push_back(xy.x());
        out_ys.push_back(xy.y());
    }
    out_n = (int)out_xs.size();
    return out_n >= 6;
}

/**
 * Planar ellipse fit (OpenCV ``fitEllipse`` on task-frame metres). Oblique camera → rim
 * projects as an oval; circle QA is biased — use eccentricity / axis ratio as ground hint.
 * ``angle_deg`` follows OpenCV RotatedRect (degrees, sensor-style ellipse orientation).
 */
static bool ellipseFitTaskXY(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    Eigen::Vector2d& out_center,
    double& out_semi_major_m,
    double& out_semi_minor_m,
    double& out_angle_deg,
    double& out_eccentricity,
    double& out_geom_rmse_m)
{
    const size_t n = xs.size();
    if (n < 6) return false;
    std::vector<cv::Point2f> pts(n);
    for (size_t i = 0; i < n; ++i)
        pts[i] = cv::Point2f((float)xs[i], (float)ys[i]);
    const cv::RotatedRect ell = cv::fitEllipse(pts);
    out_center = Eigen::Vector2d(ell.center.x, ell.center.y);
    const double hw = 0.5 * ell.size.width;
    const double hh = 0.5 * ell.size.height;
    out_semi_major_m = std::max(hw, hh);
    out_semi_minor_m = std::min(hw, hh);
    out_angle_deg = ell.angle;
    const double sma = std::max(out_semi_major_m, 1e-12);
    const double smi = out_semi_minor_m;
    out_eccentricity = std::sqrt(std::max(0.0, 1.0 - (smi * smi) / (sma * sma)));
    const double theta = ell.angle * M_PI / 180.0;
    const double ca = std::cos(theta);
    const double sa = std::sin(theta);
    double sum_sq = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double dx = xs[i] - out_center.x();
        const double dy = ys[i] - out_center.y();
        const double xr = ca * dx + sa * dy;
        const double yr = -sa * dx + ca * dy;
        const double nh = std::sqrt((xr * xr) / (hw * hw) + (yr * yr) / (hh * hh));
        const double res = std::abs(nh - 1.0) * sma;
        sum_sq += res * res;
    }
    out_geom_rmse_m = std::sqrt(sum_sq / (double)n);
    return hw > 1e-9 && hh > 1e-9;
}

static bool taskToPixel(
    const Eigen::Vector3d& p_task,
    const Eigen::Matrix4d& T_task_cam,
    const rs2_intrinsics& intr,
    cv::Point& out_px)
{
    Eigen::Matrix4d T_cam_task = T_task_cam.inverse();
    Eigen::Vector4d p_task_h(p_task.x(), p_task.y(), p_task.z(), 1.0);
    Eigen::Vector4d p_cam_h = T_cam_task * p_task_h;
    if (p_cam_h.z() <= 1e-6) return false;
    float p_cam[3] = {(float)p_cam_h.x(), (float)p_cam_h.y(), (float)p_cam_h.z()};
    float pix[2] = {0.0f, 0.0f};
    rs2_project_point_to_pixel(pix, &intr, p_cam);
    out_px = cv::Point((int)std::lround(pix[0]), (int)std::lround(pix[1]));
    return true;
}

// ---------------------------------------------------------------------------
// Grasp geometry
// ---------------------------------------------------------------------------

static Eigen::Matrix3d hook_rim_rotation(double angle_rad, double tilt_rad=0.0) {
    return Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ()).matrix()
         * Eigen::AngleAxisd(M_PI,      Eigen::Vector3d::UnitX()).matrix()
         * Eigen::AngleAxisd(tilt_rad,  Eigen::Vector3d::UnitY()).matrix();
}
static Eigen::Vector3d to_rotvec(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R); return aa.axis()*aa.angle();
}
struct GraspCandidate { Eigen::Vector3d t,rv,pre_t,pre_rv; double pregrasp_m,force_n; std::string desc; };

static std::vector<GraspCandidate> bowl_hook_candidates(const Eigen::Vector3d& base) {
    std::vector<GraspCandidate> out; out.reserve(N_ANGLES);
    for (int i=0;i<N_ANGLES;i++) {
        double angle = 2.0*M_PI*i/N_ANGLES;
        Eigen::Vector3d rim(BOWL_RIM_OUTER_RADIUS_M*std::cos(angle),
                            BOWL_RIM_OUTER_RADIUS_M*std::sin(angle), BOWL_RIM_Z_OFFSET_M);
        Eigen::Vector3d gt = base+rim;
        Eigen::Matrix3d R  = hook_rim_rotation(angle,0.0);
        Eigen::Vector3d pt = gt - HOOK_RIM_PREGRASP_OFFSET*R.col(2);
        std::ostringstream d; d<<std::fixed; d.precision(0);
        d<<"bowl hook @ "<<(angle*180.0/M_PI)<<"deg";
        out.push_back({gt,to_rotvec(R),pt,to_rotvec(R),HOOK_RIM_PREGRASP_OFFSET,BOWL_GRASP_FORCE,d.str()});
    }
    return out;
}

// ---------------------------------------------------------------------------
// Depth statistics (table corners vs bbox interior; edge cue for rim contact)
// ---------------------------------------------------------------------------

static cv::Rect shrinkRect(cv::Rect b, float margin_frac, int W, int H) {
    int mx = (int)std::lround((double)b.width * margin_frac);
    int my = (int)std::lround((double)b.height * margin_frac);
    mx = std::max(1, mx);
    my = std::max(1, my);
    cv::Rect r(b.x + mx, b.y + my, b.width - 2 * mx, b.height - 2 * my);
    return r & cv::Rect(0, 0, W, H);
}

static bool medianDepthRoi(const rs2::depth_frame& depth, const cv::Rect& r_in,
                           int W, int H, double& out_median) {
    cv::Rect r = r_in & cv::Rect(0, 0, W, H);
    if (r.width <= 0 || r.height <= 0) return false;
    std::vector<float> vals;
    vals.reserve((size_t)r.area());
    for (int v = r.y; v < r.y + r.height; ++v) {
        for (int u = r.x; u < r.x + r.width; ++u) {
            float d = depthDistOfs(depth, u, v);
            if (d > 0.05f && d < 10.0f) vals.push_back(d);
        }
    }
    if (vals.size() < 8u) return false;
    auto mid = vals.begin() + (vals.size() / 2);
    std::nth_element(vals.begin(), mid, vals.end());
    out_median = (double)*mid;
    return true;
}

/** Image corners (away from centre) ≈ table plane when the bowl is central. */
static bool medianTableFromCorners(const rs2::depth_frame& depth, int W, int H,
                                   int roi, double& out_median) {
    std::vector<double> corner_med;
    corner_med.reserve(4);
    const cv::Rect corners[4] = {
        cv::Rect(0, 0, roi, roi),
        cv::Rect(W - roi, 0, roi, roi),
        cv::Rect(0, H - roi, roi, roi),
        cv::Rect(W - roi, H - roi, roi, roi),
    };
    for (const auto& cr : corners) {
        double m = 0.0;
        if (medianDepthRoi(depth, cr, W, H, m)) corner_med.push_back(m);
    }
    if (corner_med.empty()) return false;
    auto mid = corner_med.begin() + (corner_med.size() / 2);
    std::nth_element(corner_med.begin(), mid, corner_med.end());
    out_median = *mid;
    return true;
}

/** Mean gradient magnitude of depth in bbox (rim / discontinuity heuristic). */
static bool depthEdgeMetric(const rs2::depth_frame& depth, const cv::Rect& box_in,
                            int W, int H, double& out_mean) {
    cv::Rect r = box_in & cv::Rect(0, 0, W, H);
    if (r.width < 4 || r.height < 4) return false;
    cv::Mat z(r.height, r.width, CV_32F);
    for (int v = 0; v < r.height; ++v) {
        for (int u = 0; u < r.width; ++u) {
            float d = depthDistOfs(depth, u + r.x, v + r.y);
            z.at<float>(v, u) = (d > 0.05f && d < 10.0f) ? d : 0.0f;
        }
    }
    cv::Mat gx, gy;
    cv::Sobel(z, gx, CV_32F, 1, 0, 3);
    cv::Sobel(z, gy, CV_32F, 0, 1, 3);
    double sum = 0.0;
    int cnt = 0;
    for (int v = 1; v < r.height - 1; ++v) {
        for (int u = 1; u < r.width - 1; ++u) {
            if (z.at<float>(v, u) <= 0.0f) continue;
            float m = std::sqrt(gx.at<float>(v, u) * gx.at<float>(v, u)
                              + gy.at<float>(v, u) * gy.at<float>(v, u));
            sum += (double)m;
            ++cnt;
        }
    }
    if (cnt < 10) return false;
    out_mean = sum / (double)cnt;
    return true;
}

// ---------------------------------------------------------------------------
// JSON
// ---------------------------------------------------------------------------

static std::string v3(const Eigen::Vector3d& v) {
    std::ostringstream ss; ss<<std::fixed; ss.precision(6);
    ss<<"["<<v[0]<<","<<v[1]<<","<<v[2]<<"]"; return ss.str();
}
static void emit_json(std::ostream& os, bool detected, float conf,
                      const Eigen::Vector3d& pos_cam,
                      const Eigen::Vector3d& bowl_base,
                      const Eigen::Vector3d& bowl_rim,
                      const std::vector<GraspCandidate>& cands,
                      bool have_depth_metrics,
                      double table_med_m,
                      double object_med_m,
                      double depth_delta_m,
                      double edge_metric,
                      const std::string& center_method,
                      bool have_mask_refine,
                      const Eigen::Vector3d& bowl_base_before_mask,
                      int mask_planar_n,
                      bool have_mask_pca,
                      double mask_pca_major_rad,
                      double mask_pca_ecc,
                      bool have_circle_fit,
                      const Eigen::Vector2d& circle_center_xy,
                      double circle_r_m,
                      double circle_rmse_m,
                      int circle_pts_n,
                      double circle_radius_minus_nominal_m,
                      bool have_ellipse_fit,
                      const Eigen::Vector2d& ellipse_center_xy,
                      double ellipse_semi_major_m,
                      double ellipse_semi_minor_m,
                      double ellipse_angle_deg,
                      double ellipse_eccentricity,
                      double ellipse_rmse_m,
                      bool planar_oblique_view_hint,
                      bool have_ray_table_xy,
                      const Eigen::Vector2d& ray_table_xy)
{
    os<<"{\n";
    os<<"  \"detected\": "<<(detected?"true":"false")<<",\n";
    if (!detected) { os<<"  \"candidates\": []\n}\n"; return; }
    os<<"  \"confidence\": "<<conf<<",\n";
    os<<"  \"bowl_pos_camera_m\": "<<v3(pos_cam)<<",\n";
    os<<"  \"bowl_base_task_m\": " <<v3(bowl_base)<<",\n";
    os<<"  \"bowl_rim_task_m\": "  <<v3(bowl_rim)<<",\n";
    os<<"  \"bowl_center_method\": \""<<center_method<<"\",\n";
    if (have_ray_table_xy) {
        os<<std::fixed;
        os.precision(8);
        os<<"  \"bowl_center_ray_table_xy_m\": ["<<ray_table_xy.x()<<","<<ray_table_xy.y()<<"],\n";
        Eigen::Vector2d dxy = bowl_base.head<2>() - ray_table_xy;
        os<<"  \"bowl_center_chosen_minus_ray_xy_m\": ["<<dxy.x()<<","<<dxy.y()<<"],\n";
    }
    if (have_mask_refine) {
        os<<"  \"bowl_base_before_mask_refine_m\": "<<v3(bowl_base_before_mask)<<",\n";
        os<<"  \"mask_planar_samples_n\": "<<mask_planar_n<<",\n";
        if (have_mask_pca) {
            os<<std::fixed;
            os.precision(8);
            os<<"  \"mask_planar_pca_major_rad_task\": "<<mask_pca_major_rad<<",\n";
            os<<"  \"mask_planar_eccentricity\": "<<mask_pca_ecc<<",\n";
        }
    }
    if (have_circle_fit) {
        os<<std::fixed;
        os.precision(8);
        os<<"  \"bowl_circle_fit_center_xy_task_m\": ["<<circle_center_xy.x()<<","
          <<circle_center_xy.y()<<"],\n";
        os<<"  \"bowl_circle_fit_radius_m\": "<<circle_r_m<<",\n";
        os<<"  \"bowl_circle_fit_rmse_m\": "<<circle_rmse_m<<",\n";
        os<<"  \"bowl_circle_fit_points_n\": "<<circle_pts_n<<",\n";
        os<<"  \"bowl_circle_fit_radius_minus_nominal_m\": "<<circle_radius_minus_nominal_m<<",\n";
    }
    if (have_ellipse_fit) {
        os<<std::fixed;
        os.precision(8);
        os<<"  \"bowl_ellipse_fit_center_xy_task_m\": ["<<ellipse_center_xy.x()<<","
          <<ellipse_center_xy.y()<<"],\n";
        os<<"  \"bowl_ellipse_semi_major_m\": "<<ellipse_semi_major_m<<",\n";
        os<<"  \"bowl_ellipse_semi_minor_m\": "<<ellipse_semi_minor_m<<",\n";
        os<<"  \"bowl_ellipse_angle_deg_opencv\": "<<ellipse_angle_deg<<",\n";
        os<<"  \"bowl_ellipse_eccentricity\": "<<ellipse_eccentricity<<",\n";
        os<<"  \"bowl_ellipse_rmse_m\": "<<ellipse_rmse_m<<",\n";
        os<<"  \"planar_oblique_view_hint\": "<<(planar_oblique_view_hint?"true":"false")<<",\n";
    }
    if (have_depth_metrics) {
        os<<std::fixed;
        os.precision(6);
        os<<"  \"table_median_depth_m\": "<<table_med_m<<",\n";
        os<<"  \"object_median_depth_m\": "<<object_med_m<<",\n";
        os<<"  \"depth_delta_table_minus_object_m\": "<<depth_delta_m<<",\n";
        os<<"  \"depth_edge_metric_mean\": "<<edge_metric<<",\n";
    }
    os<<"  \"candidates\": [\n";
    for (int i=0;i<(int)cands.size();i++) {
        const auto& g=cands[i];
        os<<"    {"
          <<"\"index\":"<<i<<","
          <<"\"description\":\""<<g.desc<<"\","
          <<"\"translation\":"<<v3(g.t)<<","
          <<"\"rotvec\":"<<v3(g.rv)<<","
          <<"\"pre_translation\":"<<v3(g.pre_t)<<","
          <<"\"pre_rotvec\":"<<v3(g.pre_rv)<<","
          <<"\"pregrasp_offset_m\":"<<g.pregrasp_m<<","
          <<"\"grasp_force_n\":"<<g.force_n<<"}";
        if (i+1<(int)cands.size()) os<<",";
        os<<"\n";
    }
    os<<"  ]\n}\n";
}

// ---------------------------------------------------------------------------
// Annotate + save frame to JPEG
// ---------------------------------------------------------------------------

static void save_annotated_frame(
    const cv::Mat& bgr,
    const cv::Rect& box, float conf,
    const Eigen::Vector3d& base_task,
    const Eigen::Vector3d& rim_task,
    const cv::Mat& sam_mask,
    const cv::Point& sam_centroid_px,
    bool sam_used,
    const cv::Point& planned_grasp_px,
    const cv::Point& planned_pregrasp_px,
    bool planned_grasp_ok,
    bool planned_pregrasp_ok,
    bool detected,
    const std::string& save_path)
{
    cv::Mat disp = bgr.clone();

    auto put = [&](const std::string& txt, int row, cv::Scalar col={0,255,0}) {
        cv::putText(disp, txt, cv::Point(10, 28+row*26),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0,0,0), 3);
        cv::putText(disp, txt, cv::Point(10, 28+row*26),
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, col, 2);
    };

    if (detected) {
        if (sam_used && !sam_mask.empty()) {
            cv::Mat green(disp.size(), disp.type(), cv::Scalar(0, 180, 0));
            cv::Mat blended;
            cv::addWeighted(disp, 0.65, green, 0.35, 0.0, blended);
            blended.copyTo(disp, sam_mask);
            cv::circle(disp, sam_centroid_px, 5, cv::Scalar(255, 0, 255), 2);
            put("SAM2 segmentation: ON (depth @ mask)", 2, cv::Scalar(255, 160, 0));
        } else {
            put("SAM2 segmentation: OFF (YOLO bbox depth)", 2, cv::Scalar(80, 180, 255));
        }

        cv::rectangle(disp, box, cv::Scalar(0,255,0), 2);
        if (planned_grasp_ok) {
            cv::circle(disp, planned_grasp_px, 6, cv::Scalar(0, 0, 255), 2);
            cv::putText(disp, "grasp", planned_grasp_px + cv::Point(8, -6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 2);
        }
        if (planned_pregrasp_ok) {
            cv::circle(disp, planned_pregrasp_px, 6, cv::Scalar(255, 255, 0), 2);
            cv::putText(disp, "pregrasp", planned_pregrasp_px + cv::Point(8, -6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 0), 2);
            if (planned_grasp_ok) {
                cv::line(disp, planned_pregrasp_px, planned_grasp_px, cv::Scalar(255, 255, 0), 1);
            }
        }

        std::ostringstream ct; ct<<std::fixed; ct.precision(2);
        ct<<"bowl  conf="<<conf;
        cv::putText(disp, ct.str(),
                    cv::Point(box.x, std::max(0, box.y-8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0,255,0), 2);

        std::ostringstream b, r;
        b<<std::fixed; b.precision(3);
        r<<std::fixed; r.precision(3);
        b<<"base_task [m]: "<<base_task[0]<<", "<<base_task[1]<<", "<<base_task[2];
        r<<"rim_task  [m]: "<<rim_task[0] <<", "<<rim_task[1] <<", "<<rim_task[2];
        put(b.str(), 0);
        put(r.str(), 1);
        put("BOWL DETECTED", 3, cv::Scalar(0,200,255));
    } else {
        put("No bowl detected", 0, cv::Scalar(0,80,255));
    }

    if (!cv::imwrite(save_path, disp))
        std::cerr << "[WARN] Could not save image to: " << save_path << "\n";
    else
        std::cerr << "[PREVIEW] Annotated frame saved to: " << save_path << "\n";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::string model_path   = "../../../../212_Perception/build/yolo11n.onnx";
    std::string classes_path = "../../../../212_Perception/build/coco-classes.txt";
    std::string save_image   = "/tmp/hook_grasp_testing_preview.jpg";
    std::string camera_serial = "";
    std::string sam_encoder_path = "";
    std::string sam_decoder_path = "";
    double grasp_angle_deg = 180.0;
    double approach_tilt_deg = 10.0;
    /** Expected outer rim radius [m] for geometry QA — align with ``grasps/bowl.py`` / Python flags. */
    double nominal_rim_radius_m = 0.037;
    float conf_threshold     = 0.40f;
    bool  headless           = false;
    bool  no_sam             = false;
    int warmup_frames        = DEFAULT_N_WARMUP;
    int detect_frames        = DEFAULT_N_FRAMES;

    Eigen::Matrix4d T_task_cam = Eigen::Matrix4d::Identity();

    for (int i=1;i<argc;i++) {
        std::string a(argv[i]);
        if      (a=="--model"   && i+1<argc) { model_path   = argv[++i]; }
        else if (a=="--classes" && i+1<argc) { classes_path = argv[++i]; }
        else if (a=="--conf"    && i+1<argc) { conf_threshold = std::stof(argv[++i]); }
        else if (a=="--save-image" && i+1<argc) { save_image = argv[++i]; }
        else if (a=="--serial"  && i+1<argc) { camera_serial = argv[++i]; }
        else if (a=="--no-sam")             { no_sam = true; }
        else if (a=="--sam-encoder" && i+1<argc) { sam_encoder_path = argv[++i]; }
        else if (a=="--sam-decoder" && i+1<argc) { sam_decoder_path = argv[++i]; }
        else if (a=="--grasp-angle-deg" && i+1<argc) { grasp_angle_deg = std::stod(argv[++i]); }
        else if (a=="--approach-tilt-deg" && i+1<argc) { approach_tilt_deg = std::stod(argv[++i]); }
        else if (a=="--nominal-rim-radius-m" && i+1<argc) { nominal_rim_radius_m = std::stod(argv[++i]); }
        else if (a=="--depth-color-offset-pix" && i+2<argc) {
            g_depth_color_off_u = std::stoi(argv[++i]);
            g_depth_color_off_v = std::stoi(argv[++i]);
        }
        else if (a=="--warmup"  && i+1<argc) { warmup_frames = std::max(0, std::stoi(argv[++i])); }
        else if (a=="--frames"  && i+1<argc) { detect_frames = std::max(1, std::stoi(argv[++i])); }
        else if (a=="--headless")             { headless = true; }
        else if (a=="--T-task-camera" && i+1<argc) {
            std::istringstream ss(argv[++i]);
            bool ok=true;
            for (int r=0;r<4&&ok;r++)
                for (int c=0;c<4&&ok;c++) ok=!!(ss>>T_task_cam(r,c));
            if (!ok) { std::cerr<<"[ERROR] --T-task-camera needs 16 floats\n"; return 1; }
        }
    }

    std::cerr<<"[INFO] T_task_camera:\n"<<T_task_cam<<"\n";
    std::cerr<<"[INFO] Mode: "<<(headless?"headless":"preview (saves JPEG)")<<"\n";
    if (g_depth_color_off_u != 0 || g_depth_color_off_v != 0)
        std::cerr << "[INFO] depth-color pixel offset (aligned-depth sampling): du=" << g_depth_color_off_u
                  << " dv=" << g_depth_color_off_v
                  << "  (use RealSense Viewer overlay to tune residual RGB–depth)\n";

#if defined(HAVE_ONNXRUNTIME)
    Sam2Segmenter sam2;
    bool sam2_loaded = false;
    const bool want_sam = !no_sam && !sam_encoder_path.empty() && !sam_decoder_path.empty();
    if (want_sam) {
        try {
            sam2_loaded = sam2.load(sam_encoder_path, sam_decoder_path);
            if (sam2_loaded) std::cerr << "[INFO] SAM2 ONNX loaded — segmentation refines YOLO box\n";
            else std::cerr << "[WARN] SAM2 load failed — using YOLO bbox geometry only\n";
        } catch (const std::exception& e) {
            std::cerr << "[WARN] SAM2 init failed (" << e.what() << ") — YOLO-only fallback\n";
            sam2_loaded = false;
        }
    } else if (!no_sam && (sam_encoder_path.empty() ^ sam_decoder_path.empty())) {
        std::cerr << "[WARN] Provide both --sam-encoder and --sam-decoder, or neither. SAM disabled.\n";
    } else if (!no_sam && sam_encoder_path.empty()) {
        std::cerr << "[INFO] SAM2 paths not passed — YOLO-only geometry (pass encoder+decoder to enable SAM)\n";
    }
#else
    bool sam2_loaded = false;
    if (!no_sam && !sam_encoder_path.empty() && !sam_decoder_path.empty()) {
        std::cerr << "[WARN] SAM2 ONNX paths passed but this binary was built without ONNX Runtime. "
                  << "Rebuild with -DONNXRUNTIME_ROOT=...  YOLO-only geometry.\n";
    }
#endif

    YOLODetector detector;
    if (!detector.loadModel(model_path, classes_path)) return 1;

    rs2::pipeline pipe;
    rs2::config   cfg;
    if (!camera_serial.empty()) cfg.enable_device(camera_serial);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16,  30);

    rs2::spatial_filter      spat; spat.set_option(RS2_OPTION_FILTER_MAGNITUDE,2);
                                   spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.5f);
                                   spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,20);
    rs2::temporal_filter     temp; temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,0.4f);
                                   temp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,20);
    rs2::hole_filling_filter hole;

    rs2::pipeline_profile profile;
    try { profile = pipe.start(cfg); }
    catch (const rs2::error& e) { std::cerr<<"[ERROR] RealSense: "<<e.what()<<"\n"; return 1; }

    rs2::align align_to_color(RS2_STREAM_COLOR);
    // After align_to_color, depth pixels map 1:1 to color pixels — use COLOR intrinsics.
    rs2_intrinsics intr = profile.get_stream(RS2_STREAM_COLOR)
                                 .as<rs2::video_stream_profile>().get_intrinsics();

    std::cerr<<"[INFO] Warming up ("<<warmup_frames<<" frames)...\n";
    for (int i=0;i<warmup_frames;i++) pipe.wait_for_frames();

    // Detection loop — keep best across detect_frames
    std::cerr<<"[INFO] Detecting over "<<detect_frames<<" frames...\n";
    bool            found      = false;
    float           best_conf  = 0.0f;
    cv::Rect        best_box;
    cv::Mat         best_frame;
    rs2::depth_frame best_depth{nullptr};

    for (int fi=0;fi<detect_frames;fi++) {
        rs2::frameset fs  = pipe.wait_for_frames();
        rs2::frameset afs = align_to_color.process(fs);
        auto cf = afs.get_color_frame();
        rs2::depth_frame df = afs.get_depth_frame();
        df = spat.process(df); df = temp.process(df); df = hole.process(df);

        cv::Mat bgr(cv::Size(cf.get_width(),cf.get_height()),
                    CV_8UC3,(void*)cf.get_data(),cv::Mat::AUTO_STEP);

        float this_conf=0.0f; cv::Rect this_box;
        if (detector.runFrame(bgr, conf_threshold, this_conf, this_box)) {
            // Sanity: verify detection has plausible depth (not a background FP)
            if (detector.depthSanity(this_box, df)) {
                if (!found || this_conf > best_conf) {
                    best_conf  = this_conf;
                    best_box   = this_box;
                    best_frame = bgr.clone();
                    best_depth = df;
                    found      = true;
                }
            }
        }
        if (!found) best_frame = bgr.clone();
    }
    pipe.stop();

    // Compute bowl position via depth deprojection: sample the median depth
    // at the bbox centre, deproject directly to 3D in camera frame, then
    // transform to task frame.  This uses the actual depth reading rather
    // than tracing a ray to an assumed z=0 plane, so calibration angular
    // errors are not amplified by the camera-to-table distance.
    Eigen::Vector3d bowl_base_task(0,0,0), bowl_rim_task(0,0,BOWL_RIM_Z_OFFSET_M);
    Eigen::Vector3d pos_cam(0,0,0);  // kept for JSON completeness only
    bool have_depth_metrics = false;
    double table_med_m = 0, object_med_m = 0, depth_delta_m = 0, edge_metric = 0;

    std::string json_center_method = "depth_patch_centroid";
    bool json_have_mask_refine = false;
    Eigen::Vector3d json_bowl_before_refine(0, 0, 0);
    int json_mask_planar_n = 0;
    bool json_have_mask_pca = false;
    double json_mask_pca_major_rad = 0.0;
    double json_mask_pca_ecc = 0.0;
    bool json_have_circle_fit = false;
    Eigen::Vector2d json_circle_c(0, 0);
    double json_circle_r = 0.0;
    double json_circle_rmse = 0.0;
    double json_circle_dr_nom = 0.0;
    int json_circle_n = 0;
    bool json_have_ellipse_fit = false;
    Eigen::Vector2d json_ellipse_c(0, 0);
    double json_ellipse_smaj = 0.0;
    double json_ellipse_smin = 0.0;
    double json_ellipse_ang_deg = 0.0;
    double json_ellipse_ecc = 0.0;
    double json_ellipse_rmse = 0.0;
    bool json_planar_oblique_hint = false;
    bool json_have_ray_table_xy = false;
    Eigen::Vector2d json_ray_table_xy(0, 0);
    if (found) {
        std::cerr<<"[INFO] Bowl detected  conf="<<best_conf
                 <<"  bbox=["<<best_box.x<<","<<best_box.y
                 <<" "<<best_box.width<<"x"<<best_box.height<<"]\n";

        int cx = best_box.x + best_box.width / 2;
        cv::Mat sam_mask;
        cv::Point2f sam_centroid((float)cx, (float)(best_box.y + best_box.height / 2));
        bool sam_refined = false;
#if defined(HAVE_ONNXRUNTIME)
        if (sam2_loaded) {
            if (sam2.refineByBox(best_frame, best_box, sam_mask, sam_centroid)) {
                sam_refined = true;
                cx = std::clamp((int)std::lround(sam_centroid.x), 0, best_frame.cols - 1);
            } else {
                std::cerr << "[WARN] SAM2 refine failed; using YOLO bbox for XY depth sample.\n";
            }
        }
#endif
        int cy_bot = best_box.y + best_box.height - std::max(1, best_box.height / 10);
        int cy_ctr = best_box.y + best_box.height / 2;
        int cy_sam = std::clamp((int)std::lround(sam_centroid.y), 0, best_frame.rows - 1);
        int cy_sample = sam_refined ? cy_sam : cy_bot;
        bool ok3d = depthTo3DTask(best_box, cx, cy_sample, best_depth, intr, T_task_cam, bowl_base_task);
        if (!ok3d && sam_refined) {
            std::cerr << "[WARN] Depth at SAM centroid failed; trying YOLO bottom edge.\n";
            ok3d = depthTo3DTask(best_box, cx, cy_bot, best_depth, intr, T_task_cam, bowl_base_task);
        }
        if (!ok3d) {
            std::cerr<<"[WARN] Bottom-edge depth sample failed, trying bbox centre.\n";
            ok3d = depthTo3DTask(best_box, cx, cy_ctr, best_depth, intr, T_task_cam, bowl_base_task);
        }
        if (!ok3d) {
            std::cerr<<"[WARN] Depth-to-3D failed (no valid depth readings). "
                     <<"Check camera range and scene coverage.\n";
            found = false;
        } else {
            json_bowl_before_refine = bowl_base_task;
            if (tablePlaneXYFromCentroidRay(
                    best_box, cx, cy_sample, best_depth, intr, T_task_cam, 0.0, json_ray_table_xy)) {
                json_have_ray_table_xy = true;
                std::cerr << "[INFO] Centroid ray ∩ plane z=0 (task XY) [m]: "
                          << json_ray_table_xy.transpose()
                          << "  (depth→range along ray; table plane fixes XY on that ray)\n";
                // For table-top bowls, prefer the table-plane ray intersection as the
                // baseline XY estimate before any mask-based refinement.
                bowl_base_task.x() = json_ray_table_xy.x();
                bowl_base_task.y() = json_ray_table_xy.y();
                bowl_base_task.z() = 0.0;
                json_center_method = "depth_patch_ray_table_xy";
            }
#if defined(HAVE_ONNXRUNTIME)
            if (sam_refined && !sam_mask.empty()) {
                std::vector<double> contour_xs, contour_ys;
                int contour_n = 0;
                const bool have_contour_xy = collectMaskContourTaskXY(
                    sam_mask, best_depth, intr, T_task_cam, contour_xs, contour_ys, contour_n);

                Eigen::Vector2d circle_c(0, 0);
                double circle_r = 0.0, circle_rmse = 0.0;
                bool circle_geom_ok = false;
                if (have_contour_xy && contour_n >= 24)
                    circle_geom_ok = kasaCircleFit(
                        contour_xs, contour_ys, circle_c.x(), circle_c.y(), circle_r, circle_rmse);
                Eigen::Vector2d fixed_c(0, 0);
                double fixed_rmse = 0.0;
                bool fixed_center_ok = false;
                if (have_contour_xy && contour_n >= 24)
                    fixed_center_ok = fixedRadiusCenterFit(
                        contour_xs, contour_ys, nominal_rim_radius_m, fixed_c.x(), fixed_c.y(), fixed_rmse);

                Eigen::Vector2d ell_c(0, 0);
                double ell_smaj = 0.0, ell_smin = 0.0, ell_ang = 0.0, ell_ecc = 0.0, ell_rmse = 0.0;
                bool ellipse_ok = false;
                if (have_contour_xy && contour_n >= 6)
                    ellipse_ok = ellipseFitTaskXY(
                        contour_xs, contour_ys, ell_c, ell_smaj, ell_smin, ell_ang, ell_ecc, ell_rmse);

                const double dr_nom = circle_geom_ok ? (circle_r - nominal_rim_radius_m) : 0.0;
                if (circle_geom_ok) {
                    json_have_circle_fit = true;
                    json_circle_c = circle_c;
                    json_circle_r = circle_r;
                    json_circle_rmse = circle_rmse;
                    json_circle_n = contour_n;
                    json_circle_dr_nom = dr_nom;
                    std::cerr << "[INFO] Mask contour circle fit (task XY): r=" << circle_r << " m  RMSE="
                              << circle_rmse << " m  n=" << contour_n
                              << "  (r−nominal)=" << dr_nom << " m (nominal=" << nominal_rim_radius_m << ")\n";
                }
                if (fixed_center_ok) {
                    std::cerr << "[INFO] Fixed-R center fit (task XY): R=" << nominal_rim_radius_m
                              << " m  RMSE=" << fixed_rmse << " m  n=" << contour_n << "\n";
                }
                if (ellipse_ok) {
                    json_have_ellipse_fit = true;
                    json_ellipse_c = ell_c;
                    json_ellipse_smaj = ell_smaj;
                    json_ellipse_smin = ell_smin;
                    json_ellipse_ang_deg = ell_ang;
                    json_ellipse_ecc = ell_ecc;
                    json_ellipse_rmse = ell_rmse;
                    json_planar_oblique_hint = ell_ecc >= 0.20;
                    std::cerr << "[INFO] Mask contour ellipse fit (task XY): a=" << ell_smaj << " m  b="
                              << ell_smin << " m  ecc=" << ell_ecc << "  RMSE=" << ell_rmse << " m  angle_deg="
                              << ell_ang << "\n";
                    if (json_planar_oblique_hint)
                        std::cerr << "[INFO] Planar footprint is elongated — oblique view / oval rim "
                                     "(circle nominal-R check is weak here; prefer ellipse metrics).\n";
                }

                Eigen::Vector3d bowl_median = bowl_base_task;
                std::string mmethod;
                int mn = 0;
                double pmaj = 0.0, pecc = 0.0;
                bool hpca = false;
                const bool median_ok = refineBowlBaseFromMaskPlanar(
                    sam_mask, best_depth, intr, T_task_cam, bowl_median,
                    mmethod, mn, pmaj, pecc, hpca);
                static constexpr double kMaxRmseUseCircle = 0.012;
                static constexpr double kMaxRmseUseFixedCenter = 0.012;
                static constexpr double kMaxDrNom = 0.035;
                static constexpr double kMinR = 0.020;
                static constexpr double kMaxR = 0.120;
                static constexpr double kObliqueEccForEllipseCenter = 0.22;
                static constexpr double kMaxRmseEllipseCenter = 0.014;
                static constexpr double kMaxMedianShiftFromSeedM = 0.050;
                static constexpr double kMaxMedianShiftFromRayM = 0.050;
                const bool use_circle = circle_geom_ok && circle_rmse < kMaxRmseUseCircle
                    && circle_r > kMinR && circle_r < kMaxR
                    && std::abs(dr_nom) < kMaxDrNom;
                const bool use_fixed_center = fixed_center_ok && fixed_rmse < kMaxRmseUseFixedCenter;
                const bool use_ellipse_center = !use_circle && !use_fixed_center
                    && ellipse_ok && ell_ecc >= kObliqueEccForEllipseCenter
                    && ell_rmse < kMaxRmseEllipseCenter && contour_n >= 24;
                const bool geom_implausible = (circle_geom_ok && std::abs(dr_nom) > 0.10)
                    || (ellipse_ok && ell_smaj > (4.0 * nominal_rim_radius_m));
                if (use_fixed_center) {
                    bowl_base_task.x() = fixed_c.x();
                    bowl_base_task.y() = fixed_c.y();
                    bowl_base_task.z() = 0.0;
                    json_center_method = "mask_contour_fixed_radius_center";
                    json_have_mask_refine = true;
                    json_mask_planar_n = mn;
                    json_mask_pca_major_rad = pmaj;
                    json_mask_pca_ecc = pecc;
                    json_have_mask_pca = hpca;
                    std::cerr << "[INFO] Bowl XY: using fixed-radius contour center (known rim prior).\n";
                } else if (use_circle) {
                    bowl_base_task.x() = circle_c.x();
                    bowl_base_task.y() = circle_c.y();
                    bowl_base_task.z() = 0.0;
                    json_center_method = "mask_contour_circle_fit";
                    json_have_mask_refine = true;
                    json_mask_planar_n = mn;
                    json_mask_pca_major_rad = pmaj;
                    json_mask_pca_ecc = pecc;
                    json_have_mask_pca = hpca;
                    std::cerr << "[INFO] Bowl XY: using contour circle center (geometry matched nominal R).\n";
                } else if (use_ellipse_center) {
                    bowl_base_task.x() = ell_c.x();
                    bowl_base_task.y() = ell_c.y();
                    bowl_base_task.z() = 0.0;
                    json_center_method = "mask_contour_ellipse_center_oblique";
                    json_have_mask_refine = true;
                    json_mask_planar_n = mn;
                    json_mask_pca_major_rad = pmaj;
                    json_mask_pca_ecc = pecc;
                    json_have_mask_pca = hpca;
                    std::cerr << "[INFO] Bowl XY: using ellipse center (oblique view — circle QA skipped).\n";
                } else if (median_ok) {
                    const Eigen::Vector2d seed_xy = json_bowl_before_refine.head<2>();
                    const Eigen::Vector2d med_xy = bowl_median.head<2>();
                    const double d_seed = (med_xy - seed_xy).norm();
                    const double d_ray = json_have_ray_table_xy
                        ? (med_xy - json_ray_table_xy).norm()
                        : 0.0;
                    const bool median_plausible =
                        (d_seed <= kMaxMedianShiftFromSeedM) &&
                        (!json_have_ray_table_xy || d_ray <= kMaxMedianShiftFromRayM) &&
                        !geom_implausible;
                    if (median_plausible) {
                        bowl_base_task = bowl_median;
                        json_center_method = mmethod;
                        json_have_mask_refine = true;
                        json_mask_planar_n = mn;
                        json_mask_pca_major_rad = pmaj;
                        json_mask_pca_ecc = pecc;
                        json_have_mask_pca = hpca;
                        std::cerr << "[INFO] Bowl XY refined from SAM mask (median planar): n=" << mn;
                        if (hpca)
                            std::cerr << "  planar_PCA_ecc=" << pecc
                                      << "  major_rad_task=" << pmaj;
                        std::cerr << "\n";
                    } else {
                        std::cerr << "[WARN] Rejecting SAM planar median center: "
                                  << "shift_from_seed=" << d_seed << " m";
                        if (json_have_ray_table_xy)
                            std::cerr << "  shift_from_ray=" << d_ray << " m";
                        if (geom_implausible)
                            std::cerr << "  (implausible contour geometry)";
                        std::cerr << ". Keeping depth/ray-based center.\n";
                    }
                    if (circle_geom_ok && !use_circle && !use_fixed_center)
                        std::cerr << "[WARN] Circle fit did not match nominal rim (RMSE or Δr).\n";
                }
            }
#endif
            bowl_rim_task = bowl_base_task;
            bowl_rim_task.z() = BOWL_RIM_Z_OFFSET_M;
            std::cerr<<"[INFO] Bowl base task ("<<bowl_base_task.transpose()<<") m\n";
            std::cerr<<"[INFO] Bowl rim  task ("<<bowl_rim_task.transpose()<<") m\n";

            {
                const int Wm = best_frame.cols, Hm = best_frame.rows;
                double tmed = 0, omed = 0, edg = 0;
                const bool ok_t = medianTableFromCorners(best_depth, Wm, Hm, 48, tmed);
                const cv::Rect inner = shrinkRect(best_box, 0.12f, Wm, Hm);
                const bool ok_o = medianDepthRoi(best_depth, inner, Wm, Hm, omed);
                const bool ok_e = depthEdgeMetric(best_depth, best_box, Wm, Hm, edg);
                if (ok_t && ok_o) {
                    have_depth_metrics = true;
                    table_med_m      = tmed;
                    object_med_m     = omed;
                    depth_delta_m    = tmed - omed;
                    edge_metric      = ok_e ? edg : 0.0;
                    std::cerr << "[INFO] depth: table_med=" << tmed << " m  object_med=" << omed
                              << " m  delta(table-object)=" << (tmed - omed) << " m";
                    if (ok_e) std::cerr << "  edge_metric=" << edg;
                    std::cerr << "\n";
                } else {
                    std::cerr << "[WARN] Table vs bbox interior depth metrics unavailable.\n";
                }
            }

            // Save annotated JPEG (unless headless)
            if (!headless && !best_frame.empty()) {
                const double angle_rad = grasp_angle_deg * M_PI / 180.0;
                const double tilt_rad = approach_tilt_deg * M_PI / 180.0;
                Eigen::Vector3d rim(
                    BOWL_RIM_OUTER_RADIUS_M * std::cos(angle_rad),
                    BOWL_RIM_OUTER_RADIUS_M * std::sin(angle_rad),
                    BOWL_RIM_Z_OFFSET_M);
                Eigen::Vector3d grasp_task = bowl_base_task + rim;
                Eigen::Matrix3d Rg = hook_rim_rotation(angle_rad, tilt_rad);
                Eigen::Vector3d pregrasp_task = grasp_task - HOOK_RIM_PREGRASP_OFFSET * Rg.col(2);
                cv::Point grasp_px, pregrasp_px;
                bool grasp_ok = taskToPixel(grasp_task, T_task_cam, intr, grasp_px);
                bool pregrasp_ok = taskToPixel(pregrasp_task, T_task_cam, intr, pregrasp_px);
                save_annotated_frame(
                    best_frame, best_box, best_conf,
                    bowl_base_task, bowl_rim_task,
                    sam_mask, cv::Point((int)std::lround(sam_centroid.x), (int)std::lround(sam_centroid.y)),
                    sam_refined,
                    grasp_px, pregrasp_px, grasp_ok, pregrasp_ok,
                    true, save_image
                );
            }
        }
    } else {
        std::cerr<<"[WARN] No bowl detected (conf≥"<<conf_threshold<<").\n";
        if (!headless && !best_frame.empty()) {
            save_annotated_frame(best_frame, best_box, best_conf,
                                 bowl_base_task, bowl_rim_task,
                                 cv::Mat(), cv::Point(), false,
                                 cv::Point(), cv::Point(), false, false,
                                 false, save_image);
        }
    }

    // Emit JSON
    auto cands = found ? bowl_hook_candidates(bowl_base_task) : std::vector<GraspCandidate>{};
    emit_json(std::cout, found, best_conf, pos_cam, bowl_base_task, bowl_rim_task, cands,
              have_depth_metrics, table_med_m, object_med_m, depth_delta_m, edge_metric,
              json_center_method, json_have_mask_refine, json_bowl_before_refine,
              json_mask_planar_n, json_have_mask_pca, json_mask_pca_major_rad,
              json_mask_pca_ecc,
              json_have_circle_fit, json_circle_c, json_circle_r, json_circle_rmse,
              json_circle_n, json_circle_dr_nom,
              json_have_ellipse_fit, json_ellipse_c, json_ellipse_smaj, json_ellipse_smin,
              json_ellipse_ang_deg, json_ellipse_ecc, json_ellipse_rmse,
              json_planar_oblique_hint,
              json_have_ray_table_xy, json_ray_table_xy);
    return 0;
}
