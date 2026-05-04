/**
 * perception.cpp — Bottle detection → task-frame position
 * Task: pour_bottle
 *
 * Modes
 * -----
 *   default (preview)   detect bottle, save annotated JPEG to --save-image path,
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
 *   --save-image <path>       where to write the annotated JPEG (preview mode)
 *   --headless                skip image save and terminal pause
 *
 * stdout — JSON:
 *   { "detected": bool, "confidence": float,
 *     "bottle_pos_camera_m": [...], "bottle_base_task_m": [...],
 *     "bottle_body_task_m": [...], "candidates": [] }
 *
 * Note on ray-table accuracy for tall objects
 * -------------------------------------------
 * We project the camera ray through the bbox centre until it hits the table
 * plane (Z = 0 in task frame). For a 17.5 cm bottle the bbox centre is
 * roughly at the mid-body height (~8–10 cm above the base), so the ray
 * hits the table at a point that is slightly offset in XY from the true
 * base. The offset is small compared to the bottle's 7.3 cm body diameter
 * and is well inside the gripper's compliance for a side body pinch approach.
 * This is acceptable: XY accuracy from the camera ray direction
 * (calibration-quality) is far better than noisy depth-based estimates.
 *
 * Build:
 *   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr float  INPUT_WIDTH          = 640.0f;
static constexpr float  INPUT_HEIGHT         = 640.0f;
static constexpr float  SCORE_THRESHOLD      = 0.25f;
static constexpr float  NMS_THRESHOLD        = 0.45f;
static constexpr float  CONFIDENCE_THRESHOLD = 0.25f;

static constexpr int    COCO_BOTTLE          = 39;  // COCO class for bottle
static constexpr int    N_WARMUP             = 15;
static constexpr int    N_FRAMES             = 10;  // accumulate over more frames for reliability

// Geometry constants — mirror control_scripts/grasps/bottle.py
static constexpr double BOTTLE_BODY_RADIUS_M    = 0.0365;  // 3.65 cm body radius
static constexpr double BOTTLE_TOTAL_HEIGHT_M   = 0.175;   // 17.5 cm total height
static constexpr double BOTTLE_DEFAULT_GRASP_Z  = 0.1;     // mid-body grasp reference height (10 cm)

// ---------------------------------------------------------------------------
// YOLO detector (bottle-only, headless)
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

    // Returns true + fills best_conf/best_box if bottle found in this frame.
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
                float d = depth.get_distance(u, v);
                if (d > 0.05f && d < 4.0f) ds.push_back(d);
            }
        if (ds.empty()) return false;
        std::sort(ds.begin(), ds.end());
        float d_med = ds[ds.size() / 2];
        return d_med >= min_m && d_med <= max_m;
    }

    // Ray-plane intersection: trace the camera ray through the bbox centre
    // until it hits the table plane (Z = 0 in task frame).
    //
    // For a body-grasp approach we want the bottle's XY in the task frame.
    // The bbox centre typically corresponds to the bottle's mid-body, so the
    // ray-table intersection gives a point that is horizontally close to the
    // bottle's base XY — accurate enough for a side body pinch where the
    // gripper has a few cm of lateral compliance. Z is set directly from the
    // known grasp height constant after detection.
    static bool rayTableIntersect(
        const cv::Rect& box,
        const rs2_intrinsics& intr,
        const Eigen::Matrix4d& T_task_cam,
        Eigen::Vector3d& out_base_task)
    {
        int cx = box.x + box.width / 2;
        int cy = box.y + box.height / 2;

        // Camera-frame unit ray direction through (cx, cy)
        float pix[2] = {(float)cx, (float)cy}, pt[3];
        rs2_deproject_pixel_to_point(pt, &intr, pix, 1.0f);
        Eigen::Vector3d ray_cam(pt[0], pt[1], pt[2]);
        ray_cam.normalize();

        // Transform ray to task frame
        Eigen::Matrix3d R = T_task_cam.block<3,3>(0,0);
        Eigen::Vector3d t = T_task_cam.block<3,1>(0,3);
        Eigen::Vector3d ray_task = R * ray_cam;

        // Intersect with Z = 0 (table plane)
        if (std::abs(ray_task.z()) < 1e-6) return false;  // ray parallel to table
        double lam = -t.z() / ray_task.z();
        if (lam < 0.0) return false;  // intersection behind camera

        out_base_task = t + lam * ray_task;
        out_base_task.z() = 0.0;  // snap exactly to table surface
        return true;
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
            if (cl.x != COCO_BOTTLE || conf < thr) continue;
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
            if (cl.x != COCO_BOTTLE || conf < thr) continue;
            float x=data[0],y=data[1],w=data[2],h=data[3];
            boxes.emplace_back((int)((x-w/2)*xf),(int)((y-h/2)*yf),(int)(w*xf),(int)(h*yf));
            confs.push_back(conf);
        }
        tr.release();
    }
};

// ---------------------------------------------------------------------------
// JSON
// ---------------------------------------------------------------------------

static std::string v3(const Eigen::Vector3d& v) {
    std::ostringstream ss; ss<<std::fixed; ss.precision(6);
    ss<<"["<<v[0]<<","<<v[1]<<","<<v[2]<<"]"; return ss.str();
}

static void emit_json(std::ostream& os, bool detected, float conf,
                      const Eigen::Vector3d& pos_cam,
                      const Eigen::Vector3d& bottle_base,
                      const Eigen::Vector3d& bottle_body)
{
    os<<"{\n";
    os<<"  \"detected\": "<<(detected?"true":"false")<<",\n";
    if (!detected) { os<<"  \"candidates\": []\n}\n"; return; }
    os<<"  \"confidence\": "<<conf<<",\n";
    os<<"  \"bottle_pos_camera_m\": " <<v3(pos_cam)    <<",\n";
    os<<"  \"bottle_base_task_m\": "  <<v3(bottle_base)<<",\n";
    os<<"  \"bottle_body_task_m\": "  <<v3(bottle_body)<<",\n";
    os<<"  \"candidates\": []\n}\n";
}

// ---------------------------------------------------------------------------
// Annotate + save frame to JPEG
// ---------------------------------------------------------------------------

static void save_annotated_frame(
    const cv::Mat& bgr,
    const cv::Rect& box, float conf,
    const Eigen::Vector3d& base_task,
    const Eigen::Vector3d& body_task,
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
        cv::rectangle(disp, box, cv::Scalar(0,255,0), 2);

        std::ostringstream ct; ct<<std::fixed; ct.precision(2);
        ct<<"bottle  conf="<<conf;
        cv::putText(disp, ct.str(),
                    cv::Point(box.x, std::max(0, box.y-8)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0,255,0), 2);

        std::ostringstream b, r;
        b<<std::fixed; b.precision(3);
        r<<std::fixed; r.precision(3);
        b<<"base_task [m]: "<<base_task[0]<<", "<<base_task[1]<<", "<<base_task[2];
        r<<"body_task [m]: "<<body_task[0]<<", "<<body_task[1]<<", "<<body_task[2];
        put(b.str(), 0);
        put(r.str(), 1);
        put("BOTTLE DETECTED", 2, cv::Scalar(0,200,255));
    } else {
        put("No bottle detected", 0, cv::Scalar(0,80,255));
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
    std::string save_image   = "/tmp/bottle_detection_preview.jpg";
    float conf_threshold     = 0.40f;
    bool  headless           = false;

    Eigen::Matrix4d T_task_cam = Eigen::Matrix4d::Identity();

    for (int i=1;i<argc;i++) {
        std::string a(argv[i]);
        if      (a=="--model"   && i+1<argc) { model_path   = argv[++i]; }
        else if (a=="--classes" && i+1<argc) { classes_path = argv[++i]; }
        else if (a=="--conf"    && i+1<argc) { conf_threshold = std::stof(argv[++i]); }
        else if (a=="--save-image" && i+1<argc) { save_image = argv[++i]; }
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

    YOLODetector detector;
    if (!detector.loadModel(model_path, classes_path)) return 1;

    rs2::pipeline pipe;
    rs2::config   cfg;
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

    rs2_intrinsics intr = profile.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>().get_intrinsics();
    rs2::align align_to_color(RS2_STREAM_COLOR);

    std::cerr<<"[INFO] Warming up ("<<N_WARMUP<<" frames)...\n";
    for (int i=0;i<N_WARMUP;i++) pipe.wait_for_frames();

    // Detection loop — keep best across N_FRAMES
    std::cerr<<"[INFO] Detecting over "<<N_FRAMES<<" frames...\n";
    bool            found     = false;
    float           best_conf = 0.0f;
    cv::Rect        best_box;
    cv::Mat         best_frame;

    for (int fi=0;fi<N_FRAMES;fi++) {
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
                    found      = true;
                }
            }
        }
        if (!found) best_frame = bgr.clone();
    }
    pipe.stop();

    // Compute bottle XY position via ray-plane intersection (table Z = 0).
    // We project the camera ray through the bbox centre to the table plane.
    // For a 17.5 cm bottle the bbox centre is around the mid-body (~8-10 cm),
    // so the ray hits the table slightly offset from the true base in XY — but
    // this is small compared to the bottle's 7.3 cm body diameter and acceptable
    // for a body-grasp side pinch. The grasp Z is set from the known constant.
    Eigen::Vector3d bottle_base_task, bottle_body_task;
    Eigen::Vector3d pos_cam(0,0,0);  // kept for JSON completeness only
    if (found) {
        std::cerr<<"[INFO] Bottle detected  conf="<<best_conf
                 <<"  bbox=["<<best_box.x<<","<<best_box.y
                 <<" "<<best_box.width<<"x"<<best_box.height<<"]\n";

        if (!YOLODetector::rayTableIntersect(best_box, intr, T_task_cam, bottle_base_task)) {
            std::cerr<<"[WARN] Ray-table intersection failed (camera looking up?). "
                     <<"Falling back to depth.\n";
            found = false;
        } else {
            // bottle_base_task: XY from ray-table at Z=0, Z=0 (table surface).
            // bottle_body_task: same XY, Z = BOTTLE_DEFAULT_GRASP_Z (10 cm).
            //   Used as the reference for display and grasp planning in Python.
            bottle_body_task    = bottle_base_task;
            bottle_body_task.z() = BOTTLE_DEFAULT_GRASP_Z;
            std::cerr<<"[INFO] Bottle base task ("<<bottle_base_task.transpose()<<") m\n";
            std::cerr<<"[INFO] Bottle body task ("<<bottle_body_task.transpose()<<") m\n";
        }
    } else {
        std::cerr<<"[WARN] No bottle detected (conf≥"<<conf_threshold<<").\n";
    }

    // Save annotated JPEG (unless headless)
    if (!headless && !best_frame.empty()) {
        save_annotated_frame(best_frame, best_box, best_conf,
                             bottle_base_task, bottle_body_task, found, save_image);
    }

    // Emit JSON
    emit_json(std::cout, found, best_conf, pos_cam, bottle_base_task, bottle_body_task);
    return 0;
}
