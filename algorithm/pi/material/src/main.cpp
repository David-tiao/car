// C++17 Raspberry Pi 5 material recognition & advisory visual-servo demo.
// QR authority: previously confirmed algorithm/pi/QR local text output.
// No serial TX, GPIO, ROS2 velocity publication, gripper or wheel commands.
#include "pi_material/camera.hpp"
#include "pi_material/core.hpp"
#include "pi_material/detector.hpp"
#include "pi_material/yolo_detector.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>

namespace {
using pi_material::Spot;
using pi_material::Target;
using pi_material::ServoEstimate;

struct Options {
    pi_material::CameraOptions camera;
    pi_material::DetectorConfig detector;
    pi_material::YoloConfig yolo;
    std::string backend = "hsv"; // hsv | yolo
    bool hsv_file_given = false;
    pi_material::ServoSettings servo;
    std::string qr_file = "/tmp/pi_qr_verified.txt";
    std::string task_code;
    std::string result_file = "/tmp/pi_material_result.json";
    int max_task_age_sec = 600;
    int confirm_frames = 3;
    double jitter_px = 10;
    double known_diameter_mm = 0; // optional monocular rough depth model
    double focal_px = 0;
    bool headless = false;
    bool auto_next = false;   // permitted only for offline image list
};

double number(const char* text, const std::string& key) {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != std::strlen(text) || !std::isfinite(value))
        throw std::invalid_argument("invalid number: " + key);
    return value;
}
int integer(const char* text, const std::string& key) {
    const double d = number(text, key);
    if (std::floor(d) != d || d < -1000000 || d > 1000000)
        throw std::invalid_argument("invalid integer: " + key);
    return static_cast<int>(d);
}
std::string next_arg(int& i, int argc, char** argv, const std::string& key) {
    if (++i >= argc) throw std::invalid_argument("missing argument to " + key);
    return argv[i];
}
std::vector<double> csv_numbers(const std::string& line, int count,
                                const std::string& key) {
    std::istringstream stream(line);
    std::string token;
    std::vector<double> vals;
    while (std::getline(stream, token, ',')) {
        vals.push_back(number(token.c_str(), key));
    }
    if (static_cast<int>(vals.size()) != count)
        throw std::invalid_argument(key + " needs " + std::to_string(count) +
                                    " comma-separated values");
    return vals;
}
void usage(const char* program) {
    std::cout
        << "Pi5 material recognizer + calibrated visual-servo ADVISORY\n"
        << "Usage: " << program << " [options]\n"
        << "  --qr-file /tmp/pi_qr_verified.txt  input from pi/QR --output\n"
        << "  --task-code 156+123+516+231      manual/offline task instead of file\n"
        << "  --max-task-age-sec 600            reject stale QR file at startup\n"
        << "  --source pi|usb|images            default: pi (CSI rpicam-vid)\n"
        << "  --device /dev/video0 --camera 0   USB device or CSI index\n"
        << "  --width 640 --height 480 --fps 15\n"
        << "  --image-list /tmp/frames.txt      offline paths, one per line\n"
        << "  --repeat-each 3                   repeat each offline still image\n"
        << "  --backend hsv|yolo                hsv by default; YOLO uses ONNX\n"
        << "  --model /path/weights.onnx        REQUIRED for --backend yolo\n"
        << "  --yolo-imgsz 640                  square ONNX input dimension\n"
        << "  --confidence 0.50 --nms-iou 0.45  YOLO thresholds\n"
        << "  --roi x,y,w,h                     crop to material viewing station\n"
        << "  --disk cx,cy,r                    optional observed turntable orbit gate\n"
        << "  --hsv-file thresholds.csv         per-camera color thresholds 1..6\n"
        << "  --min-area 160                    reject small blobs (pixel area)\n"
        << "  --confirm-frames 3 --jitter-px 10\n"
        << "  --target-u 320 --target-v 380     measured gripper image reference\n"
        << "  --jacobian j00,j01,j10,j11        measured pixel/mm, REQUIRED for mm output\n"
        << "  --gain 0.7 --max-step-mm 5 --max-speed-mm-s 25\n"
        << "  --pixel-tolerance 8               deadband on image-plane error\n"
        << "  --focal-px 650 --known-diameter-mm 30   optional approximate range\n"
        << "  --result-file /tmp/pi_material_result.json\n"
        << "  --headless                        console only\n"
        << "  --offline-auto-next               offline images only, NO gripper event\n"
        << "Keys: N = accept stable VISUAL OBSERVATION / next material,\n"
        << "      S = skip target, R = clear current tracking, Q/Esc = exit.\n";
}

Options parse_args(int argc, char** argv) {
    Options opt;
    bool target_u = false, target_v = false;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "-h" || key == "--help") {
            usage(argv[0]);
            std::exit(0);
        }
        if (key == "--headless") { opt.headless = true; continue; }
        if (key == "--offline-auto-next") { opt.auto_next = true; continue; }
        const std::string val = next_arg(i, argc, argv, key);
        if (key == "--qr-file") opt.qr_file = val;
        else if (key == "--task-code") opt.task_code = val;
        else if (key == "--max-task-age-sec") opt.max_task_age_sec = integer(val.c_str(), key);
        else if (key == "--backend") opt.backend = val;
        else if (key == "--model") opt.yolo.onnx_path = val;
        else if (key == "--yolo-imgsz") opt.yolo.input_size = integer(val.c_str(), key);
        else if (key == "--confidence") opt.yolo.confidence =
            static_cast<float>(number(val.c_str(), key));
        else if (key == "--nms-iou") opt.yolo.nms_iou =
            static_cast<float>(number(val.c_str(), key));
        else if (key == "--source") opt.camera.source = val;
        else if (key == "--device") opt.camera.device = val;
        else if (key == "--camera") opt.camera.camera_index = integer(val.c_str(), key);
        else if (key == "--width") opt.camera.width = integer(val.c_str(), key);
        else if (key == "--height") opt.camera.height = integer(val.c_str(), key);
        else if (key == "--fps") opt.camera.fps = integer(val.c_str(), key);
        else if (key == "--image-list") {
            opt.camera.images_file = val;
            opt.camera.source = "images";
        } else if (key == "--repeat-each") opt.camera.repeat_each_image = integer(val.c_str(), key);
        else if (key == "--min-area") opt.detector.min_area_px = number(val.c_str(), key);
        else if (key == "--pixel-tolerance") opt.servo.tolerance_px = number(val.c_str(), key);
        else if (key == "--confirm-frames") opt.confirm_frames = integer(val.c_str(), key);
        else if (key == "--jitter-px") opt.jitter_px = number(val.c_str(), key);
        else if (key == "--target-u") {
            opt.servo.ref_u_px = number(val.c_str(), key); target_u = true;
        } else if (key == "--target-v") {
            opt.servo.ref_v_px = number(val.c_str(), key); target_v = true;
        } else if (key == "--jacobian") {
            const auto a = csv_numbers(val, 4, key);
            opt.servo.jacobian = pi_material::Jacobian{a[0], a[1], a[2], a[3]};
        } else if (key == "--roi") {
            const auto a = csv_numbers(val, 4, key);
            for (const auto x : a) if (std::floor(x) != x)
                throw std::invalid_argument("--roi requires integers");
            opt.detector.roi = cv::Rect(static_cast<int>(a[0]), static_cast<int>(a[1]),
                                       static_cast<int>(a[2]), static_cast<int>(a[3]));
            if (a[2] < 20 || a[3] < 20 || a[0] < 0 || a[1] < 0)
                throw std::invalid_argument("invalid ROI");
        } else if (key == "--disk") {
            const auto a = csv_numbers(val, 3, key);
            if (a[2] <= 0) throw std::invalid_argument("disk radius must be positive");
            opt.detector.disk = cv::Vec3d(a[0], a[1], a[2]);
        } else if (key == "--hsv-file") {
            opt.hsv_file_given = true;
            opt.detector.ranges = pi_material::load_hsv_ranges(val);
        } else if (key == "--gain") opt.servo.gain_per_s = number(val.c_str(), key);
        else if (key == "--max-step-mm") opt.servo.max_step_mm = number(val.c_str(), key);
        else if (key == "--max-speed-mm-s") opt.servo.max_speed_mm_s = number(val.c_str(), key);
        else if (key == "--focal-px") opt.focal_px = number(val.c_str(), key);
        else if (key == "--known-diameter-mm") opt.known_diameter_mm = number(val.c_str(), key);
        else if (key == "--result-file") opt.result_file = val;
        else throw std::invalid_argument("unrecognized option: " + key);
    }
    if (target_u != target_v)
        throw std::invalid_argument("--target-u and --target-v must be set together");
    opt.servo.reference_taught = target_u && target_v;
    if (opt.servo.jacobian &&
        (!opt.servo.reference_taught ||
         !pi_material::valid_jacobian(*opt.servo.jacobian)))
        throw std::invalid_argument("Jacobian requires taught target and nonsingular matrix");
    if (opt.camera.width < 320 || opt.camera.height < 240 ||
        opt.camera.width > 1920 || opt.camera.height > 1080 ||
        opt.camera.width % 2 || opt.camera.height % 2 ||
        opt.camera.fps < 1 || opt.camera.fps > 60 || opt.camera.camera_index < 0 ||
        opt.confirm_frames < 1 || opt.confirm_frames > 100 ||
        opt.max_task_age_sec < 0 || opt.max_task_age_sec > 86400 ||
        opt.focal_px < 0 || opt.known_diameter_mm < 0 ||
        ((opt.focal_px > 0) != (opt.known_diameter_mm > 0)))
        throw std::invalid_argument("invalid camera, confirm or range calibration settings");
    if (opt.backend != "hsv" && opt.backend != "yolo")
        throw std::invalid_argument("--backend must be hsv or yolo");
    if (opt.backend == "yolo" && opt.yolo.onnx_path.empty())
        throw std::invalid_argument("--backend yolo REQUIRES --model six-class ONNX");
    if (opt.backend == "yolo" && opt.hsv_file_given)
        throw std::invalid_argument("--hsv-file applies to HSV backend only");
    if (opt.backend == "hsv" && !opt.yolo.onnx_path.empty())
        throw std::invalid_argument("--model requires --backend yolo");
    if (opt.camera.source != "pi" && opt.camera.source != "usb" &&
        opt.camera.source != "images")
        throw std::invalid_argument("source must be pi, usb or images");
    if (opt.camera.source == "images" && opt.camera.images_file.empty())
        throw std::invalid_argument("--image-list is required for images source");
    if (opt.auto_next && opt.camera.source != "images")
        throw std::invalid_argument("--offline-auto-next cannot run with a real camera");
    if (opt.auto_next && !opt.headless)
        throw std::invalid_argument("--offline-auto-next needs --headless");
    if (opt.result_file.empty()) throw std::invalid_argument("empty --result-file");
    return opt;
}

pi_qr::TaskCode read_task(const Options& opt) {
    std::string text = opt.task_code;
    if (text.empty()) {
        struct stat info{};
        if (::stat(opt.qr_file.c_str(), &info) != 0 || !S_ISREG(info.st_mode))
            throw std::runtime_error(
                "no QR task file: run algorithm/pi/QR/pi_qr --output " + opt.qr_file);
        const double age = std::difftime(std::time(nullptr), info.st_mtime);
        if (age > opt.max_task_age_sec || age < -5)
            throw std::runtime_error(
                "QR task file stale/future-dated; scan a fresh QR or --task-code for offline test");
        std::ifstream in(opt.qr_file, std::ios::binary);
        if (!in) throw std::runtime_error("cannot read QR file");
        std::getline(in, text);
        std::string excess;
        if (std::getline(in, excess) && !excess.empty())
            throw std::runtime_error("QR task file contains multiple lines");
        if (text.size() > 64) throw std::runtime_error("QR text file too long");
    }
    auto code = pi_qr::parse_task_code(text);
    if (!code) throw std::invalid_argument("QR task format invalid");
    return *code;
}

struct Record {
    bool observed = false;
    bool skipped = false;
    Spot spot;
    ServoEstimate servo;
    std::optional<double> range_mm;
    std::optional<double> confidence; // YOLO confidence only, not calibrated probability
    double processing_ms = 0;  // detector wall-time, NOT capture-to-action latency
};

std::string status_of(const Record& r) {
    if (r.skipped) return "SKIPPED";
    if (!r.observed) return "PENDING";
    if (!r.servo.calibrated)
        return r.servo.pixel_centered ? "PIXEL_CENTERED_UNCALIBRATED"
                                      : "PIXEL_OBSERVED_UNCALIBRATED";
    return r.servo.pixel_centered ? "CENTERED_CALIBRATED_ESTIMATE"
                                  : "VISUAL_CORRECTION_REQUIRED";
}

std::optional<double> estimate_range(const Spot& spot, const Options& opt) {
    if (opt.focal_px <= 0 || opt.known_diameter_mm <= 0 || spot.diameter_px <= 0)
        return std::nullopt;
    const double z = opt.focal_px * opt.known_diameter_mm / spot.diameter_px;
    if (!std::isfinite(z) || z <= 0) return std::nullopt;
    return z;
}

void write_result(const Options& opt, const pi_qr::TaskCode& task,
                  const std::array<Target, 6>& plan,
                  const std::array<Record, 6>& results) {
    const auto tmp = opt.result_file + ".tmp";
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("cannot create result temp file");
    out << std::fixed << std::setprecision(3);
    int observed = 0, skipped = 0;
    for (const auto& record : results) {
        if (record.observed) ++observed;
        if (record.skipped) ++skipped;
    }
    out << "{\n  \"schema\": \"pi_material_v1\",\n"
        << "  \"task_code\": \"" << task.text << "\",\n"
        << "  \"camera_source\": \"" << opt.camera.source << "\",\n"
        << "  \"detector_backend\": \"" << opt.backend << "\",\n"
        << "  \"physical_actions_executed\": false,\n"
        << "  \"all_six_observed\": " << (observed == 6 ? "true" : "false") << ",\n"
        << "  \"observed_count\": " << observed << ",\n"
        << "  \"skipped_count\": " << skipped << ",\n"
        << "  \"steps\": [\n";
    for (int i = 0; i < 6; ++i) {
        const auto& step = plan[i];
        const auto& result = results[i];
        out << "    {\"index\": " << (i+1)
            << ", \"batch\": " << step.batch
            << ", \"color_id\": " << step.color
            << ", \"color_name\": \"" << pi_material::color_name(step.color) << "\""
            << ", \"process_ring\": " << step.process_ring
            << ", \"storage_ring\": " << step.storage_ring
            << ", \"status\": \"" << status_of(result) << "\"";
        if (result.observed) {
            out << ", \"center_px\": [" << result.spot.u << ", "
                << result.spot.v << "]"
                << ", \"diameter_px\": " << result.spot.diameter_px
                << ", \"error_px\": [" << result.servo.error_u_px << ", "
                << result.servo.error_v_px << "]"
                << ", \"pixel_centered\": "
                << (result.servo.pixel_centered ? "true" : "false")
                << ", \"servo_calibrated\": "
                << (result.servo.calibrated ? "true" : "false");
            if (result.servo.calibrated) {
                out << ", \"full_correction_mm\": ["
                    << result.servo.full_x_mm << ", "
                    << result.servo.full_y_mm << "]"
                    << ", \"bounded_step_mm\": ["
                    << result.servo.step_x_mm << ", "
                    << result.servo.step_y_mm << "]"
                    << ", \"suggested_velocity_mm_s\": ["
                    << result.servo.vx_mm_s << ", "
                    << result.servo.vy_mm_s << "]";
            } else {
                out << ", \"full_correction_mm\": null"
                    << ", \"bounded_step_mm\": null"
                    << ", \"suggested_velocity_mm_s\": null";
            }
            out << ", \"detector_processing_ms\": " << result.processing_ms;
            out << ", \"detector_confidence\": ";
            if (result.confidence) out << *result.confidence;
            else out << "null";
            out << ", \"approx_monocular_range_mm\": ";
            if (result.range_mm) out << *result.range_mm;
            else out << "null";
        }
        out << "}" << (i == 5 ? "\n" : ",\n");
    }
    out << "  ]\n}\n";
    out.close();
    if (!out) throw std::runtime_error("result JSON write failed");
    if (std::rename(tmp.c_str(), opt.result_file.c_str()) != 0)
        throw std::runtime_error("atomic result replace failed");
}

void draw(const cv::Mat& frame, const Options& opt, const Target& step,
          int step_index, const pi_material::Detection& detection,
          const std::optional<Spot>& stable,
          const std::optional<ServoEstimate>& servo, cv::Mat& shown) {
    frame.copyTo(shown);
    const cv::Point ref(static_cast<int>(std::lround(opt.servo.ref_u_px)),
                        static_cast<int>(std::lround(opt.servo.ref_v_px)));
    cv::drawMarker(shown, ref, cv::Scalar(255, 255, 255),
                   cv::MARKER_CROSS, 32, 2, cv::LINE_AA);
    if (opt.detector.roi)
        cv::rectangle(shown, *opt.detector.roi, cv::Scalar(220, 160, 40), 2);
    if (opt.detector.disk) {
        const auto& d = *opt.detector.disk;
        cv::circle(shown, cv::Point(static_cast<int>(d[0]), static_cast<int>(d[1])),
                   static_cast<int>(d[2]), cv::Scalar(150, 150, 150), 1);
    }
    for (const auto& hit : detection.candidates) {
        cv::rectangle(shown, hit.box, cv::Scalar(0, 230, 255), 2);
        if (hit.confidence > 0) {
            std::ostringstream label;
            label << "YOLO " << std::fixed << std::setprecision(2)
                  << hit.confidence;
            cv::putText(shown, label.str(),
                {hit.box.x, std::max(18, hit.box.y - 5)},
                cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255,255,255),
                1, cv::LINE_AA);
        }
        cv::circle(shown,
                   cv::Point(static_cast<int>(hit.spot.u),
                             static_cast<int>(hit.spot.v)),
                   5, cv::Scalar(0, 230, 255), -1);
    }
    if (stable) {
        const cv::Point p(static_cast<int>(stable->u), static_cast<int>(stable->v));
        cv::arrowedLine(shown, ref, p, cv::Scalar(255, 120, 40),
                        2, cv::LINE_AA);
    }
    const cv::Scalar label(255, 255, 255);
    const auto print_line = [&](int line, const std::string& str,
                                const cv::Scalar& color = cv::Scalar(255,255,255)) {
        cv::putText(shown, str, {14, 26 + line * 29}, cv::FONT_HERSHEY_SIMPLEX,
                    0.63, color, 2, cv::LINE_AA);
    };
    cv::rectangle(shown, cv::Rect(0, 0, shown.cols, std::min(142, shown.rows)), cv::Scalar(14, 15, 17), -1);
    print_line(0, "MATERIAL " + std::to_string(step_index + 1) +
                  "/6  BATCH " + std::to_string(step.batch) + "  " +
                  pi_material::color_name(step.color));
    print_line(1, "ROUGH RING " + std::to_string(step.process_ring) +
                  "  STORAGE RING " + std::to_string(step.storage_ring));
    print_line(2, detection.ambiguous ? "AMBIGUOUS: MULTIPLE SAME-COLOR OBJECTS"
                 : stable ? "STABLE OBSERVATION / PRESS N TO RECORD"
                          : "SEARCHING / VERIFYING", cv::Scalar(80, 230, 255));
    if (servo) {
        std::ostringstream line;
        line << std::fixed << std::setprecision(1) << "ERR(PX) "
             << servo->error_u_px << ", " << servo->error_v_px;
        if (servo->calibrated)
            line << "  STEP(MM) " << servo->step_x_mm
                 << ", " << servo->step_y_mm;
        else line << "  NO MM CALIBRATION";
        print_line(3, line.str());
    } else {
        print_line(3, "NO STABLE IMAGE LOCATION");
    }
    cv::putText(shown, "N OBSERVE/NEXT  S SKIP  R RESET  Q EXIT",
                {14, shown.rows - 14}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                label, 2, cv::LINE_AA);
}

void report_record(int index, const Target& step, const Record& record) {
    std::cout << std::fixed << std::setprecision(2)
              << "STEP_RESULT " << (index+1) << "/6"
              << " batch=" << step.batch
              << " target=" << pi_material::color_name(step.color)
              << " processing_ring=" << step.process_ring
              << " storage_ring=" << step.storage_ring
              << " status=" << status_of(record);
    if (record.observed) {
        if (record.confidence) std::cout << " yolo_conf=" << *record.confidence;
        std::cout << " detector_ms=" << record.processing_ms;
        std::cout << " uv=(" << record.spot.u << "," << record.spot.v << ")"
                  << " error_px=(" << record.servo.error_u_px << ","
                  << record.servo.error_v_px << ")";
        if (record.servo.calibrated)
            std::cout << " correction_mm=(" << record.servo.full_x_mm << ","
                      << record.servo.full_y_mm << ")"
                      << " bounded_step_mm=(" << record.servo.step_x_mm << ","
                      << record.servo.step_y_mm << ")"
                      << " velocity_mm_s=(" << record.servo.vx_mm_s << ","
                      << record.servo.vy_mm_s << ")";
        if (record.range_mm)
            std::cout << " approx_range_mm=" << *record.range_mm;
    }
    std::cout << " ACTIONS_EXECUTED=FALSE\n";
}

int run(const Options& opt) {
    const auto task = read_task(opt);
    const auto plan = pi_material::make_targets(task);
    std::array<Record, 6> records{};
    pi_material::FrameSource frames(opt.camera);
    std::unique_ptr<pi_material::ColorDetector> hsv_detector;
    std::unique_ptr<pi_material::YoloDetector> yolo_detector;
    if (opt.backend == "yolo")
        yolo_detector = std::make_unique<pi_material::YoloDetector>(
            opt.yolo, opt.detector);
    else
        hsv_detector = std::make_unique<pi_material::ColorDetector>(opt.detector);
    pi_material::StableSpot tracking(opt.confirm_frames, opt.jitter_px);

    std::cout << "QR_TASK " << task.text << " SOURCE=" << opt.camera.source
              << " BACKEND=" << opt.backend
              << " PLAN=6 TARGETS, NO HARDWARE ACTUATION\n";
    if (frames.offline())
        std::cout << "OFFLINE_IMAGE_REPEATS ARE NOT INDEPENDENT CAMERA FRAMES\n";
    if (!opt.headless) {
        cv::namedWindow("Pi Material / Visual Servo", cv::WINDOW_NORMAL);
        cv::resizeWindow("Pi Material / Visual Servo", 960, 720);
    }
    std::optional<Spot> current;
    std::optional<ServoEstimate> current_servo;
    std::optional<double> current_range;
    std::optional<double> current_confidence;
    int index = 0;
    std::size_t frame_no = 0;
    while (index < 6 && !pi_material::stop_requested()) {
        cv::Mat frame;
        if (!frames.next(frame)) break;
        if (frame.empty()) {
            tracking.reset();
            current.reset(); current_servo.reset(); current_range.reset();
            current_confidence.reset();
            continue;
        }
        pi_material::ServoSettings servo_cfg = opt.servo;
        if (!servo_cfg.reference_taught) {
            // For pixel diagnostics ONLY; OpenMV pickup point is (160,190)/QVGA.
            // It must NOT be interpreted as calibration for the Pi camera.
            servo_cfg.ref_u_px = frame.cols * 0.5;
            servo_cfg.ref_v_px = frame.rows * (190.0 / 240.0);
        } else if (servo_cfg.ref_u_px >= frame.cols ||
                   servo_cfg.ref_v_px >= frame.rows ||
                   servo_cfg.ref_u_px < 0 || servo_cfg.ref_v_px < 0) {
            throw std::runtime_error("taught target is outside camera frame");
        }
        const auto infer_started = std::chrono::steady_clock::now();
        const auto detection = yolo_detector
            ? yolo_detector->detect(frame, plan[index].color)
            : hsv_detector->detect(frame, plan[index].color);
        const double processing_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - infer_started).count();
        const int previous_count = tracking.count();
        current = tracking.update(detection.unique);
        if (current) {
            current_servo = pi_material::calculate_servo(*current, servo_cfg);
            current_range = estimate_range(*current, opt);
            current_confidence = detection.unique_confidence;
            if (previous_count < opt.confirm_frames) {
                std::cout << "TARGET_STABLE step=" << (index+1)
                          << " color=" << pi_material::color_name(plan[index].color)
                          << " center=(" << current->u << "," << current->v
                          << ") detector_ms=" << processing_ms
                          << " servo_calibrated="
                          << (current_servo->calibrated ? "yes" : "no") << "\n";
            }
        } else {
            current_servo.reset(); current_range.reset();
            current_confidence.reset();
        }
        ++frame_no;
        if (!opt.headless) {
            cv::Mat displayed;
            auto display_opts = opt;
            display_opts.servo = servo_cfg;
            draw(frame, display_opts, plan[index], index, detection,
                 current, current_servo, displayed);
            cv::imshow("Pi Material / Visual Servo", displayed);
        }
        int key = opt.headless ? -1 : (cv::waitKey(1) & 0xff);
        if (key == 'q' || key == 'Q' || key == 27) break;
        if (key == 'r' || key == 'R') {
            tracking.reset(); current.reset(); current_servo.reset();
            current_range.reset(); current_confidence.reset();
            std::cout << "TRACKER_RESET\n";
            continue;
        }

        const bool accept = (key == 'n' || key == 'N' ||
                             (opt.auto_next && current.has_value()));
        const bool skip = (key == 's' || key == 'S');
        if ((accept && current && current_servo) || skip) {
            Record& rec = records[index];
            rec.skipped = skip;
            if (!skip) {
                rec.observed = true;
                rec.spot = *current;
                rec.servo = *current_servo;
                rec.range_mm = current_range;
                rec.confidence = current_confidence;
                rec.processing_ms = processing_ms;
            }
            report_record(index, plan[index], rec);
            write_result(opt, task, plan, records);
            ++index;
            tracking.reset();
            current.reset(); current_servo.reset(); current_range.reset();
            current_confidence.reset();
        } else if (accept) {
            std::cout << "NOT_STABLE: cannot mark target observed\n";
        }
    }
    write_result(opt, task, plan, records);
    if (!opt.headless) cv::destroyAllWindows();
    int observed = 0;
    for (const auto& record : records) if (record.observed) ++observed;
    std::cout << "FINAL_VISUAL_RESULT observed=" << observed
              << "/6 result_file=" << opt.result_file
              << " physical_actions_executed=false\n";
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, pi_material::request_stop);
    std::signal(SIGTERM, pi_material::request_stop);
    try {
        return run(parse_args(argc, argv));
    } catch (const std::exception& e) {
        std::cerr << "Pi material error: " << e.what() << "\n";
        return 1;
    }
}
