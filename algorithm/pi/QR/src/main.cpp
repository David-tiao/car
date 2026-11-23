// Raspberry Pi 5 stand-alone competition QR reader and large-screen display.
// Capture: rpicam-vid raw YUV420 (CSI) or OpenCV V4L2 (USB), plus image mode.
// No ROS 2 publishing, serial writes, GPIO, network, or motor commands.
#include "pi_qr/task_code.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <poll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void on_signal(int) { stop_requested = 1; }

struct Options {
    std::string source = "pi";  // pi | usb | image
    std::string image_file;
    std::string device = "/dev/video0";
    std::string output_file;
    int camera_id = 0;
    int width = 640, height = 480, fps = 15;
    int screen_width = 960, screen_height = 540;
    double screen_height_mm = 0.0;
    unsigned confirm_frames = 3;
    bool headless = false;
    bool fullscreen = true;
};

void usage(const char* argv0) {
    std::cout
        << "Pi5 C++ QR reader (no actuation)\n"
        << "Usage: " << argv0 << " [options]\n"
        << "  --source pi|usb|image   default: pi (CSI with rpicam-vid)\n"
        << "  --device /dev/video0    USB/V4L2 camera only\n"
        << "  --image file.png        test a static image; implies --source image\n"
        << "  --camera 0              rpicam-vid camera index\n"
        << "  --width 640 --height 480 --fps 15\n"
        << "  --screen-width 960 --screen-height 540 (display canvas size)\n"
        << "  --screen-height-mm 160  physical screen height; check 12 mm text\n"
        << "  --confirm-frames 3      consecutive identical, valid frames\n"
        << "  --output /tmp/pi_qr.txt write only newly confirmed task codes\n"
        << "  --window                windowed development mode\n"
        << "  --headless              no GUI (SSH/test); prints accepted code\n"
        << "  -h, --help              show help\n"
        << "GUI: Q/Esc exit, R clear confirmed code.\n";
}

int value_int(int& i, int argc, char** argv, const std::string& option) {
    if (++i >= argc) throw std::invalid_argument("missing value: " + option);
    std::size_t pos = 0;
    const auto v = std::stoi(argv[i], &pos);
    if (pos != std::strlen(argv[i]))
        throw std::invalid_argument("non-integer value for " + option);
    return v;
}

std::string value_string(int& i, int argc, char** argv, const std::string& option) {
    if (++i >= argc) throw std::invalid_argument("missing value: " + option);
    return argv[i];
}

Options parse_args(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "-h" || key == "--help") {
            usage(argv[0]);
            std::exit(0);
        } else if (key == "--source") {
            opt.source = value_string(i, argc, argv, key);
        } else if (key == "--image") {
            opt.image_file = value_string(i, argc, argv, key);
            opt.source = "image";
        } else if (key == "--device") {
            opt.device = value_string(i, argc, argv, key);
        } else if (key == "--output") {
            opt.output_file = value_string(i, argc, argv, key);
        } else if (key == "--camera") {
            opt.camera_id = value_int(i, argc, argv, key);
        } else if (key == "--width") {
            opt.width = value_int(i, argc, argv, key);
        } else if (key == "--height") {
            opt.height = value_int(i, argc, argv, key);
        } else if (key == "--fps") {
            opt.fps = value_int(i, argc, argv, key);
        } else if (key == "--screen-width") {
            opt.screen_width = value_int(i, argc, argv, key);
        } else if (key == "--screen-height") {
            opt.screen_height = value_int(i, argc, argv, key);
        } else if (key == "--screen-height-mm") {
            opt.screen_height_mm = std::stod(value_string(i, argc, argv, key));
        } else if (key == "--confirm-frames") {
            const int n = value_int(i, argc, argv, key);
            if (n < 1 || n > 100)
                throw std::invalid_argument("confirm-frames must be 1..100");
            opt.confirm_frames = static_cast<unsigned>(n);
        } else if (key == "--headless") {
            opt.headless = true;
        } else if (key == "--window") {
            opt.fullscreen = false;
        } else {
            throw std::invalid_argument("unrecognized option: " + key);
        }
    }
    if (opt.source != "pi" && opt.source != "usb" && opt.source != "image")
        throw std::invalid_argument("--source must be pi, usb, or image");
    if (opt.source == "image" && opt.image_file.empty())
        throw std::invalid_argument("--image <path> is required for image source");
    if (opt.width < 320 || opt.width > 1920 || opt.width % 2 ||
        opt.height < 240 || opt.height > 1080 || opt.height % 2)
        throw std::invalid_argument("capture dimensions must be even; width=320..1920 height=240..1080");
    if (opt.fps < 1 || opt.fps > 60 || opt.camera_id < 0 ||
        opt.screen_width < 320 || opt.screen_height < 240 ||
        opt.screen_height_mm < 0)
        throw std::invalid_argument("invalid camera or display setting");
    return opt;
}

// Direct argv exec: no shell interpolation and no file/URL contents executed.
class PiYuvCamera {
public:
    explicit PiYuvCamera(const Options& opt)
        : width_(opt.width), height_(opt.height),
          raw_(static_cast<std::size_t>(opt.width) * opt.height * 3 / 2) {
        int fds[2];
        if (::pipe(fds) != 0) throw std::runtime_error("pipe failed");
        pid_ = ::fork();
        if (pid_ < 0) {
            ::close(fds[0]);
            ::close(fds[1]);
            throw std::runtime_error("fork failed");
        }
        if (pid_ == 0) {
            ::close(fds[0]);
            if (::dup2(fds[1], STDOUT_FILENO) < 0) _exit(126);
            ::close(fds[1]);
            const std::string width = std::to_string(opt.width);
            const std::string height = std::to_string(opt.height);
            const std::string fps = std::to_string(opt.fps);
            const std::string camera = std::to_string(opt.camera_id);
            ::execlp("rpicam-vid", "rpicam-vid",
                     "--timeout", "0", "--nopreview",
                     "--codec", "yuv420", "--width", width.c_str(),
                     "--height", height.c_str(), "--framerate", fps.c_str(),
                     "--camera", camera.c_str(), "--output", "-",
                     static_cast<char*>(nullptr));
            std::fprintf(stderr, "cannot exec rpicam-vid: %s\n", std::strerror(errno));
            _exit(127);
        }
        ::close(fds[1]);
        fd_ = fds[0];
    }

    PiYuvCamera(const PiYuvCamera&) = delete;
    PiYuvCamera& operator=(const PiYuvCamera&) = delete;

    ~PiYuvCamera() {
        if (fd_ >= 0) ::close(fd_);
        if (pid_ > 0) {
            ::kill(pid_, SIGTERM);
            int status = 0;
            for (int i = 0; i < 20; ++i) {
                const pid_t result = ::waitpid(pid_, &status, WNOHANG);
                if (result == pid_ || (result == -1 && errno == ECHILD)) return;
                ::usleep(50000);
            }
            ::kill(pid_, SIGKILL);
            while (::waitpid(pid_, &status, 0) < 0 && errno == EINTR) {}
        }
    }

    bool read_frame(cv::Mat& bgr) {
        std::size_t completed = 0;
        while (completed < raw_.size() && !stop_requested) {
            pollfd watch{fd_, POLLIN, 0};
            const int ready = ::poll(&watch, 1, 8000);
            if (ready < 0 && errno == EINTR) continue;
            if (ready == 0) throw std::runtime_error(
                "camera timeout: check rpicam-hello --list-cameras / CSI cable");
            if (ready < 0) throw std::runtime_error("poll failed on rpicam-vid");
            const ssize_t n = ::read(fd_, raw_.data() + completed,
                                     raw_.size() - completed);
            if (n == 0) throw std::runtime_error(
                "rpicam-vid closed stdout; check camera and permissions");
            if (n < 0 && errno == EINTR) continue;
            if (n < 0) throw std::runtime_error("rpicam-vid raw read failed");
            completed += static_cast<std::size_t>(n);
        }
        if (stop_requested) return false;
        cv::Mat yuv(height_ * 3 / 2, width_, CV_8UC1, raw_.data());
        cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_I420);
        return true;
    }

private:
    int width_, height_, fd_ = -1;
    pid_t pid_ = -1;
    std::vector<unsigned char> raw_;
};

std::vector<std::string> decode_qrs(cv::QRCodeDetector& detector,
                                     const cv::Mat& bgr) {
    std::vector<cv::String> multiple;
    std::vector<std::string> values;
    try {
        if (detector.detectAndDecodeMulti(bgr, multiple)) {
            for (const auto& text : multiple) {
                if (!text.empty()) values.emplace_back(text);
            }
            if (!values.empty()) return values;
        }
        const std::string single = detector.detectAndDecode(bgr);
        if (!single.empty()) values.push_back(single);
    } catch (const cv::Exception& exc) {
        std::cerr << "QR decode warning: " << exc.what() << "\n";
    }
    return values;
}

void publish_confirmed(const std::string& code, const Options& opt) {
    std::cout << "QR_CONFIRMED " << code << std::endl;
    if (opt.output_file.empty()) return;
    const std::string temporary = opt.output_file + ".tmp";
    {
        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
        if (!stream) throw std::runtime_error("cannot write " + temporary);
        stream << code << "\n";
        stream.close();
        if (!stream) throw std::runtime_error("writing output code failed");
    }
    if (std::rename(temporary.c_str(), opt.output_file.c_str()) != 0)
        throw std::runtime_error("cannot replace " + opt.output_file);
}

int centered_text(cv::Mat& canvas, const std::string& text, int baseline_y,
                  double wanted_height, const cv::Scalar& color,
                  int thickness = 3) {
    const int font = cv::FONT_HERSHEY_DUPLEX;
    int baseline = 0;
    const cv::Size unit = cv::getTextSize(text, font, 1.0, thickness, &baseline);
    if (unit.width == 0 || unit.height == 0) return 0;
    const double fit = static_cast<double>(canvas.cols - 40) / unit.width;
    const double scale = std::min(wanted_height / unit.height, fit);
    const cv::Size actual = cv::getTextSize(text, font, scale, thickness, &baseline);
    cv::putText(canvas, text,
                cv::Point((canvas.cols - actual.width) / 2, baseline_y),
                font, scale, color, thickness, cv::LINE_AA);
    return actual.height;
}

void draw_display(const Options& opt, const cv::Mat& bgr,
                  const pi_qr::ThreeFrameFilter& filter,
                  const std::vector<std::string>& payloads,
                  cv::Mat& screen) {
    screen = cv::Mat(opt.screen_height, opt.screen_width, CV_8UC3,
                     cv::Scalar(25, 22, 18));
    cv::putText(screen, "PI5 / QR TASK CODE", cv::Point(24, 45),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 240, 210),
                2, cv::LINE_AA);

    if (!bgr.empty()) {
        const int thumb_w = std::max(100, opt.screen_width / 5);
        const int thumb_h = std::min(opt.screen_height / 4,
                                     thumb_w * bgr.rows / bgr.cols);
        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(thumb_w, thumb_h));
        resized.copyTo(screen(cv::Rect(opt.screen_width - thumb_w - 12,
                                       9, thumb_w, thumb_h)));
    }

    std::string state = "WAITING FOR QR";
    cv::Scalar status_color(150, 195, 255);
    if (payloads.size() > 1) {
        state = "MULTIPLE QR: REJECTED";
    } else if (payloads.size() == 1 &&
               !pi_qr::parse_task_code(payloads.front())) {
        state = "INVALID TASK CODE";
    } else if (filter.streak() > 0 &&
               filter.streak() < filter.required_frames()) {
        state = "VERIFYING " + std::to_string(filter.streak()) + "/" +
                std::to_string(filter.required_frames());
    } else if (filter.confirmed() &&
               filter.streak() >= filter.required_frames()) {
        state = "CONFIRMED / LIVE";
        status_color = cv::Scalar(115, 240, 150);
    } else if (filter.confirmed()) {
        state = "LAST CONFIRMED / NOT LIVE";
    }
    cv::putText(screen, state, cv::Point(24, 84),
                cv::FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2, cv::LINE_AA);

    if (filter.confirmed()) {
        const auto& groups = filter.confirmed()->groups;
        const std::string first = groups[0] + "+" + groups[1] + "+";
        const std::string second = groups[2] + "+" + groups[3];
        const double target_pixels = opt.screen_height_mm > 0
            ? 15.0 * opt.screen_height / opt.screen_height_mm
            : std::max(60.0, opt.screen_height * 0.145);
        const int h1 = centered_text(screen, first,
                                    opt.screen_height * 55 / 100,
                                    target_pixels, cv::Scalar(255, 255, 255));
        const int h2 = centered_text(screen, second,
                                    opt.screen_height * 78 / 100,
                                    target_pixels, cv::Scalar(255, 255, 255));
        if (opt.screen_height_mm > 0) {
            const double min_height_px =
                12.0 * opt.screen_height / opt.screen_height_mm;
            if (h1 < min_height_px || h2 < min_height_px) {
                cv::putText(screen, "WARNING: TEXT UNDER 12mm",
                    cv::Point(24, opt.screen_height - 42),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(60, 120, 255), 2, cv::LINE_AA);
            }
        }
    } else {
        centered_text(screen, "NO VALID TASK CODE",
                      opt.screen_height * 65 / 100,
                      opt.screen_height * 0.08, cv::Scalar(130, 160, 180), 2);
    }
    cv::putText(screen, "Q/ESC EXIT   R CLEAR   (NO MOTOR CONTROL)",
                cv::Point(20, opt.screen_height - 14),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(140, 140, 145),
                1, cv::LINE_AA);
}

void show_screen(const cv::Mat& screen) {
    cv::imshow("Pi QR Display", screen);
}

int run(const Options& opt) {
    cv::QRCodeDetector detector;
    pi_qr::ThreeFrameFilter filter(opt.confirm_frames);
    const bool static_image = opt.source == "image";
    cv::Mat saved;
    cv::VideoCapture usb;
    std::optional<PiYuvCamera> pi;

    if (static_image) {
        saved = cv::imread(opt.image_file);
        if (saved.empty()) throw std::runtime_error("cannot open image: " + opt.image_file);
        std::cerr << "STATIC IMAGE TEST: repeated frames are simulated, not live evidence\n";
    } else if (opt.source == "pi") {
        pi.emplace(opt);
    } else {
        usb.open(opt.device, cv::CAP_V4L2);
        if (!usb.isOpened()) throw std::runtime_error(
            "cannot open USB/V4L2 camera " + opt.device);
        usb.set(cv::CAP_PROP_FRAME_WIDTH, opt.width);
        usb.set(cv::CAP_PROP_FRAME_HEIGHT, opt.height);
        usb.set(cv::CAP_PROP_FPS, opt.fps);
    }

    if (!opt.headless) {
        cv::namedWindow("Pi QR Display", cv::WINDOW_NORMAL);
        if (opt.fullscreen) {
            try {
                cv::setWindowProperty("Pi QR Display",
                    cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            } catch (const cv::Exception&) {
                std::cerr << "Fullscreen unsupported by current OpenCV GUI; windowed fallback\n";
                cv::resizeWindow("Pi QR Display", opt.screen_width, opt.screen_height);
            }
        } else {
            cv::resizeWindow("Pi QR Display", opt.screen_width, opt.screen_height);
        }
    }

    unsigned frame_count = 0, failed_usb = 0;
    while (!stop_requested) {
        cv::Mat frame;
        if (static_image) {
            frame = saved;
        } else if (opt.source == "pi") {
            if (!pi->read_frame(frame)) break;
        } else {
            if (!usb.read(frame) || frame.empty()) {
                if (++failed_usb > 30)
                    throw std::runtime_error("USB camera stopped delivering frames");
                filter.observe({});
                ::usleep(100000);
                continue;
            }
            failed_usb = 0;
        }
        const auto payloads = decode_qrs(detector, frame);
        const auto new_code = filter.observe(payloads);
        if (new_code) publish_confirmed(new_code->text, opt);
        ++frame_count;

        if (!opt.headless) {
            cv::Mat screen;
            draw_display(opt, frame, filter, payloads, screen);
            show_screen(screen);
            const int key = cv::waitKey(static_image ? 40 : 1) & 0xff;
            if (key == 27 || key == 'q' || key == 'Q') break;
            if (key == 'r' || key == 'R') {
                filter.clear();
                std::cout << "QR_CLEARED\n";
            }
        }
        // On a still image, repeats exercise the filter logic only; not true temporal stability.
        if (static_image && (opt.headless || frame_count >= opt.confirm_frames) &&
            opt.headless)
            return filter.confirmed().has_value() ? 0 : 2;
    }
    if (!opt.headless) cv::destroyAllWindows();
    return 0;
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    try {
        return run(parse_args(argc, argv));
    } catch (const std::exception& e) {
        std::cerr << "Pi QR error: " << e.what() << "\n";
        return 1;
    }
}
