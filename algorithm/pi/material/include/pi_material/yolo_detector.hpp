#pragma once

#include "pi_material/detector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <string>
#include <vector>

namespace pi_material {

// Contract: Ultralytics YOLOv8/YOLO11 DETECTION, ONNX, batch=1,
// static square size, six classes [red,yellow,blue,green,black,cyan],
// NMS DISABLED, raw output [1,10,N] or [1,N,10].
// Not compatible with COCO-80, YOLOv5 objectness, segmentation,
// built-in NMS outputs, YOLO26 end-to-end, or arbitrary models.
struct YoloConfig {
    std::string onnx_path;
    int input_size = 640;   // model exported at exactly this size
    float confidence = 0.50f;
    float nms_iou = 0.45f;
    int max_candidates_per_class = 300;
};

struct LetterboxInfo {
    cv::Rect roi;      // within the ORIGINAL BGR frame
    cv::Size resized;  // resized ROI dimensions before padding
    int pad_left = 0;
    int pad_top = 0;
    int input_size = 0;
    double scale_x = 0;
    double scale_y = 0;
};

LetterboxInfo compute_letterbox(cv::Size frame_size, cv::Rect roi, int input_size);

// Decode raw float32 output into image-coordinate detections for one
// selected color. Class selection is based on argmax of ALL 6 class
// scores, so non-target colors cannot be treated as targets.
Detection decode_yolo_raw(const cv::Mat& output, const LetterboxInfo& info,
                          int wanted_color, const YoloConfig& yolo,
                          const DetectorConfig& spatial);

// Unlike the HSV geometry detector, YOLO expects bounding boxes for
// the material TOP-FACE at training time, not the entire gripper or disk.
class YoloDetector {
public:
    YoloDetector(YoloConfig yolo, DetectorConfig spatial);
    Detection detect(const cv::Mat& bgr, int wanted_color);

private:
    YoloConfig cfg_;
    DetectorConfig spatial_;
    cv::dnn::Net net_;
};

}  // namespace pi_material
