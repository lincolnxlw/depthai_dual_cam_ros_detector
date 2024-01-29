#include <cstdio>
#include <iostream>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>

#include "depthai/depthai.hpp"

#include <yaml-cpp/yaml.h>

class ImagePublisher {
 public:
  ImagePublisher();

 private:
  dai::Pipeline createPipeline(int fps,
                               int nn_width,
                               int nn_height,
                               float confidence_threshold,
                               float iou_threshold,
                               int classes,
                               int coordinates,
                               std::string nn_path,
                               std::vector<float> anchors,
                               std::map<std::string, cv::dnn::MatShape> anchor_masks);
  bool checkFileExists(const std::string& file_path);
  void readYoloV6YamlConfig(const std::string& config_path,
                            int& nn_width,
                            int& nn_height,
                            int& classes,
                            int& coordinates,
                            float& iou_threshold,
                            float& confidence_threshold,
                            std::vector<float>& anchors,
                            std::map<std::string, cv::dnn::MatShape>& anchor_masks,
                            std::vector<std::string>& labels);
};

bool ImagePublisher::checkFileExists(const std::string& file_path){
  std::ifstream file(file_path);
  return file.good();
}

void ImagePublisher::readYoloV6YamlConfig(const std::string& config_path,
                                          int& nn_width,
                                          int& nn_height,
                                          int& classes,
                                          int& coordinates,
                                          float& iou_threshold,
                                          float& confidence_threshold,
                                          std::vector<float>& anchors,
                                          std::map<std::string, cv::dnn::MatShape>& anchor_masks,
                                          std::vector<std::string>& labels)
{
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);

    std::string input_size = config["nn_config"]["input_size"].as<std::string>();
    if (sscanf(input_size.c_str(), "%dx%d", &nn_width, &nn_height) != 2) {
      std::cerr << "Invalid input_size format: " << input_size << std::endl;
    }

    classes = config["nn_config"]["NN_specific_metadata"]["classes"].as<int>();
    coordinates = config["nn_config"]["NN_specific_metadata"]["coordinates"].as<int>();
    iou_threshold = config["nn_config"]["NN_specific_metadata"]["iou_threshold"].as<float>();
    confidence_threshold = config["nn_config"]["NN_specific_metadata"]["confidence_threshold"].as<float>();

    labels.clear();
    for (const auto& label : config["mappings"]["labels"]) {
      labels.push_back(label.as<std::string>());
      std::cout << "Found label: " << label.as<std::string>() << std::endl;
    }

  } catch (const YAML::Exception& e) {
    std::cerr << "Error reading YAML file: " << e.what() << std::endl;
  }
}

dai::Pipeline ImagePublisher::createPipeline(int fps,
                                             int nn_width,
                                             int nn_height,
                                             float confidence_threshold,
                                             float iou_threshold,
                                             int classes,
                                             int coordinates,
                                             std::string nn_path,
                                             std::vector<float> anchors,
                                             std::map<std::string, cv::dnn::MatShape> anchor_masks)
{
  dai::Pipeline pipeline;

  auto right_cam = pipeline.create<dai::node::ColorCamera>();
  auto left_cam = pipeline.create<dai::node::ColorCamera>();
  auto right_cam_detection_nn = pipeline.create<dai::node::YoloDetectionNetwork>();
  auto left_cam_detection_nn = pipeline.create<dai::node::YoloDetectionNetwork>();
  auto right_cam_image_xlink_out = pipeline.create<dai::node::XLinkOut>();
  auto left_cam_image_xlink_out = pipeline.create<dai::node::XLinkOut>();
  auto right_cam_det_xlink_out = pipeline.create<dai::node::XLinkOut>();
  auto left_cam_det_xlink_out = pipeline.create<dai::node::XLinkOut>();

  right_cam_image_xlink_out->setStreamName("right_cam_image");
  left_cam_image_xlink_out->setStreamName("left_cam_image");
  right_cam_det_xlink_out->setStreamName("right_cam_detection");
  left_cam_det_xlink_out->setStreamName("left_cam_detection");

  // camera
  right_cam->setPreviewSize(nn_width, nn_height);
  right_cam->setPreviewKeepAspectRatio(false);
  right_cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  right_cam->setBoardSocket(dai::CameraBoardSocket::CAM_B);
  right_cam->setInterleaved(false);
  right_cam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  right_cam->setFps(fps);

  left_cam->setPreviewSize(nn_width, nn_height);
  left_cam->setPreviewKeepAspectRatio(false);
  left_cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  left_cam->setBoardSocket(dai::CameraBoardSocket::CAM_C);
  left_cam->setInterleaved(false);
  left_cam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  left_cam->setFps(fps);

  // detection
  right_cam_detection_nn->setConfidenceThreshold(confidence_threshold);
  right_cam_detection_nn->setIouThreshold(iou_threshold);
  right_cam_detection_nn->setNumClasses(classes);
  right_cam_detection_nn->setCoordinateSize(coordinates);
  right_cam_detection_nn->setAnchors(anchors);
  right_cam_detection_nn->setAnchorMasks(anchor_masks);
  right_cam_detection_nn->setBlobPath(nn_path);
  right_cam_detection_nn->setNumInferenceThreads(1);
  right_cam_detection_nn->setNumNCEPerInferenceThread(1);
  right_cam_detection_nn->input.setBlocking(false);

  left_cam_detection_nn->setConfidenceThreshold(confidence_threshold);
  left_cam_detection_nn->setIouThreshold(iou_threshold);
  left_cam_detection_nn->setNumClasses(classes);
  left_cam_detection_nn->setCoordinateSize(coordinates);
  left_cam_detection_nn->setAnchors(anchors);
  left_cam_detection_nn->setAnchorMasks(anchor_masks);
  left_cam_detection_nn->setBlobPath(nn_path);
  left_cam_detection_nn->setNumInferenceThreads(1);
  left_cam_detection_nn->setNumNCEPerInferenceThread(1);
  left_cam_detection_nn->input.setBlocking(false);

  // rgb linking
  right_cam->preview.link(right_cam_detection_nn->input);
  right_cam->preview.link(right_cam_image_xlink_out->input);
  right_cam_detection_nn->out.link(right_cam_det_xlink_out->input);

  left_cam->preview.link(left_cam_detection_nn->input);
  left_cam->preview.link(left_cam_image_xlink_out->input);
  left_cam_detection_nn->out.link(left_cam_det_xlink_out->input);

  std::cout << "Pipeline created!" << std::endl;

  return pipeline;
}

ImagePublisher::ImagePublisher()
{
  std::cout << "Starting depthai image publisher..." << std::endl;

  // image config
  int fps = 15;
  double focal_scale = 1.0;

  // nn config
  std::string nn_model_path, nn_config_path;
  nn_model_path = std::string("/catkin_ws/src/minimum_image_publisher/models/yolov6n_openvino_2022.1_6shave.blob");
  nn_config_path = std::string("/catkin_ws/src/minimum_image_publisher/config/yolov6n.yaml");

  if (!checkFileExists(nn_model_path)) {
    std::cerr << "NN model not found!" << std::endl;
    exit(1);
  }
  if (!checkFileExists(nn_config_path)) {
    std::cerr << "NN config not found!" << std::endl;
    exit(1);
  }

  int nn_width;
  int nn_height;    
  std::vector<std::string> labels = {};
  int classes;
  int coordinates;
  float iou_threshold;
  float confidence_threshold;
  std::vector<float> anchors = {};
  std::map<std::string, cv::dnn::MatShape> anchor_masks = std::map<std::string, cv::dnn::MatShape>();

  readYoloV6YamlConfig(nn_config_path,
                       nn_width,
                       nn_height,
                       classes,
                       coordinates,
                       iou_threshold,
                       confidence_threshold,
                       anchors,
                       anchor_masks,
                       labels);

  // depthai
  dai::Device device;
  dai::CalibrationHandler calib_data = device.readCalibration();

  dai::Pipeline pipeline = createPipeline (
                              fps, nn_width, nn_height,
                              confidence_threshold, iou_threshold,
                              classes, coordinates,
                              nn_model_path, anchors, anchor_masks);

  device.startPipeline(pipeline);

  std::shared_ptr<dai::DataOutputQueue> right_cam_image_q = device.getOutputQueue("right_cam_image", 10, false);
  std::shared_ptr<dai::DataOutputQueue> left_cam_image_q = device.getOutputQueue("left_cam_image", 10, false);
  std::shared_ptr<dai::DataOutputQueue> right_cam_detection_q = device.getOutputQueue("right_cam_detection", 10, false); 
  std::shared_ptr<dai::DataOutputQueue> left_cam_detection_q = device.getOutputQueue("left_cam_detection", 10, false); 

  int right_fps = 0;
  int left_fps = 0;
  int right_frame_count = 0;
  int left_frame_count = 0;
  auto start_time = std::chrono::steady_clock::now();

  while (true) {
    std::shared_ptr<dai::ImgFrame> right_cam_image_frame;
    std::shared_ptr<dai::ImgFrame> left_cam_image_frame;
    right_cam_image_frame = right_cam_image_q->tryGet<dai::ImgFrame>();
    if (right_cam_image_frame) {
      right_frame_count++;
    }
    left_cam_image_frame = left_cam_image_q->tryGet<dai::ImgFrame>();
    if (left_cam_image_frame) {
      left_frame_count++;
    }

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);

    if (elapsed_time.count() >= 1) {
      left_fps = left_frame_count / elapsed_time.count();
      right_fps = right_frame_count / elapsed_time.count();

      start_time = current_time;
      left_frame_count = 0;
      right_frame_count = 0;
      std::cout << "Left FPS: " << left_fps << ", Right FPS: " << right_fps << std::endl;
    }
  }
}

int main(int argc, char** argv) {
  ImagePublisher image_publisher;
  return 0;
}
