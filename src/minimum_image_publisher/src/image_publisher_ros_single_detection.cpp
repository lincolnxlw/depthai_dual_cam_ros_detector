#include <cstdio>
#include <iostream>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include "camera_info_manager/camera_info_manager.h"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection2DArray.h"
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

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
  ros::NodeHandle nh_;
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
    
    ROS_INFO("nn_width: %d, nn_height: %d, classes: %d, coordinates: %d, iou_threshold: %.2f, confidence_threshold: %.2f",
             nn_width, nn_height, classes, coordinates, iou_threshold, confidence_threshold);

    labels.clear();
    for (const auto& label : config["mappings"]["labels"]) {
      labels.push_back(label.as<std::string>());
      ROS_INFO("Found label: %s", label.as<std::string>().c_str());
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
  auto left_cam_detection_nn = pipeline.create<dai::node::YoloDetectionNetwork>();
  auto right_cam_image_xlink_out = pipeline.create<dai::node::XLinkOut>();
  auto left_cam_image_xlink_out = pipeline.create<dai::node::XLinkOut>();
  auto left_cam_det_xlink_out = pipeline.create<dai::node::XLinkOut>();

  right_cam_image_xlink_out->setStreamName("right_cam_image");
  left_cam_image_xlink_out->setStreamName("left_cam_image");
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
  right_cam->preview.link(right_cam_image_xlink_out->input);

  left_cam->preview.link(left_cam_detection_nn->input);
  left_cam->preview.link(left_cam_image_xlink_out->input);
  left_cam_detection_nn->out.link(left_cam_det_xlink_out->input);

  ROS_INFO("Pipeline created!");

  return pipeline;
}

ImagePublisher::ImagePublisher()
:
  nh_("~")
{
  ROS_INFO("Starting depthai image publisher...");

  // image config
  int fps = 20;
  nh_.param("fps", fps, 15);
  double focal_scale = 1.0;
  nh_.param("focal_scale", focal_scale, 1.0);

  // nn config
  std::string nn_model_path, nn_config_path;
  nh_.param("nn_model_path", nn_model_path, std::string("/catkin_ws/src/minimum_image_publisher/models/yolov6n_openvino_2022.1_6shave.blob"));
  nh_.param("nn_config_path", nn_config_path, std::string("/catkin_ws/src/minimum_image_publisher/config/yolov6n.yaml"));

  if (!checkFileExists(nn_model_path)) {
    ROS_ERROR("NN model not found!");
    ros::shutdown();
  }
  if (!checkFileExists(nn_config_path)) {
    ROS_ERROR("NN config not found!");
    ros::shutdown();
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
  std::shared_ptr<dai::DataOutputQueue> left_cam_detection_q = device.getOutputQueue("left_cam_detection", 10, false); 

  dai::rosBridge::ImageConverter right_cam_image_converter("/right_cam_frame", false);
  dai::rosBridge::ImageConverter left_cam_image_converter("/left_cam_frame", false);
  dai::rosBridge::ImgDetectionConverter left_cam_det_converter("/left_cam_frame", 1920, 1080, false);

  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> right_cam_imge_publisher(
      right_cam_image_q,
      nh_,
      std::string("/right_cam/image"),
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                &right_cam_image_converter,
                std::placeholders::_1,
                std::placeholders::_2),
      10, "", "");
  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> left_cam_image_publisher(
      left_cam_image_q,
      nh_,
      std::string("/left_cam/image"),
      std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                &left_cam_image_converter,
                std::placeholders::_1,
                std::placeholders::_2),
      10, "", "");
  dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections> left_cam_det_publisher(
      left_cam_detection_q,
      nh_,
      std::string("/left_cam/detections"),
      std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg,
                &left_cam_det_converter,
                std::placeholders::_1,
                std::placeholders::_2),
      10);
  right_cam_imge_publisher.addPublisherCallback();
  left_cam_image_publisher.addPublisherCallback();
  left_cam_det_publisher.addPublisherCallback();
  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_publisher_node");
  ImagePublisher image_publisher;
  return 0;
}
