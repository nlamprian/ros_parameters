#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class CameraNode {
 public:
  CameraNode() : pnh_("~") {
    logger_prefix_ = ros::this_node::getName() + ": ";

    // Initialize publisher
    ros::NodeHandle camera_nh(gnh_, "camera");
    it_ = std::make_shared<image_transport::ImageTransport>(camera_nh);
    img_publisher_ = it_->advertise("image_raw", 1);

    // Read image
    cv::Mat image;
    bool success = getImage(getSourcePackagePath(), getSourceImg(), image);
    if (not success) return;

    // Initialize message
    image_msg_ =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_msg_->header.frame_id = getFrameId();

    // Start publish loop
    img_timer_ =
        gnh_.createTimer(getPublishRate(), &CameraNode::imgTimerCallback, this);
    ROS_INFO_STREAM(logger_prefix_ << "Node is initialized");
  }

 private:
  std::string getSourcePackagePath() {
    std::string source_pkg;
    if (pnh_.hasParam("source_pkg")) {
      pnh_.getParam("source_pkg", source_pkg);
      return ros::package::getPath(source_pkg);
    } else {
      ROS_ERROR_STREAM(logger_prefix_ << "Failed to get source_pkg");
      return std::string();
    }
  }

  std::string getSourceImg() {
    std::string source_img;
    if (pnh_.hasParam("source_img"))
      pnh_.getParam("source_img", source_img);
    else
      ROS_ERROR_STREAM(logger_prefix_ << "Failed to get source_img");
    return source_img;
  }

  std::string getFrameId() {
    std::string frame_id;
    if (pnh_.hasParam("frame_id"))
      pnh_.getParam("frame_id", frame_id);
    else
      ROS_WARN_STREAM(logger_prefix_ << "Failed to get frame_id");
    return frame_id;
  }

  ros::Rate getPublishRate() {
    double rate = 30;
    if (pnh_.hasParam("rate"))
      pnh_.getParam("rate", rate);
    else
      ROS_WARN_STREAM(logger_prefix_
                      << "Failed to get rate; defaulting to 30 Hz");
    return ros::Rate(rate);
  }

  bool getImage(const std::string& pkg_path, const std::string& img_path,
                cv::Mat& image) {
    std::string filename = pkg_path + "/" + img_path;
    image = cv::imread(filename, cv::IMREAD_COLOR);
    if (image.size().area() > 0) return true;
    ROS_FATAL_STREAM(logger_prefix_ << "Failed to read source image "
                                    << filename);
    return false;
  }

  void imgTimerCallback(const ros::TimerEvent& /*event*/) {
    image_msg_->header.stamp = ros::Time::now();
    img_publisher_.publish(image_msg_);
  }

  std::string logger_prefix_;

  ros::NodeHandle gnh_, pnh_;
  ros::Timer img_timer_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher img_publisher_;
  sensor_msgs::ImagePtr image_msg_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_node");
  CameraNode node;
  ros::spin();
  return 0;
}
