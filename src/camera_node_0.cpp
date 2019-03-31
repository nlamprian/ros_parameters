#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class CameraNode {
 public:
  CameraNode() {
    logger_prefix_ = ros::this_node::getName() + ": ";

    // Initialize publisher
    ros::NodeHandle camera_nh(nh_, "camera");
    it_ = std::make_shared<image_transport::ImageTransport>(camera_nh);
    img_publisher_ = it_->advertise("image_raw", 1);

    // Read image
    std::string pkg_path = ros::package::getPath("ros_parameters");
    cv::Mat image = cv::imread(pkg_path + "/data/ros.jpg", cv::IMREAD_COLOR);
    if (image.size().area() == 0) {
      ROS_FATAL_STREAM(logger_prefix_ << "Failed to read source image");
      return;
    }

    // Initialize message
    image_msg_ =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_msg_->header.frame_id = "camera_link";

    // Start publish loop
    img_timer_ =
        nh_.createTimer(ros::Rate(30), &CameraNode::imgTimerCallback, this);
    ROS_INFO_STREAM(logger_prefix_ << "Node is initialized");
  }

 private:
  void imgTimerCallback(const ros::TimerEvent& /*event*/) {
    image_msg_->header.stamp = ros::Time::now();
    img_publisher_.publish(image_msg_);
  }

  std::string logger_prefix_;

  ros::NodeHandle nh_;
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
