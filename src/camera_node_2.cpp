#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <ros_parameters/CameraNodeConfig.h>

class CameraNode {
  using CameraNodeConfig = ros_parameters::CameraNodeConfig;
  using CameraNodeDynConfServer = dynamic_reconfigure::Server<CameraNodeConfig>;
  using CameraNodeDynConfServerPtr = std::shared_ptr<CameraNodeDynConfServer>;

 public:
  CameraNode() : pnh_("~") {
    logger_prefix_ = ros::this_node::getName() + ": ";
    ros::NodeHandle camera_nh(gnh_, "camera");

    // Initialize publisher
    it_ = std::make_shared<image_transport::ImageTransport>(camera_nh);
    img_publisher_ = it_->advertise("image_raw", 1);

    // Initialize publish timer
    img_timer_ = gnh_.createTimer(ros::Rate(1), &CameraNode::imgTimerCallback,
                                  this, false, false);

    // Initialize dynamic reconfigure server
    dynconf_server_ = std::make_shared<CameraNodeDynConfServer>(camera_nh);
    dynconf_server_->setCallback(
        boost::bind(&CameraNode::dynConfCallback, this, _1, _2));
  }

 private:
  void dynConfCallback(CameraNodeConfig& config, uint32_t /*level*/) {
    std::string pkg_path = getSourcePackagePath(config.source_pkg);
    std::string filename = pkg_path + "/" + config.source_img;

    // Read image
    cv::Mat image;
    getImage(filename, image);

    // Initialize message
    image_msg_ =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_msg_->header.frame_id = config.frame_id;

    // Update publish rate
    img_timer_.setPeriod(ros::Duration(1.0 / config.rate));
    img_timer_.start();
  }

  std::string getSourcePackagePath(const std::string& pkg_name) {
    std::string pkg_path = ros::package::getPath(pkg_name);
    if (pkg_path.empty())
      ROS_ERROR_STREAM(logger_prefix_ << "Failed to get package path");
    return pkg_path;
  }

  bool getImage(const std::string& filename, cv::Mat& image) {
    image = cv::imread(filename, cv::IMREAD_COLOR);
    if (image.size().area() > 0) return true;
    ROS_ERROR_STREAM(logger_prefix_ << "Failed to read source image "
                                    << filename);
    return false;
  }

  void imgTimerCallback(const ros::TimerEvent& /*event*/) {
    image_msg_->header.stamp = ros::Time::now();
    img_publisher_.publish(image_msg_);
  }

  std::string logger_prefix_;

  ros::NodeHandle gnh_, pnh_;
  CameraNodeDynConfServerPtr dynconf_server_;
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
