#include <rclcpp/rclcpp.hpp>

#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>

#include <tf2_ros/transform_listener.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
//#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/srv/empty.hpp>

#include <rct_image_tools/image_observation_finder.h>

#include <vector>

class TransformMonitor
{
public:
  TransformMonitor(const std::string& base_frame, const std::string& tool_frame, std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , base_frame_(base_frame)
    , tool_frame_(tool_frame)
    , buffer_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME))
    , listener_(buffer_)
  {
    // Validate that we can look up required transforms
    geometry_msgs::msg::TransformStamped dummy;
    if (!capture(dummy))
    {
      throw std::runtime_error("Transform from " + base_frame_ + " to " + tool_frame_ + " not available");
    }
  }

  bool capture(geometry_msgs::msg::TransformStamped& out)
  {
    try
    {
      geometry_msgs::msg::TransformStamped t = buffer_.lookupTransform(base_frame_, tool_frame_, rclcpp::Clock().now(), std::chrono::seconds(3));
      out = t;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to compute transform");
//      ROS_WARN_STREAM("Failed to compute transfrom between " << base_frame_ << " and " << tool_frame_ << ": "
//                      << ex.what());
      return false;
    }
  }

private:
  std::string base_frame_;
  std::string tool_frame_;

  std::shared_ptr<rclcpp::Node> node_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

class ImageMonitor
{
public:
  ImageMonitor(const rct_image_tools::ModifiedCircleGridObservationFinder& finder,
               const std::string& nominal_image_topic,
               rclcpp::Node::SharedPtr node)
    : finder_(finder)
    , it_(node)
  {
    im_sub_ = it_.subscribe(nominal_image_topic, 1, &ImageMonitor::onNewImage, this);
    im_pub_ = it_.advertise(nominal_image_topic + "_observer", 1);
  }

  void onNewImage(const sensor_msgs::msg::ImageConstPtr& msg)
  {
//    ROS_INFO_STREAM("New image");
    RCLCPP_INFO(node_->get_logger(), "New image");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if(msg->encoding == "mono16")
      {
        cv_bridge::CvImagePtr temp_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::msg::image_encodings::MONO16);

        cv::Mat img_conv;
        cv::cvtColor(temp_ptr->image, img_conv, CV_GRAY2BGR);
        img_conv.convertTo(img_conv, CV_8UC1);
        cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage(temp_ptr->header, sensor_msgs::msg::image_encodings::BGR8, img_conv));
      }
      else
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::msg::image_encodings::BGR8);
      }

      auto obs = finder_.findObservations(cv_ptr->image);
      if (obs)
      {
        auto modified = finder_.drawObservations(cv_ptr->image, *obs);
        cv_bridge::CvImagePtr ptr (new cv_bridge::CvImage(cv_ptr->header, cv_ptr->encoding, modified));
        im_pub_.publish(ptr->toImageMsg());
      }
      else
      {
        im_pub_.publish(cv_ptr->toImageMsg());
      }
      last_frame_ = cv_ptr;
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "BAD THING HAPPEND");
//      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  bool capture(cv::Mat& frame)
  {
    if (last_frame_)
    {
      frame = last_frame_->image;
      return true;
    }
    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rct_image_tools::ModifiedCircleGridObservationFinder finder_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber im_sub_;
  image_transport::Publisher im_pub_;
  cv_bridge::CvImagePtr last_frame_;
};

struct DataCollectionConfig
{
  std::string base_frame;
  std::string tool_frame;

  std::string image_topic;
  rct_image_tools::ModifiedCircleGridTarget target;

  std::string save_dir;
};

struct DataCollection
{

  DataCollection(const DataCollectionConfig& config, std::shared_ptr<rclcpp::Node> node)
    : node_(node)
    , trigger_server_(node -> create_service<std_srvs::srv::Empty>("collect", std::bind(&DataCollection::onTrigger, this, std::placeholders::_1, std::placeholders::_2)))
    , save_server_(node -> create_service<std_srvs::srv::Empty>("save", std::bind(&DataCollection::onSave, this, std::placeholders::_1, std::placeholders::_2)))
    , tf_monitor(config.base_frame, config.tool_frame)
    , image_monitor(config.target, config.image_topic)
    , save_dir_(config.save_dir)
  {
//    ros::NodeHandle nh;
//    trigger_server = nh.advertiseService("collect", &DataCollection::onTrigger, this);
//    save_server = nh.advertiseService("save", &DataCollection::onSave, this);

//    ROS_INFO_STREAM("Call " << trigger_server.getService() << " to capture a pose/image pair");
//    ROS_INFO_STREAM("Call " << save_server.getService() << " to save the captured data");
  }

  bool onTrigger(std_srvs::srv::Empty::Request&, std_srvs::srv::Empty::Response&)
  {
    RCLCPP_INFO(node_->get_logger(), "Pose/Image capture triggered...");
    geometry_msgs::msg::TransformStamped pose;
    cv::Mat image;

    if (tf_monitor.capture(pose) && image_monitor.capture(image))
    {
      poses.push_back(pose);
      images.push_back(image);
      RCLCPP_INFO(node_->get_logger(), "Data collected successfully");
      return true;
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to capture pose/image pair");
      return false;
    }
  }

  bool onSave(std_srvs::srv::Empty::Request&, std_srvs::srv::Empty::Response&)
  {
    rct_ros_tools::ExtrinsicDataSet data;
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
      cv::Mat image = images[i];
      auto msg = poses[i];
      Eigen::Affine3d pose;
      tf::transformMsgToEigen(msg.transform, pose);

      data.images.push_back(image);
      data.tool_poses.push_back(pose);
    }

//    ROS_INFO_STREAM("Saving data-set to " << save_dir_);
    rct_ros_tools::saveToDirectory(save_dir_, data);
    return true;
  }

  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr trigger_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_server_;

//  ros::ServiceServer trigger_server;
//  ros::ServiceServer save_server;

  std::vector<geometry_msgs::msg::TransformStamped> poses;
  std::vector<cv::Mat> images;

  TransformMonitor tf_monitor;
  ImageMonitor image_monitor;

  std::string save_dir_;
};

template <typename T>
bool get(const rclcpp::SyncParametersClient::SharedPtr& pc, const std::string& key, T& value)
{
    try {
        value = pc->get_parameter<T>(key);
        return true;
    }
    catch (std::runtime_error &e)
    {
        return false;
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("command_line_cal_node");

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
//  ros::init(argc, argv, "rct_examples");
//  ros::NodeHandle pnh ("~");

  // Load data collection parameters
  DataCollectionConfig config;

  if (!get(parameters_client, "base_frame", config.base_frame)) return 1;
  if (!get(parameters_client, "tool_frame", config.tool_frame)) return 1;
  if (!get(parameters_client, "image_topic", config.image_topic)) return 1;
  if (!get(parameters_client, "save_dir", config.save_dir)) return 1;
//  if (!rct_ros_tools::loadTarget(parameters_client, "target_definition", config.target))
//  {
//    RCLCPP_ERROR(node->get_logger(), "Must provide parameters to load target!");
//    return 1;
//  }

  DataCollection dc (config);
  rclcpp::spin(node);
  return 0;
}
