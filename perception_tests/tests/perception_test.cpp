#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rmw/serialized_message.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class PerceptionTest : public ::testing::Test
{
protected:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;

  rclcpp::Node::SharedPtr node;
  std::unordered_set<std::string> allowed_topics;
  sensor_msgs::msg::PointCloud2 final_map;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription;

  bool map_received = false;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_map_pub;

  void SetUp() override
  {

    rclcpp::init(0, nullptr);
    // Create a test node
    node = std::make_shared<rclcpp::Node>("rosbag_playback_test");

    create_static_tf();

    // Initialize Variables
    map_received = false;

    // Set up a subscription to /perception/map
    map_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/map",
        10,
        std::bind(&PerceptionTest::perception_callback, this, std::placeholders::_1));

    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    transformed_map_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/perception/test/final_map", 10);
  }

  void create_static_tf()
  {
    static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

    // Create static transform: slam/vehicle → rslidar
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = node->now();
    static_transform.header.frame_id = "slam/vehicle";
    static_transform.child_frame_id = "rslidar";

    // Set the relative position of rslidar on the vehicle (example values)
    static_transform.transform.translation.x = 1.5;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;

    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    static_broadcaster->sendTransform(static_transform);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  void perception_callback(sensor_msgs::msg::PointCloud2 detected_cones)
  {
    ASSERT_FALSE(detected_cones.data.empty()) << "Cone data was empty";
    map_received = true;

    if (final_map.data.empty())
    {
      RCLCPP_WARN(node->get_logger(), "Final map not loaded yet, skipping transform.");
      return;
    }

    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer->lookupTransform("rslidar", "arussim/world", rclcpp::Time(0), tf2::durationFromSec(1.0));

      sensor_msgs::msg::PointCloud2 transformed_cloud;
      tf2::doTransform(final_map, transformed_cloud, transform_stamped);

      cropping(transformed_cloud);

      transformed_map_pub->publish(transformed_cloud);
      RCLCPP_INFO(node->get_logger(), "Published transformed final map.");
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(node->get_logger(), "Could not transform final map: %s", ex.what());
      return;
    }
  }

  void cropping(sensor_msgs::msg::PointCloud2 &transformed_cloud)
  {
    // Configure the cropping filter

    double Mx = 30;
    double My = 15;
    double Mz = 0.5;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

    pcl::CropBox<pcl::PointXYZI> crop_box_filter;
    crop_box_filter.setInputCloud(pcl_cloud);
    crop_box_filter.setMin(Eigen::Vector4f(0, -My, -100.0, 1.0));
    crop_box_filter.setMax(Eigen::Vector4f(Mx, My, Mz, 1.0));

    // Store the cropped cloud
    crop_box_filter.filter(*pcl_cloud);

    pcl::toROSMsg(*pcl_cloud, transformed_cloud);

    transformed_cloud.header.frame_id = "rslidar";
    transformed_cloud.header.stamp = node->now();
  }

  void play_rosbag(const std::string &bag_path)
  {

    // Setup a rosbag reader
    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options{"cdr", "cdr"};
    reader.open(storage_options, converter_options);

    // Preregister Topics and Types
    auto topics_and_types = reader.get_all_topics_and_types();
    std::unordered_map<std::string, std::string> topic_type_map;
    for (const auto &info : topics_and_types)
    {
      topic_type_map[info.name] = info.type;
    }

    std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers;

    // Set up the executor
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (reader.has_next())
    {
      auto bag_msg = reader.read_next();
      const std::string &topic = bag_msg->topic_name;

      // Skip until the final map has information
      if (final_map.data.empty())
      {

        if (topic == "/slam/final_map")
        {

          // Deserialize to actual PointCloud2 message
          rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
          sensor_msgs::msg::PointCloud2 msg;
          rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
          serializer.deserialize_message(&serialized_msg, &msg);
          // Save to final_map
          final_map = msg;
        }
        else
        {
          continue;
        }
      }

      if (publishers.find(topic) == publishers.end())
      {
        auto it = topic_type_map.find(topic);
        if (it == topic_type_map.end())
        {
          RCLCPP_WARN(node->get_logger(), "Unknown type for topic: %s", topic.c_str());
          continue;
        }

        auto publisher = node->create_generic_publisher(topic, it->second, 10);
        publishers[topic] = publisher;
      }

      rclcpp::SerializedMessage msg(*bag_msg->serialized_data);
      publishers[topic]->publish(msg);

      if (topic == "/rslidar_points")
      {
        // Spin until /perception/map is received
        auto start = std::chrono::steady_clock::now();
        int kTimeOut = 10;
        while (!map_received &&
               std::chrono::steady_clock::now() - start < std::chrono::seconds(kTimeOut))
        {
          executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!map_received)
        {
          FAIL() << "Timeout waiting for /perception/map response.";
        }

        // Reset
        map_received = false;
      }
    }
  }
};

TEST_F(PerceptionTest, FilterAndPublishMcapRosbagWithMapWait)
{
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("perception_tests");
  std::string rosbag_path = package_share_directory + "/data/cartuja.mcap";
  try
  {
    play_rosbag(rosbag_path);
  }
  catch (const std::exception &e)
  {
    FAIL() << "Exception caught during rosbag playback: " << e.what();
  }
  catch (...)
  {
    FAIL() << "Unknown exception caught during rosbag playback.";
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
