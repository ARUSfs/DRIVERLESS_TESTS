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

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <condition_variable>
#include <mutex>

class PerceptionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    // Create a test node
    node = std::make_shared<rclcpp::Node>("rosbag_playback_test");

    // Initialize Variables
    map_received = false;

    // Set up a subscription to /perception/map
    map_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/perception/map",
        10,
        std::bind(&PerceptionTest::perception_callback, this, std::placeholders::_1));
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  void perception_callback(sensor_msgs::msg::PointCloud2 detected_cones)
  {
    ASSERT_FALSE(detected_cones.data.empty()) << "Cone data was empty";
    map_received = true;
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

  rclcpp::Node::SharedPtr node;
  std::unordered_set<std::string> allowed_topics;
  sensor_msgs::msg::PointCloud2 final_map;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription;

  bool map_received = false;
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
