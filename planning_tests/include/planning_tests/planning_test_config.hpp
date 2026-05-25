#ifndef PLANNING_TEST_CONFIG_HPP_
#define PLANNING_TEST_CONFIG_HPP_

#include <string>

namespace planning_test_config {

// Umbrales de evaluación
constexpr double kMinDetectedAccuracy = 0.8;
constexpr double kDistanceThreshold = 0.5;
constexpr int kMaxEvaluations = 50;
constexpr int kTimeOutSeconds = 10;
constexpr int kMaxTimeoutAttempts = 2;

// Nombres de paquetes y Launch files
const std::string kPathPlanningPackage = "path_planning";
const std::string kPathPlanningLaunch = "path_planning_launch.py";
const std::string kTestPackageName = "planning_tests";

// Tópicos de ROS 2
const std::string kTrayectoryTopic = "/path_planning/trajectory";
const std::string kSlamMapTopic = "/slam/map";

} // namespace planning_test_config

#endif // PLANNING_TEST_CONFIG_HPP_