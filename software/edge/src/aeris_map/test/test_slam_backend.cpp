#include <gtest/gtest.h>

#include "aeris_map/slam_backend.hpp"

namespace sb = aeris_map::slam_backend;

TEST(SlamBackend, CreatesVioBackendByDefault)
{
  auto backend = sb::make_backend("");
  const auto contract = backend->contract();
  EXPECT_EQ(contract.mode, "vio");
  EXPECT_EQ(contract.frame_chain, "map -> odom -> base_link");
  EXPECT_GE(contract.expected_input_topics.size(), 4U);
  EXPECT_GE(contract.produced_outputs.size(), 3U);
}

TEST(SlamBackend, RejectsUnsupportedModeWithoutFallback)
{
  const auto result = sb::start_backend("unsupported", "occupancy", "/map", "/rtabmap/cloud_map");
  EXPECT_FALSE(result.status.ready);
  EXPECT_EQ(result.status.code, sb::StartupErrorCode::kInvalidConfig);
  EXPECT_NE(result.status.message.find("Unsupported slam_mode"), std::string::npos);
}

TEST(SlamBackend, ActivatesVioWithHybridStreams)
{
  const auto result = sb::start_backend("vio", "hybrid", "/map", "/rtabmap/cloud_map");
  ASSERT_TRUE(result.status.ready) << result.status.message;
  EXPECT_EQ(result.activation.contract.mode, "vio");
  EXPECT_TRUE(result.activation.use_occupancy);
  EXPECT_TRUE(result.activation.use_point_cloud);
  EXPECT_EQ(result.activation.map_source, "hybrid");
}

TEST(SlamBackend, ValidatesRequiredTopicsPerSource)
{
  const auto occupancy_result = sb::start_backend("vio", "occupancy", "", "/rtabmap/cloud_map");
  EXPECT_FALSE(occupancy_result.status.ready);
  EXPECT_EQ(occupancy_result.status.code, sb::StartupErrorCode::kInvalidConfig);

  const auto cloud_result = sb::start_backend("vio", "point_cloud", "/map", "");
  EXPECT_FALSE(cloud_result.status.ready);
  EXPECT_EQ(cloud_result.status.code, sb::StartupErrorCode::kInvalidConfig);
}

TEST(SlamBackend, RegistersLiosamAsNotImplemented)
{
  auto backend = sb::make_backend("liosam");
  const auto contract = backend->contract();
  EXPECT_EQ(contract.mode, "liosam");

  const auto result = sb::start_backend("liosam", "occupancy", "/map", "/rtabmap/cloud_map");
  EXPECT_FALSE(result.status.ready);
  EXPECT_EQ(result.status.code, sb::StartupErrorCode::kNotImplemented);
  EXPECT_NE(result.status.message.find("not implemented"), std::string::npos);
}

TEST(SlamBackend, ReportsCodeNames)
{
  EXPECT_EQ(sb::startup_error_code_name(sb::StartupErrorCode::kNone), "none");
  EXPECT_EQ(sb::startup_error_code_name(sb::StartupErrorCode::kInvalidConfig), "invalid-config");
  EXPECT_EQ(sb::startup_error_code_name(sb::StartupErrorCode::kUnavailableTopics), "unavailable-topics");
  EXPECT_EQ(sb::startup_error_code_name(sb::StartupErrorCode::kStartupFailure), "startup-failure");
  EXPECT_EQ(sb::startup_error_code_name(sb::StartupErrorCode::kNotImplemented), "not-implemented");
}
