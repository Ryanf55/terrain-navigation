#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <ompl/base/StateSampler.h>
#include <Eigen/Dense>

#include <terrain_navigation/terrain_map.h>

#include <terrain_planner/ompl_setup.h>
#include <terrain_planner/DubinsAirplane.hpp>


TEST(TerrainOmpl, CheckCollisionBelowTerrain) {

  using namespace ompl;

  // The original map data can be private, but best to find a public 
  // SRTM or TIF data. Try terrain_navigation_ros/resource/davosdorf.

  // The map comes from TerrainOmplRrt.setMap. See ompl_benchmark_node
  // which loads a map using grid_map_geo.
  auto const share_dir = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("terrain_planner"));
  auto const davsdorf_tif = share_dir / "resources" / "davosdorf.tif";

  auto terrain_map = std::make_shared<TerrainMap>();
  ASSERT_TRUE(terrain_map->initializeFromGeotiff(davsdorf_tif.string()));

  auto const surface_dist = 50.0;
  terrain_map->AddLayerDistanceTransform(50.0, "distance_surface");

  auto problem_setup = OmplSetup(
    base::StateSpacePtr(
      new fw_planning::spaces::DubinsAirplaneStateSpace()));

  problem_setup.clear();
  problem_setup.clearStartStates();

  problem_setup.setDefaultPlanner();
  problem_setup.setDefaultObjective();

  const bool check_max_alt = false;
  problem_setup.setTerrainCollisionChecking(terrain_map->getGridMap(), check_max_alt);

  // Construct Eigen::Vector3d for the query
  // gdallocationinfo terrain_planner/resources/davosdorf.tif 5 10
  //    Report:
  //     Location: (5P,10L)
  //     Band 1:
  //      Value: 2584.947265625
  auto const elevation_truth_height = 2584.947265625;

  // Check collision
  auto const is_above = true;
  EXPECT_FALSE(terrain_map->isInCollision(
    "distance_surface",
    Eigen::Vector3d(5, 10, elevation_truth_height + surface_dist + 1.0),
    is_above));

  EXPECT_TRUE(terrain_map->isInCollision(
    "distance_surface",
    Eigen::Vector3d(5, 10, elevation_truth_height + surface_dist - 1.0),
    is_above));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
