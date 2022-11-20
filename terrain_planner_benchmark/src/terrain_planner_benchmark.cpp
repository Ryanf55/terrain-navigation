
#include "terrain_planner_benchmark/terrain_planner_benchmark.h"
#include "terrain_planner/terrain_ompl_rrt.h"

TerrainPlannerBenchmark::TerrainPlannerBenchmark() {
  data_logger = std::make_shared<DataLogger>();
  data_logger->setPrintHeader(true);
  data_logger->setKeys({"id", "planning_method", "path_length"});
}

TerrainPlannerBenchmark::~TerrainPlannerBenchmark() {}

void TerrainPlannerBenchmark::runBenchmark(const int num_experiments) {
  // Set start and end goals
  TrajectorySegments path;
  std::vector<Eigen::Vector3d> interpolated_path;

  for (int i = 0; i < num_experiments; i++) {
    std::cout << "Running experiment: " << i << std::endl;

    /// TODO: Configure planner

    /// TODO: Run planner experiment
    // Initialize planner with loaded terrain map
    auto planner = std::make_shared<TerrainOmplRrt>();
    planner->setMap(map_);
    planner->setAltitudeLimits(150.0, 50.0);
    /// TODO: Get bounds from gridmap
    planner->setBoundsFromMap(map_->getGridMap());

    double terrain_altitude{100.0};

    const Eigen::Vector2d map_pos = map_->getGridMap().getPosition();
    const double map_width_x = map_->getGridMap().getLength().x();
    const double map_width_y = map_->getGridMap().getLength().y();

    Eigen::Vector3d start{Eigen::Vector3d(map_pos(0) - 0.4 * map_width_x, map_pos(1) - 0.4 * map_width_y, 0.0)};
    start(2) = map_->getGridMap().atPosition("elevation", Eigen::Vector2d(start(0), start(1))) + terrain_altitude;
    double start_yaw = 0.0;
    Eigen::Vector3d start_vel = 10.0 * Eigen::Vector3d(std::cos(start_yaw), std::sin(start_yaw), 0.0);
    Eigen::Vector3d goal{Eigen::Vector3d(map_pos(0) + 0.4 * map_width_x, map_pos(1) + 0.4 * map_width_y, 0.0)};
    goal(2) = map_->getGridMap().atPosition("elevation", Eigen::Vector2d(goal(0), goal(1))) + terrain_altitude;
    double goal_yaw = 0.0;
    Eigen::Vector3d goal_vel = 10.0 * Eigen::Vector3d(std::cos(goal_yaw), std::sin(goal_yaw), 0.0);

    planner->setupProblem(start, start_vel, goal, goal_vel);
    bool found_solution{false};
    while (!found_solution) {
      found_solution = planner->Solve(1.0, path);
    }
    planner->getSolutionPath(interpolated_path);

    // std::shared_ptr<ompl::base::PlannerData> planner_data = planner->getPlannerData();
    // planner_data->decoupleFromPlanner();

    double solution_path_length{0.0};
    double path_length{0.0};
    if (planner->getSolutionPathLength(path_length)) {
      solution_path_length = path_length;
    }

    BenchmarkResult result;
    result.id = i;
    result.planning_method = "RRT*";
    result.path_length = solution_path_length;
    results.push_back(result);
  }
}

void TerrainPlannerBenchmark::writeResultstoFile(const std::string& file_path) {
  for (auto result : results) {
    std::unordered_map<std::string, std::any> state;
    state.insert(std::pair<std::string, double>("id", result.id));
    state.insert(std::pair<std::string, std::string>("planning_method", result.planning_method));
    state.insert(std::pair<std::string, double>("path_length", result.path_length));
    data_logger->record(state);
  }
  data_logger->writeToFile(file_path);
}