
#pragma once

#include <memory>
#include <string>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"
#include "modules/planning/scenarios/obstacle_parking/obstacle_parking.h"

#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

class StageSurpassObstacle : public Stage {
public:
    bool Init(
            const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir,
            void* context);
    StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

private:
    bool CheckADCStop(const Frame& frame);
    bool IsObstacleBlockingPath(const Obstacle* obstacle);
    StageResult ExecuteSurpassObstacleTask(const common::TrajectoryPoint& planning_init_point, Frame* frame);
    StageResult ContinueToTargetParkingSpot(const common::TrajectoryPoint& planning_init_point, Frame* frame);

    ObstacleParkingConfig scenario_config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::StageSurpassObstacle, Stage)

}  // namespace planning
}  // namespace apollo