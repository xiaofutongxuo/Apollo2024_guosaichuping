
#include <string>
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/scenarios/obstacle_parking/stage_surpass_obstacle.h"

#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

bool StageSurpassObstacle::Init(
        const StagePipeline& config,
        const std::shared_ptr<DependencyInjector>& injector,
        const std::string& config_dir,
        void* context) {
    if (!Stage::Init(config, injector, config_dir, context)) {
        return false;
    }
    scenario_config_.CopyFrom(GetContextAs<ObstacleParkingContext>()->scenario_config);
    return true;
}

StageResult StageSurpassObstacle::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
    ADEBUG << "stage: StageSurpassObstacle";
    CHECK_NOTNULL(frame);
    StageResult result;
    auto scenario_context = GetContextAs<ObstacleParkingContext>();

    const auto& reference_line_info = frame->reference_line_info().front();
    const auto& reference_line = reference_line_info.reference_line();
    bool has_obstacle = false;
    for (const auto& path_obstacle : reference_line_info.path_decision().obstacles().Items()) {
        const auto& obstacle = path_obstacle;
        if (IsObstacleBlockingPath(obstacle)) {
            has_obstacle = true;
            break;
        }
        if (has_obstacle) {
            break;
        }
    }

    if (has_obstacle) {
        // 执行绕行任务
        result = ExecuteSurpassObstacleTask(planning_init_point, frame);
    } else {
        // 如果没有障碍物，继续正常行驶至目标停车位
        result = ContinueToTargetParkingSpot(planning_init_point, frame);
    }

    if (CheckADCStop(*frame)) {
        next_stage_ = "PARKING";
        return StageResult(StageStatusType::FINISHED);
    }

    if (result.HasError()) {
        AERROR << "StageSurpassObstacle planning error";
        return result.SetStageStatus(StageStatusType::ERROR);
    }

    return result.SetStageStatus(StageStatusType::RUNNING);
}

bool StageSurpassObstacle::IsObstacleBlockingPath(const Obstacle* obstacle) {
    // 障碍物是9073
    if (obstacle->Id() == "9073" or obstacle->Id() == "9073_0") {
        return true;
    } else {
        return false;
    }
}

StageResult StageSurpassObstacle::ExecuteSurpassObstacleTask(
        const common::TrajectoryPoint& planning_init_point,
        Frame* frame) {
    // 实现绕行障碍物的逻辑
    StageResult result;
    // 绕行任务
    // ...
    return result;
}

StageResult StageSurpassObstacle::ContinueToTargetParkingSpot(
        const common::TrajectoryPoint& planning_init_point,
        Frame* frame) {
    // 继续前往目标停车位的逻辑
    StageResult result;
    // 继续行驶
    // ...
    return result;
}

bool StageSurpassObstacle::CheckADCStop(const Frame& frame) {
    const auto& reference_line_info = frame.reference_line_info().front();
    const double adc_speed = injector_->vehicle_state()->linear_velocity();
    const double max_adc_stop_speed
            = common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param().max_abs_speed_when_stopped();
    if (adc_speed > max_adc_stop_speed) {
        ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
        return false;
    }

    const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    const double stop_fence_start_s = frame.open_space_info().open_space_pre_stop_fence_s();
    const double distance_stop_line_to_adc_front_edge = stop_fence_start_s - adc_front_edge_s;

    return true;
}

}  // namespace planning
}  // namespace apollo