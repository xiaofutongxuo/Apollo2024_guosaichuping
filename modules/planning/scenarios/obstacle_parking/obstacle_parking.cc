/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/******************************************************************************
 * @file obstacle_parking.cc
 *****************************************************************************/

#include <memory>
#include "modules/planning/scenarios/obstacle_parking/obstacle_parking.h"

#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

bool ObstacleParking::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<ObstacleParkingConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);
    init_ = true;
    return true;
}

bool ObstacleParking::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;
    }

    // 获取车辆状态和参考线信息
    const auto& vehicle_state = frame.vehicle_state();
    const auto& reference_line_info = frame.reference_line_info().front();
    const auto& reference_line = reference_line_info.reference_line();
    // 目标障碍物ID以及障碍物距离阈值
    const std::string target_obstacle_id = "9073";
    AINFO << "target_obstacle_id: " << target_obstacle_id;
    const double obstacle_distance_threshold = 15.0;

    for (const auto& path_obstacle : reference_line_info.path_decision().obstacles().Items()) {
        const auto& obstacle = path_obstacle;
        if (obstacle->Id() == target_obstacle_id) {
            // 计算车辆前端到障碍物的距离
            common::SLPoint obstacle_sl_point;
            reference_line.XYToSL(obstacle->PerceptionBoundingBox().center(), &obstacle_sl_point);
            const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
            const double distance_to_obstacle = obstacle_sl_point.s() - adc_front_edge_s;
            AINFO << "识别到9073障碍物,距离为:" << distance_to_obstacle << "米";

            // 判断距离是否在阈值范围内
            if (distance_to_obstacle <= obstacle_distance_threshold) {
                AINFO << "Target Obstacle:" << target_obstacle_id << " is within distance threshold";
                return true;
            }
        }
    }

    return false;
}

}  // namespace planning
}  // namespace apollo