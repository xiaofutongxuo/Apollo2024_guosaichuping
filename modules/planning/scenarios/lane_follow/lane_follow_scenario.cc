/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"

namespace apollo {
namespace planning {

bool LaneFollowScenario::IsTransferable(const Scenario* other_scenario, const Frame& frame) {
    // const auto& vehicle_state = frame.vehicle_state();
    // const auto& reference_line_info = frame.reference_line_info().front();
    // const auto& reference_line = reference_line_info.reference_line();
    // const double obstacle_distance_threshold = 15.0;
    // for (const auto& path_obstacle : reference_line_info.path_decision().obstacles().Items()) {
    //     const auto& obstacle = path_obstacle;
    //     if (obstacle->Id() == "9073" or obstacle->Id() == "9073_0") {
    //         common::SLPoint obstacle_sl_point;
    //         reference_line.XYToSL(obstacle->PerceptionBoundingBox().center(), &obstacle_sl_point);
    //         const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
    //         const double distance_to_obstacle = obstacle_sl_point.s() - adc_front_edge_s;
    //         AINFO << "识别到9073障碍物,距离为:" << distance_to_obstacle << "米";

    //         if (distance_to_obstacle <= obstacle_distance_threshold) {
    //             return false;
    //         } else {
    //             continue;
    //         }
    //     } else {
    //         continue;
    //     }
    // }

    if (!frame.local_view().planning_command->has_lane_follow_command()) {
        return false;
    }
    if (frame.reference_line_info().empty()) {
        return false;
    }
    if (other_scenario == nullptr) {
        return true;
    }
    return true;
}

}  // namespace planning
}  // namespace apollo
