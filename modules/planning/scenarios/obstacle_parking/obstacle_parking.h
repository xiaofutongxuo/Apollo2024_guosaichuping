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
 * @file obstacle_parking.h
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/common_msgs/map_msgs/map_id.pb.h"
#include "modules/planning/scenarios/obstacle_parking/proto/obstacle_parking.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

#include "gtest/gtest.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

struct ObstacleParkingContext : public ScenarioContext {
    ObstacleParkingConfig scenario_config;
    // std::string target_parking_spot_id;
    // bool pre_stop_rightaway_flag = false;
    // hdmap::MapPathPoint pre_stop_rightaway_point;
};

class ObstacleParking : public apollo::planning::Scenario {
public:
    bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) override;

    ObstacleParkingContext* GetContext() override {
        return &context_;
    }

    bool IsTransferable(const Scenario* const other_scenario, const Frame& frame) override;

private:
    bool init_ = false;
    ObstacleParkingContext context_;
    const hdmap::HDMap* hdmap_ = nullptr;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::ObstacleParking, apollo::planning::Scenario)

}  // namespace planning
}  // namespace apollo