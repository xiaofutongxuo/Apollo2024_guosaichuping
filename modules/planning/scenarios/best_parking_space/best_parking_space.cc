/******************************************************************************
 * @file best_parking_space.cc
 *****************************************************************************/

#include "modules/planning/scenarios/best_parking_space/best_parking_space.h"

#include "modules/planning/planning_base/common/frame.h"

// #include "modules/planning/scenarios/best_parking_space/stage_approaching_parking_spot.h"
// #include "modules/planning/scenarios/best_parking_space/stage_parking.h"

namespace apollo {
namespace planning {

using apollo::common::VehicleState;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;
using apollo::hdmap::ParkingSpaceInfoConstPtr;
using apollo::hdmap::Path;
using apollo::hdmap::PathOverlap;

bool BestParkingSpaceScenario::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
    if (init_) {
        return true;
    }

    if (!Scenario::Init(injector, name)) {
        AERROR << "failed to init scenario" << Name();
        return false;
    }

    if (!Scenario::LoadConfig<BestParkingSpaceConfig>(&context_.scenario_config)) {
        AERROR << "fail to get config of scenario" << Name();
        return false;
    }
    hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
    CHECK_NOTNULL(hdmap_);
    init_ = true;
    return true;
}

bool BestParkingSpaceScenario::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
    std::string target_parking_spot_id;
    double parking_spot_range_to_start = context_.scenario_config.parking_spot_range_to_start();
    auto parking_command = frame.local_view().planning_command->has_parking_command();
    auto parking_spot_id = frame.local_view().planning_command->parking_command().has_parking_spot_id();

    if (other_scenario == nullptr || frame.reference_line_info().empty()) {
        return false;

        if (parking_command && parking_spot_id) {
            target_parking_spot_id = frame.local_view().planning_command->parking_command().parking_spot_id();
        }
    }

    if (!parking_command) {
        const auto routing_end = frame.local_view().end_lane_way_point;
        common::PointENU end_postion;
        end_postion.set_x(routing_end->pose().x());
        end_postion.set_y(routing_end->pose().y());
        if (nullptr == routing_end) {
            return false;
        }

        common::SLPoint dest_sl;
        const auto& reference_line_info = frame.reference_line_info().front();
        const auto& reference_line = reference_line_info.reference_line();
        reference_line.XYToSL(routing_end->pose(), &dest_sl);
        const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
        const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;

        if (adc_distance_to_dest > parking_spot_range_to_start) {
            return false;
        }

        if (target_parking_spot_id.empty()) {
            std::vector<apollo::hdmap::ParkingSpaceInfoConstPtr> parking_spaces;

            if (hdmap_->GetParkingSpaces(end_postion, parking_spot_range_to_start, &parking_spaces) == 0
                && parking_spaces.size() > 0) {
                for (auto parking_space : parking_spaces) {
                    target_parking_spot_id = parking_space->parking_space().id().id();
                }
            }
        }
    }

    if (target_parking_spot_id.empty()) {
        return false;
    }

    AINFO << "target_parking_spot_id" << target_parking_spot_id;

    const auto& nearby_path = frame.reference_line_info().front().reference_line().map_path();
    PathOverlap parking_space_overlap;
    const auto& vehicle_state = frame.vehicle_state();

    if (!SearchTargetParkingSpotOnPath(nearby_path, target_parking_spot_id, &parking_space_overlap)) {
        ADEBUG << "No such parking spot found after searching all path forward "
                  "possible"
               << target_parking_spot_id;
        return false;
    }
    if (!CheckDistanceToParkingSpot(
                frame, vehicle_state, nearby_path, parking_spot_range_to_start, parking_space_overlap)) {
        ADEBUG << "target parking spot found, but too far, distance larger than "
                  "pre-defined distance"
               << target_parking_spot_id;
        return false;
    }
    context_.target_parking_spot_id = target_parking_spot_id;
    return true;
}

bool BestParkingSpaceScenario::SearchTargetParkingSpotOnPath(
        const Path& nearby_path,
        const std::string& target_parking_id,
        PathOverlap* parking_space_overlap) {
    const auto& parking_space_overlaps = nearby_path.parking_space_overlaps();
    for (const auto& parking_overlap : parking_space_overlaps) {
        if (parking_overlap.object_id == target_parking_id) {
            *parking_space_overlap = parking_overlap;
            return true;
        }
    }
    return false;
}

bool BestParkingSpaceScenario::CheckDistanceToParkingSpot(
        const Frame& frame,
        const VehicleState& vehicle_state,
        const Path& nearby_path,
        const double parking_start_range,
        const PathOverlap& parking_space_overlap) {
    // TODO(Jinyun) parking overlap s are wrong on map, not usable
    const hdmap::HDMap* hdmap = hdmap::HDMapUtil::BaseMapPtr();
    hdmap::Id id;
    double center_point_s, center_point_l;
    id.set_id(parking_space_overlap.object_id);
    ParkingSpaceInfoConstPtr target_parking_spot_ptr = hdmap->GetParkingSpaceById(id);
    Vec2d left_bottom_point = target_parking_spot_ptr->polygon().points().at(0);
    Vec2d right_bottom_point = target_parking_spot_ptr->polygon().points().at(1);
    Vec2d right_top_point = target_parking_spot_ptr->polygon().points().at(2);
    Vec2d left_top_point = target_parking_spot_ptr->polygon().points().at(3);
    Vec2d center_point = (left_bottom_point + right_bottom_point + right_top_point + left_top_point) / 4.0;
    nearby_path.GetNearestPoint(center_point, &center_point_s, &center_point_l);
    double vehicle_point_s = 0.0;
    double vehicle_point_l = 0.0;
    Vec2d vehicle_vec(vehicle_state.x(), vehicle_state.y());
    nearby_path.GetNearestPoint(vehicle_vec, &vehicle_point_s, &vehicle_point_l);
    if (std::abs(center_point_s - vehicle_point_s) < parking_start_range) {
        return true;
    }
    return false;
}

}  // namespace planning
}  // namespace apollo
