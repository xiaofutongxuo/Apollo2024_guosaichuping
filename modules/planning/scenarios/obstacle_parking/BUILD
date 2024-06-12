load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "obstacle_parking_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libobstacle_parking.so",
    srcs = [
        "obstacle_parking.cc",
        "stage_surpass_obstacle.cc",
    ],
    hdrs = [
        "obstacle_parking.h",
        "stage_surpass_obstacle.h",
    ],
    opts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugin_obstacle_parking_description.xml",
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/obstacle_parking/proto:obstacle_parking_proto",

    ],
)

apollo_package()

cpplint()