// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include "include/param_node.h"

ParametersClass::ParametersClass(SearchCfg *cfg, const std::string &node_name)
    : Node(node_name) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ParametersClass node construct");
  cfg_ = cfg;
  if (cfg_) {
    this->declare_parameter<int>("detect_area_height", cfg_->area_height);
    this->declare_parameter<int>("detect_area_width", cfg_->area_width);
    this->declare_parameter<int>("area_ingored_bottom", cfg_->ingored_bottom);
    this->declare_parameter<float>("parking_space_thread", cfg_->parking_thread);
    this->declare_parameter<float>("path_space_thread", cfg_->path_thread);
    this->declare_parameter<float>("move_step", cfg_->move_step);
    this->declare_parameter<float>("rotate_step", cfg_->rotate_step);
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&ParametersClass::Respond, this));
  }
}

void ParametersClass::Respond() {
  if (!cfg_) return;
  this->get_parameter("detect_area_height", cfg_->area_height);
  this->get_parameter("detect_area_width", cfg_->area_width);
  this->get_parameter("area_ingored_bottom", cfg_->ingored_bottom);
  this->get_parameter("parking_space_thread", cfg_->parking_thread);
  this->get_parameter("path_space_thread", cfg_->path_thread);
  this->get_parameter("move_step", cfg_->move_step);
  this->get_parameter("rotate_step", cfg_->rotate_step);

  std::stringstream ss;
  ss << "detect_area_height: " << cfg_->area_height << std::endl;
  ss << "detect_area_width: " << cfg_->area_width << std::endl;
  ss << "move_step: " << cfg_->move_step << std::endl;
  ss << "parking_space_thread: " << cfg_->parking_thread << std::endl;
  ss << "path_space_thread: " << cfg_->path_thread << std::endl;
  ss << "move_step: " << cfg_->move_step << std::endl;
  ss << "rotate_step: " << cfg_->rotate_step << std::endl;
  if (first_dump_config_) {
    first_dump_config_ = false;
    RCLCPP_WARN(this->get_logger(), "SearchCfg param are\n%s", ss.str().data());
  } else {
    RCLCPP_DEBUG(this->get_logger(), "SearchCfg param are\n%s", ss.str().data());
  }
}
