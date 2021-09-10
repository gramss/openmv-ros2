// Copyright (c) 2018 Intel Corporation
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
#include <memory>

#include "openmv_bt_plugins/plugins/action/setup_cam_normal.hpp"

namespace openmv_bt_plugins
{

SetupCamNormal::SetupCamNormal(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<openmv_msgs::action::Control>(xml_tag_name, action_name, conf)
{
  // double dist;
  // getInput("backup_dist", dist);
  // double speed;
  // getInput("backup_speed", speed);

  // Populate the input message
  goal_.function = 4;
}

void SetupCamNormal::on_tick()
{
}

}  // namespace openmv_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<openmv_bt_plugins::SetupCamNormal>(
        name, "openmv_interface", config);
    };

  factory.registerBuilder<openmv_bt_plugins::SetupCamNormal>("SetupCamNormal", builder);
}
