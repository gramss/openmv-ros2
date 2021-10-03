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

#ifndef OPENMV_BT_PLUGINS__PLUGINS__ACTION__SETUP_CAM_NORMAL_HPP_
#define OPENMV_BT_PLUGINS__PLUGINS__ACTION__SETUP_CAM_NORMAL_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "openmv_msgs/action/control.hpp"

namespace openmv_bt_plugins
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::BackUp
 */
class SetupCamNormal : public nav2_behavior_tree::BtActionNode<openmv_msgs::action::Control>
{
public:
  /**
   * @brief A constructor for openmv_bt_plugins::BackUpAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SetupCamNormal(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        // BT::InputPort<double>("backup_dist", 0.15, "Distance to backup"),
        // BT::InputPort<double>("backup_speed", 0.025, "Speed at which to backup")
      });
  }
};

}  // namespace openmv_bt_plugins

#endif  // OPENMV_BT_PLUGINS__PLUGINS__ACTION__SETUP_CAM_NORMAL_HPP_
