# Copyright 2021 Florian Gramß
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# When run from container, please consider this and not just -priviliged:
# http://marc.merlins.org/perso/linux/post_2018-12-20_Accessing-USB-Devices-In-Docker-_ttyUSB0_-dev-bus-usb-_-for-fastboot_-adb_-without-using-privileged.html



import time

from openmv_msgs.action import Control
from openmv_msgs.msg import QRCode

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from openmv_driver_res import openmv_interface as oi

from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class OpenMVDriverActionServer(Node):

    def __init__(self):
        super().__init__('openmv_driver')
        self._goal_handle = None
        self._active_sub_node = None
        self._action_server = ActionServer(
            self,
            Control,
            'openmv_interface',
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        self.cam = oi.OMV_CAM(name="normal", port="/dev/ttyACM0")
        # self.cam_FLIR = oi.OMV_CAM(name="FLIR", port="/dev/ttyACM1")

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Start execution of the next goal."""
        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Aborting previous goal')
            # Abort the existing goal
            self._goal_handle.abort()
        
        if self._active_sub_node is not None:
            self.get_logger().info('Removing previous sub node from executor')
            self.executor.remove_node(self._active_sub_node)
            self._active_sub_node.destroy_node()
        
        self._goal_handle = goal_handle
        # Start goal execution right away
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')
        gh = goal_handle
        
        self.give_feedback(gh, 0)

        fct_id = goal_handle.request.function  # here is the enum inside..

        exec = self.executor

        

        if fct_id == 0:
            
            self.cam.exe_flo_test(self)
        elif fct_id ==1:
            node = rclpy.create_node(self.get_name() + "image_normal", use_global_arguments=False, start_parameter_services=False)
            publisher = node.create_publisher(Image, 'image', 10)
            width = 320
            heigth = 240

            def timer_cb():
                im = self.cam.exe_jpeg_snapshot(gh)
                rawmsg = Image()
                rawmsg.header.frame_id = "map"
                rawmsg.header.stamp = node.get_clock().now().to_msg()
                rawmsg.width = width
                rawmsg.height = heigth
                rawmsg.encoding = "bgr8"
                rawmsg.data = im.tobytes()
                rawmsg.step = int(len(rawmsg.data)/rawmsg.height)
                publisher.publish(rawmsg)

            timer_period = 0.5  # seconds
            timer = node.create_timer(timer_period, timer_cb)
            exec.add_node(node)
            self._active_sub_node = node

        elif fct_id == 2:
            node = rclpy.create_node(self.get_name() + "qrcode", use_global_arguments=False, start_parameter_services=False)
            publisher = node.create_publisher(QRCode, 'qrcode', 10)
            
            def timer_cb():
                self.give_feedback(goal_handle, 1)
                result = self.cam.exe_qrcode_detection()
                if result is None:
                    result = "No QR code found"
                msg = QRCode()
                msg.qrcode = result
                publisher.publish(msg)

            timer_period = 0.5  # seconds
            timer = node.create_timer(timer_period, timer_cb)
            exec.add_node(node)
            self._active_sub_node = node

            


        #please put in enum-switch that is not ultimativ ugly...
        # also put in ref to self / node, so that the exe_fct. can add publisher and stuff
        # maybe a good thing would be to add a type and name .... hmmmmmmmm


        
        
        goal_handle.succeed()

        self.get_logger().info('Returning result: (empty)')
        result = Control.Result()

        return result

    def give_feedback(self, goal_handle, status_code):
        feedback_msg = Control.Feedback()
        feedback_msg.status = status_code
        goal_handle.publish_feedback(feedback_msg)

def main(args=None):
    rclpy.init(args=args)

    action_server_node = OpenMVDriverActionServer()

    # Only one function at once can be processed by the OpenMV cam
    executor = MultiThreadedExecutor()
    executor.add_node(action_server_node)

    executor.spin()

    # rclpy.spin(minimal_action_server, executor=executor)

    executor.shutdown()
    action_server_node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
