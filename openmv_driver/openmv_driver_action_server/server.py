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
from enum import Enum

from openmv_msgs.action import Control
from openmv_msgs.msg import QRCode, MovementTrigger, SteeringValue

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from openmv_driver_res import openmv_interface as oi

from sensor_msgs.msg import Image, CompressedImage, CameraInfo


CAM_PORT = "/dev/ttyACM0"


class Message(Enum):
    SANITY_CHECK = 0
    IMAGE = 1
    QR_CODE_DETECTION = 2
    MOVEMENT_DETECTION = 3
    LINE_DETECTION = 4


class OpenMVDriverActionServer(Node):

    def __init__(self):
        super().__init__('openmv_driver')
        self._goal_handle = None
        self._active_sub_node = None
        self._active_coroutine = None
        self._action_server = ActionServer(
            self,
            Control,
            'openmv_interface',
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        self.cam = oi.OMV_CAM(name="normal", port=CAM_PORT)
        # self.cam_FLIR = oi.OMV_CAM(name="FLIR", port="/dev/ttyACM1")

        self.get_logger().info("OpenMV action server ready")

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

        if self._active_coroutine is True:
            self._active_coroutine = None
            time.sleep(5) #let stream_event in OpenMV cam go into timeout

        if self._active_sub_node is not None:
            self.get_logger().info('Removing previous sub node from executor')
            self.executor.remove_node(self._active_sub_node)
            self._active_sub_node.destroy_node()
            self._active_sub_node = None
            time.sleep(5)

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

        if fct_id == Message.SANITY_CHECK.value:
            self.cam.exe_sanity_check()

        elif fct_id == Message.IMAGE.value:
            node = rclpy.create_node(self.get_name() + "_image_2Hz", use_global_arguments=False, start_parameter_services=False)
            im_pub = node.create_publisher(Image, 'image', 10)
            width = 320
            height = 240

            def timer_cb():
                im = self.cam.exe_jpeg_snapshot(node.get_logger())
                if im is not None:
                    self.pub_raw_image(node, im, im_pub, width, height)

            timer_period = 0.5  # seconds
            timer = node.create_timer(timer_period, timer_cb)
            exec.add_node(node)
            self._active_sub_node = node

        elif fct_id == Message.QR_CODE_DETECTION.value:
            node = rclpy.create_node(self.get_name() + "qrcode", use_global_arguments=False, start_parameter_services=False)
            im_pub = node.create_publisher(Image, "qrcode", 10)
            width = 320
            height = 240

            self.cam.exe_setup_qr_mode()

            def timer_cb():
                im = self.cam.exe_qrcode_detection()
                if im is not None:
                    self.pub_raw_image(node, im, im_pub, width, height)

            # Periodically execute the timer callback.
            timer_period = 0.5  # Seconds
            node.create_timer(timer_period, timer_cb)

            exec.add_node(node)
            self._active_sub_node = node

        elif fct_id == Message.MOVEMENT_DETECTION.value:
            node = rclpy.create_node(self.get_name() + "_movement", use_global_arguments=False, start_parameter_services=False)
            im_pub = node.create_publisher(Image, "move_diff_image", 10) 
            trigger_pub = node.create_publisher(MovementTrigger, "movement_trigger", 10)

            width = 320
            height = 240

            self._active_coroutine = True

            def stream_cb(data):
                #node.get_logger().info("stream_cb says hi")
                #trigger = data[-1:] #tailing byte is trigger byte
                #im_bytes = data[0:-1] #rest is image as jpg
                print(type(data))
                im = self.cam.helper_bytes_to_image_raw(bytes(data)) #im_bytes)
                self.pub_raw_image(node, im, im_pub, width, height)

                #msg = MovementTrigger()
                #msg.trigger = trigger
                #trigger_pub.publish(msg)

            async def stream_movement():
                #while rclpy.ok() and self._active_coroutine is True:
                if self.cam.exe_setup_im_stream():
                    res = self.cam.exe_im_stream()
                    #time.sleep(2) #wait til cam gets ready..!
                    node.get_logger().info("Active_coroutine?= " + str(self._active_coroutine))
                    if res is not None:
                        self.cam.omv_interface.stream_reader(stream_cb, 
                            queue_depth=8, 
                            read_timeout_ms=2500,
                            keep_looping=self._active_coroutine)
                        node.get_logger().info("Finished openmv stream_loop")
                    else:
                        node.get_logger().error("Could not start stream")
                        return False
                else:
                    node.get_logger().warn("Failed to setup cam for stream_movement")
                    return False

            exec.add_node(node)
            self._active_sub_node = node
            self._active_coroutine = True
            exec.create_task(stream_movement)

        elif fct_id == Message.LINE_DETECTION.value:
            node = rclpy.create_node(self.get_name() + "line_detection", use_global_arguments=False, start_parameter_services=False)
            publisher = node.create_publisher(SteeringValue, "line_detection_front", 10)
            #publisher = node.create_publisher(SteeringValue, "line_detection_rear", 10)

            width = 320
            height = 240

            self.cam.exe_setup_line_detection()

            def timer_cb():
                steering_value = self.cam.exe_line_detection()
                msg = SteeringValue()
                msg.data = int(steering_value)
                publisher.publish(msg)

            # Periodically execute the timer callback.
            timer_period = 0.1  # Seconds
            node.create_timer(timer_period, timer_cb)

            exec.add_node(node)
            self._active_sub_node = node

        # TODO: please put in enum-switch that is not ultimativ ugly...
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

    def pub_raw_image(self, node, im, publisher, width, height):
        rawmsg = Image()
        rawmsg.header.frame_id = "map" #Needs to be changed.. But to what?! :D
        rawmsg.header.stamp = node.get_clock().now().to_msg()
        rawmsg.width = width
        rawmsg.height = height
        rawmsg.encoding = "bgr8"
        rawmsg.data = im.tobytes()
        rawmsg.step = int(len(rawmsg.data)/rawmsg.height)
        publisher.publish(rawmsg)


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
