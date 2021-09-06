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

import serial

from openmv_msgs.action import Control
from openmv_msgs.msg import QRCode, MovementTrigger

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
        self._active_coroutine = None
        self._future_coroutine = None
        self._action_server = ActionServer(
            self,
            Control,
            'openmv_interface',
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        self.cam = None #oi.OMV_CAM(name="normal", port="/dev/ttyACM0")
        self.cam_FLIR = None #oi.OMV_CAM(name="FLIR", port="/dev/ttyACM1")

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
            #self.executor.spin_until_future_complete(self._future_coroutine)
            self._future_coroutine = None
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

        

        if fct_id == 0:
            result = self.cam.exe_sanity_check()
            if result is not None:
                self.get_logger().info("Sanity call - success: " + str(result.tobytes()))
                goal_handle.succeed()
                return self.generate_result()
            else:
                #self.give_feedback(gh,3)
                goal_handle.abort()
                # Is a return needed..?
                return self.generate_result()

        elif fct_id ==1:
            # This needs to be rewritten to a streamer like #3 or another function added as a streamer with normal video stream
            node = rclpy.create_node(self.get_name() + "_image_2Hz", use_global_arguments=False, start_parameter_services=False)
            im_pub = node.create_publisher(Image, 'image', 10)
            width = 320
            height = 240
            miss_counter = 0

            def timer_cb():
                im = self.cam.exe_jpeg_snapshot(node.get_logger())
                if im is not False:
                    self.pub_raw_image(node, im, im_pub, width, height)
                else:
                    self.get_logger().warn("Missed picture from cam..")
                    miss_counter =+ 1
                    if miss_counter > 50:
                        self.get_logger().error("Too much misses from cam. Dropping 2Hz stream")
                        


            timer_period = 0.5  # seconds
            timer = node.create_timer(timer_period, timer_cb)
            exec.add_node(node)
            self._active_sub_node = node
            goal_handle.succeed()

        elif fct_id == 2:
            #never tested
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

        elif fct_id == 3:
            node = rclpy.create_node(self.get_name() + "_movement", use_global_arguments=False, start_parameter_services=False)
            im_pub = node.create_publisher(Image, "move_diff_image", 10) 
            #trigger_pub = node.create_publisher(MovementTrigger, "movement_trigger", 10)
            
            width = 320
            height = 240
            miss_counter = 0

            self._active_coroutine = True

            def stream_cb(data):
                #node.get_logger().info("stream_cb says hi")
                #trigger = data[-1:] #tailing byte is trigger byte
                #im_byte_arr = data[0:-1] #rest is image as jpg
                if data is None:
                    miss_counter =+ 1
                    if miss_counter > 50:
                        goal_handle.abort()
                    return

                #print(type(data))
                im = self.cam.helper_bytes_to_image_raw(bytes(data)) #im_byte_arr)
                #im = self.cam.helper_bytes_to_image_raw(bytes(im_byte_arr)) #im_byte_arr)
                self.pub_raw_image(node, im, im_pub, width, height)

                # # Does not work that way as we would cancel the stream with a different request.
                # trigger = self.cam.exe_mov_triggered()
                # print(type(trigger))
                # msg = MovementTrigger()
                # msg.trigger = bool(trigger)
                # trigger_pub.publish(msg)



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
            self._future_coroutine = exec.create_task(stream_movement)
            goal_handle.succeed()
            #exec.spin_until_future_complete(self._future_coroutine)
            

        if fct_id == 4: #reset cam
            if self.cam is not None:
                del self.cam.omv_interface #.reset()
                self.cam = None
            if self.cam_FLIR is not None:
                self.cam_FLIR.omv_interface.reset()
                self.cam_FLIR = None
            goal_handle.succeed()

        if fct_id == 5: #start cam 1
            try:
                self.cam = oi.OMV_CAM(name="normal", port="/dev/ttyACM0")
                goal_handle.succeed()   
            except serial.SerialException as e:
                self.get_logger().error(e.strerror)
                goal_handle.abort()

        if fct_id == 6: #start FLIR Cam 2
            try:
                self.cam_FLIR = oi.OMV_CAM(name="FLIR", port="/dev/ttyACM1")
                goal_handle.succeed()
            except serial.SerialException as e:
                self.get_logger().error(e.strerror)
                goal_handle.abort()



        #please put in enum-switch that is not ultimativ ugly...
        # also put in ref to self / node, so that the exe_fct. can add publisher and stuff
        # maybe a good thing would be to add a type and name .... hmmmmmmmm

        # while (rclpy.ok()):
        #     self.give_feedback(gh, 1) #everything still fine
        #     exec.spin_once()
        #     if self._future_coroutine is not None:
        #         exec.spin_once_until_future_complete(self._future_coroutine)
            
        
        
        #goal_handle.succeed()

        return self.generate_result()

    def generate_result(self):
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
