# Copyright 2021 Florian Gramß
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


from openmv_driver_res.rpc import rpc

import json, serial, serial.tools.list_ports, struct, sys, datetime
import PIL
from PIL import Image as PILImage

class OMV_CAM:
    """ OpenMV Hardware Interface """
    
    def __init__(self, name, port):
        """
        Create an OpenMV Hardware Interface. Currently only supporting USB_VCP mode.

        :param name: Give your camera a name for multiple cameras.
        :param port: Identify /etc/dev/<port> to connect to your camera.
        """
        self.omv_name = name
        self.omv_interface = rpc.rpc_usb_vcp_master(port)

    ##############################################################
    # Call Back Handlers - from openmv.io - see rpc folder for license
    ##############################################################

    def exe_face_detection(self):
        result = self.omv_interface.call("face_detection")
        if result is not None and len(result):
            print("Largest Face Detected [x=%d, y=%d, w=%d, h=%d]" % struct.unpack("<HHHH", result))

    def exe_person_detection(self):
        result = self.omv_interface.call("person_detection")
        if result is not None:
            print(result.tobytes())

    def exe_flo_test(self):
        result = self.omv_interface.call("flo_test")
        if result is not None:
            print(result.tobytes())
        else:
            print("nope!")

    def exe_qrcode_detection(self):
        result = self.omv_interface.call("qrcode_detection")
        if result is not None and len(result):
            return(result.tobytes())

    def exe_all_qrcode_detection(self):
        result = self.omv_interface.call("all_qrcode_detection")
        if result is not None and len(result):
            print("QR Codes Detected:")
            for obj in json.loads(result.tobytes()):
                print(obj)

    def exe_apriltag_detection(self):
        result = self.omv_interface.call("apriltag_detection")
        if result is not None and len(result):
            print("Largest Tag Detected [cx=%d, cy=%d, id=%d, rot=%d]" % struct.unpack("<HHHH",result))

    def exe_all_apriltag_detection(self):
        result = self.omv_interface.call("all_apriltag_detection")
        if result is not None and len(result):
            print("Tags Detected:")
            for obj in json.loads(result.tobytes()):
                print(obj)

    def exe_datamatrix_detection(self):
        result = self.omv_interface.call("datamatrix_detection")
        if result is not None and len(result):
            print(result.tobytes())

    def exe_all_datamatrix_detection(self):
        result = self.omv_interface.call("all_datamatrix_detection")
        if result is not None and len(result):
            print("Data Matrices Detected:")
            for obj in json.loads(result.tobytes()):
                print(obj)

    def exe_barcode_detection(self):
        result = self.omv_interface.call("barcode_detection")
        if result is not None and len(result):
            print(result.tobytes())

    def exe_all_barcode_detection(self):
        result = self.omv_interface.call("all_barcode_detection")
        if result is not None and len(result):
            print("Bar Codes Detected:")
            for obj in json.loads(result.tobytes()):
                print(obj)

    def exe_color_detection(self):
        thresholds = (30, 100, 15, 127, 15, 127) # generic red thresholds
        # thresholds = (30, 100, -64, -8, -32, 32) # generic green thresholds
        # thresholds = (0, 30, 0, 64, -128, 0) # generic blue thresholds
        result = self.omv_interface.call("color_detection", struct.pack("<bbbbbb", *thresholds))
        if result is not None and len(result):
            print("Largest Color Detected [cx=%d, cy=%d]" % struct.unpack("<HH", result))

    def exe_jpeg_snapshot(self, goal_handle):
        #self.get_logger().info("Now Calling")
        result = self.omv_interface.call("jpeg_snapshot")
        #self.get_logger().info("Calling done!")
        if result is not None:
            #from https://github.com/TRI-jaguar4x4/jpeg_to_raw/blob/master/jpeg_to_raw/jpeg_to_raw.py
            try:
                im = PILImage.frombuffer("RGB",
                                        (320,240),
                                        result.tobytes(), "jpeg", "RGB", "")
                b, g, r = im.split()
                im = PILImage.merge("RGB", (r, g, b))
                return im
            except Exception as e:
                print ("Exception loading PILImage.frombuffer: ", e)
                return None
            
            
            #name = "snapshot-%s.jpg" % datetime.now().strftime("%d.%m.%Y-%H.%M.%S")
            #print("Writing jpeg %s..." % name)
            #with open(name, "wb") as snap:
            #    snap.write(result)