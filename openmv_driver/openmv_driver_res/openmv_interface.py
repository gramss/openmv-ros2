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


from typing import re
from builtins import type
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

    def exe_sanity_check(self):
        return self.omv_interface.call("sanity_check")
        # if result is not None:
        #     print(result.tobytes())
        # else:
        #     print("nope!")

    def exe_setup_qr_mode(self):
        return self.omv_interface.call("setup_qr_mode")

    def exe_qrcode_detection(self):
        result = self.omv_interface.call("qrcode_detection")
        if result is not None:
            return result[0], self.helper_bytes_to_image_raw(result[1].tobytes())

    def exe_setup_line_detection(self):
        res = self.omv_interface.call("setup_line_detection")
        if res is not None:
            return res

    def exe_line_detection(self):
        res = self.omv_interface.call("detect_blue_line")
        if res is not None:
            return res

    # def exe_all_qrcode_detection(self):
    #     result = self.omv_interface.call("all_qrcode_detection")
    #     if result is not None and len(result):
    #         print("QR Codes Detected:")
    #         for obj in json.loads(result.tobytes()):
    #             print(obj)

    # def exe_apriltag_detection(self):
    #     result = self.omv_interface.call("apriltag_detection")
    #     if result is not None and len(result):
    #         print("Largest Tag Detected [cx=%d, cy=%d, id=%d, rot=%d]" % struct.unpack("<HHHH",result))

    # def exe_all_apriltag_detection(self):
    #     result = self.omv_interface.call("all_apriltag_detection")
    #     if result is not None and len(result):
    #         print("Tags Detected:")
    #         for obj in json.loads(result.tobytes()):
    #             print(obj)

    # def exe_datamatrix_detection(self):
    #     result = self.omv_interface.call("datamatrix_detection")
    #     if result is not None and len(result):
    #         print(result.tobytes())

    # def exe_all_datamatrix_detection(self):
    #     result = self.omv_interface.call("all_datamatrix_detection")
    #     if result is not None and len(result):
    #         print("Data Matrices Detected:")
    #         for obj in json.loads(result.tobytes()):
    #             print(obj)

    # def exe_barcode_detection(self):
    #     result = self.omv_interface.call("barcode_detection")
    #     if result is not None and len(result):
    #         print(result.tobytes())

    # def exe_all_barcode_detection(self):
    #     result = self.omv_interface.call("all_barcode_detection")
    #     if result is not None and len(result):
    #         print("Bar Codes Detected:")
    #         for obj in json.loads(result.tobytes()):
    #             print(obj)

    def exe_color_detection(self):
        thresholds = (30, 100, 15, 127, 15, 127) # generic red thresholds
        # thresholds = (30, 100, -64, -8, -32, 32) # generic green thresholds
        # thresholds = (0, 30, 0, 64, -128, 0) # generic blue thresholds
        result = self.omv_interface.call("color_detection", struct.pack("<bbbbbb", *thresholds))
        if result is not None and len(result):
            print("Largest Color Detected [cx=%d, cy=%d]" % struct.unpack("<HH", result))

    def exe_jpeg_snapshot(self, logger):
        #logger.info("Now Calling")
        result = self.omv_interface.call("jpeg_snapshot")
        #logger.info("Calling done!")
        if result is not None:
            logger.info("sending result to helper")
            #print(result)
            return self.helper_bytes_to_image_raw(result.tobytes())
        else:
            return False

    ### movement stream functions

    def exe_setup_movement_stream(self):
        result = self.omv_interface.call("setup_move_settings", recv_timeout=4000)
        if result is not None:
            return True
        return False

    def exe_setup_image_stream(self, data: str):
        result = self.omv_interface.call("setup_image_stream_settings", data, recv_timeout=4000)
        if result is not None:
            return True
        return False

    ### image stream functions

    def exe_movement_im_stream(self):
        result = self.omv_interface.call("movement_im_stream")
        if result is not None:
            return True
        return False

    def exe_image_stream(self):
        result = self.omv_interface.call("image_stream")
        if result is not None:
            return True
        return False

    def helper_bytes_to_image_raw(self, im_bytes, width=320, height=240):
        #from https://github.com/TRI-jaguar4x4/jpeg_to_raw/blob/master/jpeg_to_raw/jpeg_to_raw.py
        try:
            im = PILImage.frombuffer("RGB",
                                    (width, height),
                                    im_bytes, "jpeg", "RGB", "")
            b, g, r = im.split()
            return PILImage.merge("RGB", (r, g, b))
        except Exception as e:
                print ("Exception loading PILImage.frombuffer (rgb): ", e)
                return None

    def helper_bytes_to_image_raw_grayscale(self, im_bytes, width=320, height=240):
        #from https://github.com/TRI-jaguar4x4/jpeg_to_raw/blob/master/jpeg_to_raw/jpeg_to_raw.py
        try:
            im = PILImage.frombuffer("L",
                                    (width, height),
                                    im_bytes, "jpeg", "L", "")
            g = im.split()
            return PILImage.merge("L", g)
        except Exception as e:
                print ("Exception loading PILImage.frombuffer (grayscale): ", e)
                return None
