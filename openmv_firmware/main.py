import image, network, math, rpc, sensor, struct, tf, usys, os

import time
from pyb import LED
from pyb import UART


UART_BAUDRATE = 19200
uart = UART(3, UART_BAUDRATE)
os.dupterm(uart)

class ROS2_Interface:

    def __init__(self):
        # Things needed for movement detection
        self.TRIGGER_THRESHOLD = 5
        self.BG_UPDATE_FRAMES = 5  # How many frames before blending.
        self.BG_UPDATE_BLEND = 16  # How much to blend by ([0-256]==[0.0-1.0]).
        self.frame_count = 0

        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time=2000)

        self.extra_fb = None

        # self.interface = rpc.rpc_uart_slave(baudrate=115200) #rpc_usb_vcp_slave()
        self.interface = rpc.rpc_usb_vcp_slave()

    # Helper methods used by the call backs below.
    def draw_detections(self, img, dects):
        for d in dects:
            c = d.corners()
            l = len(c)
            for i in range(l): img.draw_line(c[(i+0)%l] + c[(i+1)%l], color = (0, 255, 0))
            img.draw_rectangle(d.rect(), color = (255, 0, 0))

    def sanity_check(self, data):
        print("Sanity!")
        return "42".encode()

    def setup_cam_rgb_QVGA(self, data):
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_auto_whitebal(True)
        sensor.skip_frames(time=2000)

    def setup_line_detection(self, data):
        roi = (0, 220, 240, 20)  # x, y, w, h
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_windowing(roi)
        sensor.set_auto_gain(False)  # Must be turned off for color tracking.
        sensor.set_auto_whitebal(False)  # Must be turned off for color tracking.
        sensor.skip_frames(time=2000)
        return "line detection mode".encode()

    def jpeg_snapshot(self, data):
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.VGA)
        return sensor.snapshot().compress(quality=90).bytearray()

    def setup_qr_mode(self, data):
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.set_framesize(sensor.VGA)
        # Turn auto gain off to prevent image washout.
        sensor.set_auto_gain(False)
        return "qr mode".encode()

    def qrcode_detection(self, data):
        print("executing qrcode_detection")
        img = sensor.snapshot()
        # Apply lens correction.
        img.lens_corr(1.2)

        codes = img.find_qrcodes()
        result = list()
        if codes:
            for code in codes:
                print("\t", len(codes)," QR code(s) detected")
                print("\t", code.payload())

                result.append(code.payload())

                offset = 5
                (bb_x, bb_y, bb_w, bb_h) = code.rect()
                # Draw the bounding box.
                img.draw_rectangle(code.rect(), color=(255, 0, 0))
                # Draw the payload string (under the bounding box).
                img.draw_string(bb_x, bb_y + bb_h + offset,
                                code.payload(),
                                color=(0, 0, 0),
                                scale=1.5,
                                mono_space=False)

        # Return the camera frame (even) if no QR code was detected.
        return result, img.compress(quality=90).bytearray()

    def get_steering_values(self, x_pos, frame_center):
        return frame_center - x_pos

    def detect_blue_line(self, data):
        print("executing blue line detection")
        QVGA_H, QVGA_W = (320, 240)
        FRAME_CENTER = QVGA_W // 2

        # Color tracking thresholds (L min, L max, A min, A max, B min, B max).
        target_blue_thresh = [(18, 60, -6, 50, -80, -35)]

        # TODO: Apply lens correction value for a more wide-angle (fisheye) view.
        img = sensor.snapshot()

        # Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
        # returned."merge=True" merges all overlapping blobs in the image.
        detected_blobs = img.find_blobs(target_blue_thresh, pixels_threshold=64, area_threshold=64, merge=True)
 
        if len(detected_blobs) > 0:
            for blob in detected_blobs:
                # Turn on for debugging purposes.
                #img.draw_rectangle(blob.rect())
                #img.draw_cross(blob.cx(), blob.cy())

                steering_val = self.get_steering_values(blob.cx(), FRAME_CENTER)
                print(steering_val)
                return str(steering_val).encode()
        else:
            # Return a high negative value if no blob was detected.
            return "-1337".encode()

    def stream_generator_cb(self):
        img = sensor.snapshot() # Take a picture and return the image.
        # Rotation.
        img = img.replace(vflip=True, hmirror=False, transpose=True)
        # FIXME: If compression call is removed: Exception loading PILImage.frombuffer (rgb): cannot decode image data
        return img.compress(quality=90).bytearray()
        #return img.bytearray()

    # Transmits a stream of bytes()'s generated by stream_generator_cb to the master device.
    def image_stream_cb(self):
        self.frame_count = 0
        self.interface.stream_writer(self.stream_generator_cb)
        self.setup_undo_move_settings()

    # When called sets the pixformat and framesize, and then schedules
    # frame streaming to start after the RPC call finishes.
    #
    # data is a pixformat string and framesize string.
    def image_stream(self, data):
        self.interface.schedule_callback(self.image_stream_cb)
        return "Ready".encode()

    def setup_image_stream_settings(self, data):
        # Color & resolution.
        pixformat, framesize = bytes(data).decode().split(",")

        print("setup_image_stream_settings")
        print("data=", pixformat, framesize)

        if pixformat == "RGB":
            sensor.set_pixformat(sensor.RGB565)
        elif pixformat == "GRAY":
            sensor.set_pixformat(sensor.GRAYSCALE)

        if framesize == "QVGA":
            sensor.set_framesize(sensor.QVGA)
        elif framesize == "VGA":
            sensor.set_framesize(sensor.VGA)

        sensor.set_auto_whitebal(False)
        sensor.skip_frames(time=2000)
        print("camera set up with color=" + pixformat + ", framesize=" + framesize)

        """ # probably dont need this
        self.extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
        print("About to save background image...")
        sensor.skip_frames(time = 2000) # Give the user time to get ready.
        self.extra_fb.replace(sensor.snapshot()) # set bg-image for diff
        print("done setup im")
        """
        return "worked".encode()


    ##########################################
    #### Flo Functions
    ###########

    # This is called repeatedly by interface.stream_writer().
    def movement_stream_generator_cb(self):
        # sensor.snapshot().compress(quality=90).bytearray()
        img = sensor.snapshot() # Take a picture and return the image.
        #print("Fram_count: ")
        #print(self.frame_count)
        self.frame_count += 1
        if (self.frame_count > self.BG_UPDATE_FRAMES):
            #print("Fram_count to zero")
            self.frame_count = 0
            # Blend in new frame. We're doing 256-alpha here because we want to
            # blend the new frame into the backgound. Not the background into the
            # new frame which would be just alpha. Blend replaces each pixel by
            # ((NEW*(alpha))+(OLD*(256-alpha)))/256. So, a low alpha results in
            # low blending of the new image while a high alpha results in high
            # blending of the new image. We need to reverse that for this update.
            img.blend(self.extra_fb, alpha=(256-self.BG_UPDATE_BLEND))
            self.extra_fb.replace(img)

        # Replace the image with the "abs(NEW-OLD)" frame difference.
        img.difference(self.extra_fb)

        hist = img.get_histogram()
        # This code below works by comparing the 99th percentile value (e.g. the
        # non-outlier max value against the 90th percentile value (e.g. a non-max
        # value. The difference between the two values will grow as the difference
        # image seems more pixels change.
        diff = hist.get_percentile(0.99).l_value() - hist.get_percentile(0.90).l_value()
        triggered = diff > self.TRIGGER_THRESHOLD

        # Blob detection.
        # Generic threshold including all colors (and )
        thresholds = [(5, 100, -128, 127, -128, 127)]
        detected_blobs = img.find_blobs(thresholds, pixels_threshold=50, area_threshold=50, merge=True)
        if detected_blobs:
            for blob in detected_blobs:
                # These values depend on the blob not being circular - otherwise they will be shaky.
                #if blob.elongation() > 0.5:
                #    img.draw_edges(blob.min_corners(), color=(255,0,0))
                #    img.draw_line(blob.major_axis_line(), color=(0,255,0))
                #    img.draw_line(blob.minor_axis_line(), color=(0,0,255))
                # These values are stable all the time.
                img.draw_rectangle(blob.rect())
                #img.draw_cross(blob.cx(), blob.cy())
                # Note - the blob rotation is unique to 0-180 only.
                #img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)


        return img.compress(quality=90).bytearray()

        #prepare bytearray() to send as stream - bytearray() is actually just a ref to heap, so extending means copying which is "bad". See Issue
        compr = img.compress(quality=90).bytearray()
        t = bytearray({triggered})
        t[0:0] = compr
        #try it with bytes() and not bytearray()
        # compr = img.compress(quality=50).read()
        # t = compr.bytes().extend({triggered})
        # Do some magic while .extend() does not work..
        # see: https://github.com/openmv/openmv/issues/1443

        return t

    # Transmits a stream of bytes()'s generated by stream_generator_cb to the master device.
    def mov_jpeg_image_stream_cb(self):
        self.frame_count = 0
        self.interface.stream_writer(self.movement_stream_generator_cb)
        self.setup_undo_move_settings()

    # When called sets the pixformat and framesize, and then schedules
    # frame streaming to start after the RPC call finishes.
    #
    # data is a pixformat string and framesize string.
    def movement_im_stream(self, data):
        self.interface.schedule_callback(self.mov_jpeg_image_stream_cb)
        return "Ready".encode()

    def setup_move_settings(self, data):
        sensor.set_pixformat(sensor.RGB565) # or sensor.RGB565
        sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
        sensor.set_auto_whitebal(False) # Turn off white balance.

        self.extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
        print("About to save background image...")
        sensor.skip_frames(time = 2000) # Give the user time to get ready.
        self.extra_fb.replace(sensor.snapshot()) # set bg-image for diff
        print("done setup im")
        return "worked".encode()

    def setup_undo_move_settings(self):
        sensor.set_auto_whitebal(True)
        sensor.dealloc_extra_fb()
        print("dealloc done")

    def register_callbacks(self):
        #self.interface.register_callback(face_detection)
        #self.interface.register_callback(person_detection)
        self.interface.register_callback(self.setup_qr_mode)
        self.interface.register_callback(self.qrcode_detection)
        #self.interface.register_callback(all_qrcode_detection)
        #self.interface.register_callback(apriltag_detection)
        #self.interface.register_callback(all_apriltag_detection)
        #self.interface.register_callback(datamatrix_detection)
        #self.interface.register_callback(all_datamatrix_detection)
        #self.interface.register_callback(barcode_detection)
        #self.interface.register_callback(all_barcode_detection)
        #self.interface.register_callback(color_detection)
        self.interface.register_callback(self.jpeg_snapshot)
        self.interface.register_callback(self.sanity_check)
        self.interface.register_callback(self.setup_move_settings)
        self.interface.register_callback(self.movement_im_stream)
        self.interface.register_callback(self.image_stream)
        self.interface.register_callback(self.setup_image_stream_settings)
        self.interface.register_callback(self.setup_cam_rgb_QVGA)
        self.interface.register_callback(self.setup_line_detection)
        self.interface.register_callback(self.detect_blue_line)
        print("done registering callbacks")


# Turn on blue during operation.
blue_led = LED(3)
blue_led.on()

#time.sleep_ms(4000)
print("hello terminal")
#blue_led.off()

A = ROS2_Interface()
A.register_callbacks()
#print("hi")
#a = rpc.rpc()
#b = a._hash("sanity_check", len("sanity_check"))
#print(A.interface.__dict.get(b)("www"))


# time.sleep_ms(1000)
# blue_led.on()
# time.sleep_ms(1000)
# SystemExit()
# time.sleep_ms(1000)

A.interface.loop()
