import image, network, math, rpc, sensor, struct, tf, usys, os

import time
from pyb import LED
from pyb import UART

uart = UART(3, 19200)
os.dupterm(uart)

class ROS2_Interface:

    def __init__(self):
        # Things needed for movement detection
        self.TRIGGER_THRESHOLD = 5
        self.BG_UPDATE_FRAMES = 50 # How many frames before blending.
        self.BG_UPDATE_BLEND = 128 # How much to blend by... ([0-256]==[0.0-1.0]).
        self.frame_count = 0

        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.skip_frames(time = 2000)

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
        return  "42".encode()

    def setup_cam_rgb_QVGA(self, data):
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_auto_whitebal(True)

    def jpeg_snapshot(self, data):
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        return sensor.snapshot().compress(quality=90).bytearray()

    ##########################################
    #### Flo Functions
    ###########

    # This is called repeatedly by interface.stream_writer().
    def stream_generator_cb(self):
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
        self.interface.stream_writer(self.stream_generator_cb)
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
        #self.interface.register_callback(qrcode_detection)
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
        self.interface.register_callback(self.setup_cam_rgb_QVGA)
        print("done registering callbacks")


blue_led  = LED(3)
blue_led.on()


#time.sleep_ms(4000)
print("hello Terminal")
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



#print("no")
