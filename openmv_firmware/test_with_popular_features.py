# Remote Control - As The Remote Device
#
# This script configures your OpenMV Cam as a co-processor that can be remotely controlled by
# another microcontroller or computer such as an Arduino, ESP8266/ESP32, RaspberryPi, and
# even another OpenMV Cam.
#
# This script is designed to pair with "popular_features_as_the_controller_device.py".

import image, network, math, rpc, sensor, struct, tf

TRIGGER_THRESHOLD = 5

BG_UPDATE_FRAMES = 50 # How many frames before blending.
BG_UPDATE_BLEND = 128 # How much to blend by... ([0-256]==[0.0-1.0]).

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

extra_fb = None

interface = rpc.rpc_i2c_slave() # rpc_usb_vcp_slave()

################################################################
# Call Backs
################################################################

# Helper methods used by the call backs below.

def draw_detections(img, dects):
    for d in dects:
        c = d.corners()
        l = len(c)
        for i in range(l): img.draw_line(c[(i+0)%l] + c[(i+1)%l], color = (0, 255, 0))
        img.draw_rectangle(d.rect(), color = (255, 0, 0))

# Remote control works via call back methods that the controller
# device calls via the rpc module on this device. Call backs
# are functions which take a bytes() object as their argument
# and return a bytes() object as their result. The rpc module
# takes care of moving the bytes() objects across the link.
# bytes() may be the micropython int max in size.

# When called returns x, y, w, and h of the largest face within view.
#
# data is unused
def face_detection(data):
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QVGA)
    faces = sensor.snapshot().gamma_corr(contrast=1.5).find_features(image.HaarCascade("frontalface"))
    if not faces: return bytes() # No detections.
    for f in faces: sensor.get_fb().draw_rectangle(f, color = (255, 255, 255))
    out_face = max(faces, key = lambda f: f[2] * f[3])
    return struct.pack("<HHHH", out_face[0], out_face[1], out_face[2], out_face[3])

# When called returns if there's a "person" or "no_person" within view.
#
# data is unused
def person_detection(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    scores = tf.classify("person_detection", sensor.snapshot())[0].output()
    return ['unsure', 'person', 'no_person'][scores.index(max(scores))].encode()

def flo_test(data):
    return  "test".encode()

# When called returns the payload string for the largest qrcode
# within the OpenMV Cam's field-of-view.
#
# data is unused
def qrcode_detection(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((320, 240))
    codes = sensor.snapshot().find_qrcodes()
    if not codes: return bytes() # No detections.
    draw_detections(sensor.get_fb(), codes)
    return max(codes, key = lambda c: c.w() * c.h()).payload().encode()

# When called returns a json list of json qrcode objects for all qrcodes in view.
#
# data is unused
def all_qrcode_detection(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((320, 240))
    codes = sensor.snapshot().find_qrcodes()
    if not codes: return bytes() # No detections.
    draw_detections(sensor.get_fb(), codes)
    return str(codes).encode()

# When called returns the x/y centroid, id number, and rotation of the largest
# AprilTag within the OpenMV Cam's field-of-view.
#
# data is unused
def apriltag_detection(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA)
    tags = sensor.snapshot().find_apriltags()
    if not tags: return bytes() # No detections.
    draw_detections(sensor.get_fb(), tags)
    output_tag = max(tags, key = lambda t: t.w() * t.h())
    return struct.pack("<HHHH", output_tag.cx(), output_tag.cy(), output_tag.id(),
                       int(math.degrees(output_tag.rotation())))

# When called returns a json list of json apriltag objects for all apriltags in view.
#
# data is unused
def all_apriltag_detection(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QQVGA)
    tags = sensor.snapshot().find_apriltags()
    if not tags: return bytes() # No detections.
    draw_detections(sensor.get_fb(), tags)
    return str(tags).encode()

# When called returns the payload string for the largest datamatrix
# within the OpenMV Cam's field-of-view.
#
# data is unused
def datamatrix_detection(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((320, 240))
    codes = sensor.snapshot().find_datamatrices()
    if not codes: return bytes() # No detections.
    draw_detections(sensor.get_fb(), codes)
    return max(codes, key = lambda c: c.w() * c.h()).payload().encode()

# When called returns a json list of json datamatrix objects for all datamatrices in view.
#
# data is unused
def all_datamatrix_detection(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((320, 240))
    codes = sensor.snapshot().find_datamatrices()
    if not codes: return bytes() # No detections.
    draw_detections(sensor.get_fb(), codes)
    return str(codes).encode()

# When called returns the payload string for the largest barcode
# within the OpenMV Cam's field-of-view.
#
# data is unused
def barcode_detection(data):
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((sensor.width(), sensor.height()//8))
    codes = sensor.snapshot().find_barcodes()
    if not codes: return bytes() # No detections.
    return max(codes, key = lambda c: c.w() * c.h()).payload().encode()

# When called returns a json list of json barcode objects for all barcodes in view.
#
# data is unused
def all_barcode_detection(data):
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.VGA)
    sensor.set_windowing((sensor.width(), sensor.height()//8))
    codes = sensor.snapshot().find_barcodes()
    if not codes: return bytes() # No detections.
    return str(codes).encode()

# When called returns the x/y centroid of the largest blob
# within the OpenMV Cam's field-of-view.
#
# data is the 6-byte color tracking threshold tuple of L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX.
def color_detection(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    thresholds = struct.unpack("<bbbbbb", data)
    blobs = sensor.snapshot().find_blobs([thresholds],
                                         pixels_threshold=500,
                                         area_threshold=500,
                                         merge=True,
                                         margin=20)
    if not blobs: return "test".encode() # No detections.
    for b in blobs:
        sensor.get_fb().draw_rectangle(b.rect(), color = (255, 0, 0))
        sensor.get_fb().draw_cross(b.cx(), b.cy(), color = (0, 255, 0))
    out_blob = max(blobs, key = lambda b: b.density())
    return struct.pack("<HH", out_blob.cx(), out_blob.cy())

# When called returns a jpeg compressed image from the OpenMV
# Cam in one RPC call.
#
# data is unused
def jpeg_snapshot(data):
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    return sensor.snapshot().compress(quality=90).bytearray()


##########################################
#### Flo Functions
###########

# This is called repeatedly by interface.stream_writer().
def stream_generator_cb():
    # sensor.snapshot().compress(quality=90).bytearray()
    img = sensor.snapshot() # Take a picture and return the image.
    global extra_fb

    stream_generator_cb.frame_count += 1
    if (stream_generator_cb.frame_count > BG_UPDATE_FRAMES):
        stream_generator_cb.frame_count = 0
        # Blend in new frame. We're doing 256-alpha here because we want to
        # blend the new frame into the backgound. Not the background into the
        # new frame which would be just alpha. Blend replaces each pixel by
        # ((NEW*(alpha))+(OLD*(256-alpha)))/256. So, a low alpha results in
        # low blending of the new image while a high alpha results in high
        # blending of the new image. We need to reverse that for this update.
        img.blend(extra_fb, alpha=(256-BG_UPDATE_BLEND))
        extra_fb.replace(img)

    # Replace the image with the "abs(NEW-OLD)" frame difference.
    img.difference(extra_fb)

    hist = img.get_histogram()
    # This code below works by comparing the 99th percentile value (e.g. the
    # non-outlier max value against the 90th percentile value (e.g. a non-max
    # value. The difference between the two values will grow as the difference
    # image seems more pixels change.
    diff = hist.get_percentile(0.99).l_value() - hist.get_percentile(0.90).l_value()
    triggered = diff > TRIGGER_THRESHOLD

    return img.compress(quality=90).bytearray().extend(triggered.to_bytes(length=1, byteorder='big'))


# Transmits a stream of bytes()'s generated by stream_generator_cb to the master device.
def mov_jpeg_image_stream_cb():
    stream_generator_cb.frame_count = 0
    interface.stream_writer(stream_generator_cb)

# When called sets the pixformat and framesize, and then schedules
# frame streaming to start after the RPC call finishes.
#
# data is a pixformat string and framesize string.
def movement_im_stream(data):
    sensor.set_pixformat(sensor.RGB565) # or sensor.RGB565
    sensor.set_framesize(sensor.QVGA) # or sensor.QQVGA (or others)
    sensor.set_auto_whitebal(False) # Turn off white balance.
    global extra_fb
    extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
    extra_fb.replace(sensor.snapshot()) # set bg-image for diff
    interface.schedule_callback(jpeg_image_stream_cb)
    return "Ready".encode()

def setup_undo_move_settings(data):
    sensor.set_auto_whitebal(True)
    sensor.dealloc_extra_fb()
# Register call backs.

interface.register_callback(face_detection)
#interface.register_callback(person_detection)
#interface.register_callback(qrcode_detection)
#interface.register_callback(all_qrcode_detection)
#interface.register_callback(apriltag_detection)
#interface.register_callback(all_apriltag_detection)
#interface.register_callback(datamatrix_detection)
#interface.register_callback(all_datamatrix_detection)
#interface.register_callback(barcode_detection)
#interface.register_callback(all_barcode_detection)
interface.register_callback(color_detection)
interface.register_callback(jpeg_snapshot)
interface.register_callback(flo_test)
interface.register_callback(movement_im_stream)
interface.register_callback(setup_undo_move_settings)

# Once all call backs have been registered we can start
# processing remote events. interface.loop() does not return.

interface.loop()
