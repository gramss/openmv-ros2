import image, network, math, rpc, sensor, struct, tf

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)


img = sensor.snapshot()
triggered = 25
s = bytearray(str(triggered))
s = bytearray(triggered.to_bytes(1, "big"))
print(s)
compr = img.compress(quality=90).bytearray()
#msg = compr.extend(s)
s[0:0] = compr
print(s)
#msg = compr.extend(triggered.to_bytes(2, "big"))
#print(msg)
#print(img.compress(quality=90).bytearray().extend([triggered.to_bytes(1, "big")]))
