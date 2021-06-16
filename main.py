import pyb, sensor, image, math, time

enable_lens_corr = False # turn on for straighter lines...
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE) # grayscale is faster
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()

# All lines also have `x1()`, `y1()`, `x2()`, and `y2()` methods to get their end-points
# and a `line()` method to get all the above as one 4 value tuple for `draw_line()`.
def degrees(radians):
    return (180 * radians) / math.pi

f_x = (2.8 / 3.984) * 160 # find_apriltags defaults to this if not set
f_y = (2.8 / 2.952) * 120 # find_apriltags defaults to this if not set
c_x = 160 * 0.5 # find_apriltags defaults to this if not set (the image.w * 0.5)
c_y = 120 * 0.5 # find_apriltags defaults to this if not set (the image.h * 0.5)

uart = pyb.UART(3,9600,timeout_char=1000)
uart.init(9600,bits=8,parity = None, stop=1, timeout_char=1000)

while(True):
    clock.tick()
    img = sensor.snapshot()
    if enable_lens_corr: img.lens_corr(1.8) # for 2.8mm lens...

    for l in img.find_line_segments(merge_distance = 200, max_theta_diff = 200):
        img.draw_line(l.line(), color = (255, 255, 0))
        print_args = (l.x1(), l.y1(), l.x2(), l.y2(), l.theta())
        uart.write(("/line/run %f %f %f %f %f\n" % print_args).encode())
        print(("/line/run %f %f %f %f %f\n" % print_args).encode())
        time.sleep_ms(750)

    for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y): # defaults to TAG36H11
        img.draw_rectangle(tag.rect(), color = (255, 255, 0))
        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
        print_args = (tag.x_translation(), tag.y_translation(), tag.z_translation(), \
            degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
        # Translation units are unknown. Rotation units are in degrees.
        uart.write(("/tag/run %f %f %f %f %f %f \n" % print_args).encode())
        print(("/tag/run %f %f %f %f %f %f \n" % print_args).encode())
        time.sleep_ms(750)

