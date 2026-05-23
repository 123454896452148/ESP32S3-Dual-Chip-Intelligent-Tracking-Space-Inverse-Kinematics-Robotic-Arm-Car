import sensor, time, image, lcd, math, os, tf, gc
from machine import UART, Pin, Timer
from uservo import USERVO
from pid import PID

uart = UART(1, 115200, tx=43, rx=44)
allow_grab = False
allow_release = False

objects = list()
last_objects = objects
global key_state, key_event
key_state = 0
key_event = 0
key = Pin(1, Pin.IN, Pin.PULL_UP)

global THRESHOLD
THRESHOLD = [(43, 85, -31, 55, 27, 84)]
yellow_threshold = [(59, 100, -16, 31, 15, 89), (43, 85, -31, 55, 27, 84)]
red_threshold = [(32, 71, -45, 83, 17, 62), (38, 65, 54, 109, 34, 107)]
blue_threshold = [(22, 78, -54, 23, -67, -24), (10, 46, -26, 63, -82, -7)]

S1 = USERVO(pin=21)  # 左右
S2 = USERVO(pin=47)  # 前后
S3 = USERVO(pin=2)   # 上下
S4 = USERVO(pin=45)  # 爪子

pan_pid = PID(p=0.04, i=0, imax=90)
tilt_pid = PID(p=0.04, i=0, imax=90)
dis_pid = PID(p=0.01, i=0, imax=90)

global pan_angle, tilt_angle, dis_angle, count
global ARM_LENGHT, angle_start, arm_angle_catch, cor_dst

S1_offset = 0
S2_offset = 0
S3_offset = -5
DIS_offset = 30
ARM_LENGHT = (73+40, 95, 145)

CLAW_RELEASE = 30
CLAW_CATCH = 150
pan_angle = 90
tilt_angle = 135
dis_angle = 90
count = 0
arm_angle_catch = (0, 0, 0)
cor_dst = (0, 0, 0)
grab_completed = False

def calculate_object_xy(pan_angle, tilt_angle, real_w):
    ser_h = 50
    LX = 0
    LY = 0
    Len = 0
    degrex = math.radians(pan_angle)
    degree = math.radians(180 - tilt_angle)
    s1 = ser_h / math.sin(degree)
    Len = (ARM_LENGHT[0] + ARM_LENGHT[1] + s1 - real_w) * math.tan(degree)
    Len = int(Len)
    LX = int(Len * math.cos(degrex))
    LY = int(Len * math.sin(degrex))
    return LX, LY

def servo_angle(servo_n, s_angle, soffset=0):
    s_angle = soffset + s_angle
    if s_angle <= 0:
        s_angle = 0
    if s_angle >= 180:
        s_angle = 180
    servo_n.angle(s_angle)

def servo_move(servo_n, start_angle, end_angle, soffset):
    if start_angle >= end_angle:
        while start_angle >= end_angle:
            servo_angle(servo_n, start_angle, soffset)
            time.sleep_ms(30)
            start_angle -= 2
        return
    if start_angle <= end_angle:
        while start_angle <= end_angle:
            servo_angle(servo_n, start_angle, soffset)
            time.sleep_ms(30)
            start_angle += 2
        return

def machineArm_move_to_start(arm_angle):
    global pan_angle, dis_angle, tilt_angle
    pan_angle = 90
    dis_angle = 90
    tilt_angle = 135
    time.sleep_ms(1000)
    servo_move(S2, arm_angle[1], dis_angle, S2_offset)
    servo_move(S1, arm_angle[0], pan_angle, S1_offset)
    servo_move(S3, arm_angle[2], tilt_angle, S3_offset)
    time.sleep_ms(1000)

def machineArm_move_to_coordinate(angle_start, arm_angle):
    time.sleep_ms(1000)
    servo_move(S3, angle_start[2], arm_angle[2], S3_offset)
    time.sleep_ms(1000)
    servo_move(S2, angle_start[1], arm_angle[1], S2_offset)
    time.sleep_ms(1000)
    servo_move(S1, angle_start[0], arm_angle[0], S1_offset)
    time.sleep_ms(1000)

def machineArm_move_to_dest(angle_start, arm_angle):
    time.sleep_ms(1000)
    servo_move(S2, angle_start[1], 90, S2_offset)
    servo_move(S1, angle_start[0], arm_angle[0], S1_offset)
    servo_move(S3, angle_start[2], arm_angle[2], S3_offset)
    servo_move(S2, 90, arm_angle[1], S2_offset)
    time.sleep_ms(1000)

def find_max_object(objects):
    max_size = 0
    max_object = None
    for object in objects:
        if object[2] * object[3] > max_size:
            max_object = object
            max_size = object[2] * object[3]
    return max_object

def color_detect(img):
    global pan_angle, tilt_angle, dis_angle, count, angle_start, arm_angle_catch, cor_dst, grab_completed
    if grab_completed:
        return
    angle_start = (0, 0, 0, 0)
    ball_s = 0
    color_num = 0
    blobs0 = img.find_blobs(red_threshold, pixels_threshold=100, merge=True)
    if blobs0:
        max_blob = find_max_object(blobs0)
        color_num = 1
    else:
        blobs1 = img.find_blobs(yellow_threshold, pixels_threshold=100, merge=True)
        if blobs1:
            max_blob = find_max_object(blobs1)
            color_num = 2
        else:
            blobs2 = img.find_blobs(blue_threshold, pixels_threshold=100, merge=True)
            if blobs2:
                max_blob = find_max_object(blobs2)
                color_num = 3
    if color_num > 0:
        if color_num == 1:
            cor_dst = (-220, 0, 10)
        elif color_num == 2:
            cor_dst = (-170, 0, 10)
        elif color_num == 3:
            cor_dst = (-120, 0, 10)
        if max_blob.cx() > 0 and max_blob.cx() < img.width() and max_blob.cy() > 0 and max_blob.cy() < img.height():
            img.draw_rectangle(max_blob.rect())
            img.draw_cross(max_blob.cx(), max_blob.cy())
            pan_error = img.width() / 2 - max_blob.cx()
            tilt_error = img.height() / 2 - max_blob.cy()
            pan_output = pan_pid.get_pid(pan_error, 1) / 2
            tilt_output = tilt_pid.get_pid(tilt_error, 1) / 2
            pan_angle += pan_output
            tilt_angle -= tilt_output
            if tilt_angle >= 160:
                tilt_angle = 160
            servo_angle(S1, pan_angle, S1_offset)
            servo_angle(S3, tilt_angle, S3_offset)
            if abs(pan_error) <= 0.1 and abs(tilt_error) <= 0.1:
                count += 1
                if count >= 5:
                    count = 0
                    Lxy = calculate_object_xy(pan_angle, tilt_angle, DIS_offset)
                    if 130 <= Lxy[1] <= 300:
                        arm_angle = image.calculate_machine_hand(coordinate=(Lxy[0], Lxy[1], 10), armlength=ARM_LENGHT)
                        if (arm_angle[1] + arm_angle[2]) == 0 or arm_angle[2] > 160:
                            return
                        angle_start = (pan_angle, dis_angle, tilt_angle)
                        machineArm_move_to_coordinate(angle_start, arm_angle)
                        servo_angle(S4, CLAW_CATCH)
                        time.sleep_ms(1000)
                        machineArm_move_to_start(arm_angle)
                        grab_completed = True
                        uart.write("GRAB_OK\r\n")

servo_angle(S1, pan_angle, S1_offset)
servo_angle(S2, dis_angle, S2_offset)
servo_angle(S3, tilt_angle, S3_offset)
servo_angle(S4, CLAW_RELEASE)

sensor.reset()
sensor.__write_reg(0x11, 0x80)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.HQVGA)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2000)
sensor.set_hmirror(True)
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    # 监听下层指令
    if uart.any():
        data = uart.readline()
        if data:
            if b"START_GRAB" in data:
                allow_grab = True
                allow_release = False
                grab_completed = False
            if b"START_RELEASE" in data:
                allow_grab = False
                allow_release = True
                arm_angle1 = image.calculate_machine_hand(coordinate=cor_dst, armlength=ARM_LENGHT)
                machineArm_move_to_dest((90, 90, 135), arm_angle1)
                servo_angle(S4, CLAW_RELEASE)
                machineArm_move_to_start(arm_angle1)
                uart.write("RELEASE_OK\r\n")

    # 只有收到START_GRAB，才执行视觉+逆解+抓取
    if allow_grab:
        color_detect(img)

    time.sleep(0.05)