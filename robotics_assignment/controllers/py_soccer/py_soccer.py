from controller import Robot, Motion

TIME_STEP = 64

SCAN_ROWS = 20
CENTER_TOLERANCE = 35

TARGET_R = 255
TARGET_G = 138
TARGET_B = 20
COLOR_TOLERANCE = 80

HEAD_CENTER = 0.0
HEAD_DOWN_1 = 0.25
HEAD_DOWN_2 = 0.45

MID_THRESHOLD = 90
KICK_SIDE_OFFSET = -30
LOST_LIMIT = 8

robot = Robot()

cam_top = robot.getDevice('CameraTop')
cam_bottom = robot.getDevice('CameraBottom')
cam_top.enable(TIME_STEP)
cam_bottom.enable(TIME_STEP)

width_top = cam_top.getWidth()
height_top = cam_top.getHeight()
width_bottom = cam_bottom.getWidth()
height_bottom = cam_bottom.getHeight()

head_pitch = robot.getDevice('HeadPitch')
head_pitch.setPosition(HEAD_CENTER)

motion_left    = Motion('../../motions/SideStepLeft.motion')
motion_right   = Motion('../../motions/SideStepRight.motion')
motion_forward = Motion('../../motions/Forwards50.motion')
motion_wave    = Motion('../../motions/HandWave.motion')
motion_kick    = Motion('../../motions/Shoot.motion')

is_motion_running = False
is_cooldown = False
cooldown_timer = 0
search_stage = 0
lost_counter = 0
kick_count = 0
task_done = False


def play(motion):
    global is_motion_running
    if motion is None:
        return
    is_motion_running = True
    motion.play()
    while not motion.isOver():
        robot.step(TIME_STEP)
    is_motion_running = False


def is_ball_pixel(r, g, b):
    return (abs(r - TARGET_R) < COLOR_TOLERANCE and
            abs(g - TARGET_G) < COLOR_TOLERANCE and
            abs(b - TARGET_B) < COLOR_TOLERANCE)


def detect_ball_x(image_bytes, width, height):
    best_x = -1
    best_size = 0

    for y in range(SCAN_ROWS):
        row = height - 1 - y * 2
        if row < 0:
            break

        start = -1
        cluster_width = 0

        for x in range(width):
            # Webots image is BGRA — index 2=R, 1=G, 0=B per pixel
            base = (row * width + x) * 4
            b_val = image_bytes[base]
            g_val = image_bytes[base + 1]
            r_val = image_bytes[base + 2]

            if is_ball_pixel(r_val, g_val, b_val):
                if start == -1:
                    start = x
                cluster_width += 1
            else:
                if start != -1 and cluster_width > best_size:
                    best_size = cluster_width
                    best_x = start + cluster_width // 2
                start = -1
                cluster_width = 0

    return best_x, best_size


while robot.step(TIME_STEP) != -1:

    if task_done:
        continue

    if is_cooldown:
        cooldown_timer -= 1
        if cooldown_timer <= 0:
            is_cooldown = False
        continue

    if is_motion_running:
        continue

    img_top    = cam_top.getImage()
    img_bottom = cam_bottom.getImage()

    ball_x_top,    size_top    = -1, 0
    ball_x_bottom, size_bottom = -1, 0

    if img_top:
        ball_x_top, size_top = detect_ball_x(img_top, width_top, height_top)
    if img_bottom:
        ball_x_bottom, size_bottom = detect_ball_x(img_bottom, width_bottom, height_bottom)

    center_top    = width_top // 2
    center_bottom = width_bottom // 2

    bottom_seen = (ball_x_bottom != -1 and size_bottom > 20)
    top_seen    = (ball_x_top != -1)

    if bottom_seen:
        search_stage = 0
        lost_counter = 0

        target_center = center_bottom + KICK_SIDE_OFFSET
        error = ball_x_bottom - target_center

        if abs(error) > CENTER_TOLERANCE:
            if error < 0:
                play(motion_left)
            else:
                play(motion_right)
        else:
            play(motion_kick)
            kick_count += 1
            if kick_count >= 2:
                task_done = True
            is_cooldown = True
            cooldown_timer = 1000 // TIME_STEP

    elif top_seen:
        search_stage = 0
        lost_counter = 0

        head_pitch.setPosition(HEAD_CENTER)
        error = ball_x_top - center_top

        if abs(error) > CENTER_TOLERANCE:
            if error < 0:
                play(motion_left)
            else:
                play(motion_right)
        else:
            play(motion_forward)

    else:
        lost_counter += 1

        if lost_counter < LOST_LIMIT:
            if search_stage == 0:
                head_pitch.setPosition(HEAD_DOWN_1)
                search_stage = 1
            elif search_stage == 1:
                head_pitch.setPosition(HEAD_DOWN_2)
                search_stage = 2
        else:
            play(motion_wave)
