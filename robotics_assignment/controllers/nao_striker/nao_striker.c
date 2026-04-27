#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/utils/motion.h>
#include <webots/motor.h>

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define STEP_MS 64

#define ROW_COUNT 20
#define ALIGN_EPS 35

#define BALL_R 255
#define BALL_G 138
#define BALL_B 20
#define COLOR_DELTA 80

#define NECK_FLAT 0.0
#define NECK_TILT1 0.25
#define NECK_TILT2 0.45

#define NEAR_THRESHOLD 90

#define STRIKE_OFFSET -30

#define MAX_MISS 8

/* distance gating — kick only when ball appears in this size range  */
#define KICK_SIZE_MIN 20
#define KICK_SIZE_MAX 90

/* let the robot settle into stance before reacting to the camera   */
#define STARTUP_STEPS 32

static WbDeviceTag upper_cam, lower_cam;
static int up_w, up_h;
static int lo_w, lo_h;

static WbDeviceTag neck_pitch;

static WbMotionRef m_step_l, m_step_r, m_walk;
static WbMotionRef m_greet, m_strike, m_back;

static bool motion_busy = false;
static bool in_pause = false;
static int  pause_left = 0;

static int  look_step = 0;
static int  miss_count = 0;

static int  strikes = 0;
static bool finished = false;

static void setup(void);
static void stabilize(void);
static void run_motion(WbMotionRef motion, const char *label);
static int  scan_ball(const unsigned char *image, int width, int height, int *size_out);
static bool pixel_match(int r, int g, int b);

int main() {

  wb_robot_init();
  setup();
  stabilize();

  /* let physics settle so the kick decision isn't made mid-fall */
  for (int s = 0; s < STARTUP_STEPS; s++)
    wb_robot_step(STEP_MS);

  while (wb_robot_step(STEP_MS) != -1) {

    if (finished)
      continue;

    if (in_pause) {
      pause_left--;
      if (pause_left <= 0) in_pause = false;
      continue;
    }

    if (motion_busy)
      continue;

    const unsigned char *img_up = wb_camera_get_image(upper_cam);
    const unsigned char *img_lo = wb_camera_get_image(lower_cam);

    int ball_x_up = -1, sz_up = 0;
    int ball_x_lo = -1, sz_lo = 0;

    if (img_up)
      ball_x_up = scan_ball(img_up, up_w, up_h, &sz_up);

    if (img_lo)
      ball_x_lo = scan_ball(img_lo, lo_w, lo_h, &sz_lo);

    int mid_up = up_w / 2;
    int mid_lo = lo_w / 2;

    bool low_ok  = (ball_x_lo != -1 && sz_lo > KICK_SIZE_MIN);
    bool high_ok = (ball_x_up != -1);

    printf("vision: lo=%d sz_lo=%d  up=%d sz_up=%d\n",
           ball_x_lo, sz_lo, ball_x_up, sz_up);
    fflush(stdout);

    if (low_ok) {

      look_step = 0;
      miss_count = 0;

      int aim = mid_lo + STRIKE_OFFSET;
      int delta = ball_x_lo - aim;

      if (abs(delta) > ALIGN_EPS) {

        if (delta < 0)
          run_motion(m_step_l, "Foot Adjust Left");
        else
          run_motion(m_step_r, "Foot Adjust Right");

      } else if (sz_lo > KICK_SIZE_MAX) {

        /* ball too close to kick cleanly — back up to ideal range */
        printf("too close (sz=%d) -> BACK UP\n", sz_lo);
        fflush(stdout);
        run_motion(m_back, "Back Up");

      } else {

        printf("aligned (sz=%d delta=%d) -> KICK\n", sz_lo, delta);
        fflush(stdout);
        run_motion(m_strike, "KICK");

        strikes++;
        if (strikes >= 2)
          finished = true;

        in_pause = true;
        pause_left = 1000 / STEP_MS;
      }
    }

    else if (high_ok) {

      look_step = 0;
      miss_count = 0;

      wb_motor_set_position(neck_pitch, NECK_FLAT);

      int delta = ball_x_up - mid_up;

      if (abs(delta) > ALIGN_EPS) {

        if (delta < 0)
          run_motion(m_step_l, "Align Left");
        else
          run_motion(m_step_r, "Align Right");

      } else {

        if (sz_up > NEAR_THRESHOLD)
          run_motion(m_walk, "Approach");
        else
          run_motion(m_walk, "Approach");
      }
    }

    else {

      miss_count++;

      if (miss_count < MAX_MISS) {

        if (look_step == 0) {
          wb_motor_set_position(neck_pitch, NECK_TILT1);
          look_step = 1;
        }
        else if (look_step == 1) {
          wb_motor_set_position(neck_pitch, NECK_TILT2);
          look_step = 2;
        }

      } else {

        run_motion(m_greet, "Searching");
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}

static void setup(void) {

  upper_cam = wb_robot_get_device("CameraTop");
  lower_cam = wb_robot_get_device("CameraBottom");

  wb_camera_enable(upper_cam, STEP_MS);
  wb_camera_enable(lower_cam, STEP_MS);

  up_w = wb_camera_get_width(upper_cam);
  up_h = wb_camera_get_height(upper_cam);

  lo_w = wb_camera_get_width(lower_cam);
  lo_h = wb_camera_get_height(lower_cam);

  neck_pitch = wb_robot_get_device("HeadPitch");
  wb_motor_set_position(neck_pitch, NECK_FLAT);

  m_step_l = wbu_motion_new("../../motions/SideStepLeft.motion");
  m_step_r = wbu_motion_new("../../motions/SideStepRight.motion");
  m_walk   = wbu_motion_new("../../motions/Forwards50.motion");
  m_greet  = wbu_motion_new("../../motions/HandWave.motion");
  m_strike = wbu_motion_new("../../motions/Shoot.motion");
  m_back   = wbu_motion_new("../../motions/Backwards.motion");
}

/* drive every major joint to a known-stable NAO standing pose so the
   robot doesn't collapse on a clean world load. hip + knee + ankle
   pitches sum to ~0 -> feet stay flat while torso stays upright.   */
static void stabilize(void) {

  WbDeviceTag j;

  j = wb_robot_get_device("LShoulderPitch"); wb_motor_set_position(j,  1.45);
  j = wb_robot_get_device("RShoulderPitch"); wb_motor_set_position(j,  1.45);
  j = wb_robot_get_device("LShoulderRoll");  wb_motor_set_position(j,  0.10);
  j = wb_robot_get_device("RShoulderRoll");  wb_motor_set_position(j, -0.10);
  j = wb_robot_get_device("LElbowRoll");     wb_motor_set_position(j, -0.50);
  j = wb_robot_get_device("RElbowRoll");     wb_motor_set_position(j,  0.50);

  j = wb_robot_get_device("LHipPitch");      wb_motor_set_position(j, -0.45);
  j = wb_robot_get_device("RHipPitch");      wb_motor_set_position(j, -0.45);
  j = wb_robot_get_device("LKneePitch");     wb_motor_set_position(j,  0.96);
  j = wb_robot_get_device("RKneePitch");     wb_motor_set_position(j,  0.96);
  j = wb_robot_get_device("LAnklePitch");    wb_motor_set_position(j, -0.51);
  j = wb_robot_get_device("RAnklePitch");    wb_motor_set_position(j, -0.51);
}

static void run_motion(WbMotionRef motion, const char *label) {

  if (motion == NULL)
    return;

  motion_busy = true;
  wbu_motion_play(motion);

  while (!wbu_motion_is_over(motion))
    wb_robot_step(STEP_MS);

  motion_busy = false;
}

static int scan_ball(const unsigned char *image, int width, int height, int *size_out) {

  int best_x = -1;
  int best_size = 0;

  for (int y = 0; y < ROW_COUNT; y++) {

    int row = height - 1 - y * 2;

    int start = -1;
    int width_cluster = 0;

    for (int x = 0; x < width; x++) {

      int r = wb_camera_image_get_red(image, width, x, row);
      int g = wb_camera_image_get_green(image, width, x, row);
      int b = wb_camera_image_get_blue(image, width, x, row);

      if (pixel_match(r, g, b)) {
        if (start == -1) start = x;
        width_cluster++;
      } else {
        if (start != -1 && width_cluster > best_size) {
          best_size = width_cluster;
          best_x = start + width_cluster / 2;
        }
        start = -1;
        width_cluster = 0;
      }
    }
  }

  *size_out = best_size;
  return best_x;
}

static bool pixel_match(int r, int g, int b) {
  return (abs(r - BALL_R) < COLOR_DELTA &&
          abs(g - BALL_G) < COLOR_DELTA &&
          abs(b - BALL_B) < COLOR_DELTA);
}
