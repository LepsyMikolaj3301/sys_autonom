#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define TIME_STEP    64
#define MAX_SPEED    6.28
#define BASE_SPEED   4.0

// Progi ścian z histerezą
#define WALL_ON      150.0
#define WALL_OFF      80.0

// Detekcja celu
#define GREEN_PIXEL_MIN    30
#define GREEN_PIXEL_CLOSE  2000

// Omijanie ściany przy widocznym celu
#define AVOID_STEPS  8

// Spotkanie i podążanie
#define MEET_DISTANCE       0.11
#define FOLLOW_DISTANCE     0.20
#define TOO_CLOSE_DISTANCE  0.13

// Opóźnienie formacji po spotkaniu
#define FORMATION_DELAY 1.5
#define LEADER_STUCK_GRACE_TIME 2.0

// Wykrywanie utknięcia przez brak postępu
#define STUCK_CHECK_STEPS   15
#define STUCK_MOVE_EPS      0.035
#define DEAD_END_STEPS      6

// Wykrywanie okrążenia w ślepym zaułku
#define LOOP_TURN_LIMIT     5.2     // około 298 stopni
#define LOOP_MIN_TIME       1.5
#define LOOP_RESET_DISTANCE 0.12

// Historia pozycji lidera dla followera
#define LEADER_HISTORY_SIZE 80
#define FOLLOW_HISTORY_DELAY 18
#define TARGET_REACHED_DISTANCE 0.045

typedef enum {
  MODE_SEARCH = 0,
  MODE_LEADER = 1,
  MODE_FOLLOWER = 2
} Mode;

static WbDeviceTag left_motor, right_motor;
static WbDeviceTag ps[8];
static WbDeviceTag cam;

static int width, height;

static bool front_blocked = false;
static int avoid_steps_left = 0;
static int avoid_direction = 0;

static const char *my_name;
static const char *other_name;
static const char *leader_name = NULL;

static WbNodeRef my_node;
static WbNodeRef other_node;

// Liczniki utknięcia własnego robota jako lidera
static double last_leader_x = 0.0;
static double last_leader_y = 0.0;
static int stuck_check_counter = 0;
static int no_progress_counter = 0;

// Liczniki obserwacji lidera przez followera
static double observed_last_x = 0.0;
static double observed_last_y = 0.0;
static int observed_check_counter = 0;
static int observed_no_progress_counter = 0;

// Czas rozpoczęcia jazdy zespołowej po spotkaniu
static double team_start_time = 0.0;

// Detektor okrążenia lidera
static double last_yaw_for_loop = 0.0;
static double accumulated_left_turn = 0.0;
static double loop_start_x = 0.0;
static double loop_start_y = 0.0;
static double loop_start_time = 0.0;

// Historia pozycji lidera
static double leader_history_x[LEADER_HISTORY_SIZE];
static double leader_history_y[LEADER_HISTORY_SIZE];
static int leader_history_index = 0;
static int leader_history_filled = 0;

static double clamp(double x, double min, double max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

static void set_speed(double left, double right) {
  left = clamp(left, -MAX_SPEED, MAX_SPEED);
  right = clamp(right, -MAX_SPEED, MAX_SPEED);

  wb_motor_set_velocity(left_motor, left);
  wb_motor_set_velocity(right_motor, right);
}

static bool is_green(int r, int g, int b) {
  return g > 80 && g > r * 1.5 && g > b * 1.5;
}

static double normalize_angle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double distance_between_robots(void) {
  const double *p1 = wb_supervisor_node_get_position(my_node);
  const double *p2 = wb_supervisor_node_get_position(other_node);

  double dx = p2[0] - p1[0];
  double dy = p2[1] - p1[1];

  return sqrt(dx * dx + dy * dy);
}

static double get_robot_yaw(WbNodeRef node) {
  const double *m = wb_supervisor_node_get_orientation(node);

  /*
    Dla robota poruszającego się po płaszczyźnie bierzemy kierunek osi X robota
    rzutowany na płaszczyznę XY.
  */
  return atan2(m[3], m[0]);
}

static bool is_front_blocked_now(double v[8]) {
  double front_max = v[0] > v[7] ? v[0] : v[7];
  double front_diag = v[1] > v[6] ? v[1] : v[6];
  double front_val = front_max > front_diag ? front_max : front_diag;

  return front_val > WALL_ON;
}

static void reset_loop_detector(void) {
  const double *pos = wb_supervisor_node_get_position(my_node);

  last_yaw_for_loop = get_robot_yaw(my_node);
  accumulated_left_turn = 0.0;

  loop_start_x = pos[0];
  loop_start_y = pos[1];
  loop_start_time = wb_robot_get_time();
}

static bool leader_made_counterclockwise_loop(void) {
  const double *pos = wb_supervisor_node_get_position(my_node);

  double current_yaw = get_robot_yaw(my_node);
  double delta = normalize_angle(current_yaw - last_yaw_for_loop);

  last_yaw_for_loop = current_yaw;

  /*
    Robot trzyma się prawej ściany, więc w ślepej kiszce zwykle robi narastający
    obrót przeciwnie do ruchu wskazówek zegara. Zliczamy tylko dodatni obrót.
  */
  if (delta > 0.0)
    accumulated_left_turn += delta;

  double dx = pos[0] - loop_start_x;
  double dy = pos[1] - loop_start_y;
  double distance_from_loop_start = sqrt(dx * dx + dy * dy);

  double time_from_loop_start = wb_robot_get_time() - loop_start_time;

  /*
    Jeżeli robot wyjechał dalej, to nie jest ślepy zaułek — resetujemy licznik.
  */
  if (distance_from_loop_start > LOOP_RESET_DISTANCE) {
    reset_loop_detector();
    return false;
  }

  /*
    Jeśli robot został w małym obszarze i uzbierał prawie pełny obrót w lewo,
    uznajemy, że krąży w ślepej kiszce.
  */
  if (time_from_loop_start > LOOP_MIN_TIME &&
      accumulated_left_turn > LOOP_TURN_LIMIT) {
    return true;
  }

  return false;
}

static bool leader_is_stuck(double v[8]) {
  const double *pos = wb_supervisor_node_get_position(my_node);

  stuck_check_counter++;

  if (stuck_check_counter < STUCK_CHECK_STEPS)
    return false;

  stuck_check_counter = 0;

  double dx = pos[0] - last_leader_x;
  double dy = pos[1] - last_leader_y;
  double move = sqrt(dx * dx + dy * dy);

  last_leader_x = pos[0];
  last_leader_y = pos[1];

  bool front = is_front_blocked_now(v);

  bool side_or_front_blocked =
    v[0] > WALL_ON || v[1] > WALL_ON || v[2] > WALL_ON ||
    v[5] > WALL_ON || v[6] > WALL_ON || v[7] > WALL_ON;

  if (move < STUCK_MOVE_EPS && (front || side_or_front_blocked)) {
    no_progress_counter++;
  } else {
    no_progress_counter = 0;
  }

  if (no_progress_counter >= DEAD_END_STEPS) {
    no_progress_counter = 0;
    return true;
  }

  return false;
}

static bool observed_leader_is_stuck(void) {
  WbNodeRef leader_node;

  if (strcmp(leader_name, my_name) == 0)
    leader_node = my_node;
  else
    leader_node = other_node;

  const double *pos = wb_supervisor_node_get_position(leader_node);

  observed_check_counter++;

  if (observed_check_counter < STUCK_CHECK_STEPS)
    return false;

  observed_check_counter = 0;

  double dx = pos[0] - observed_last_x;
  double dy = pos[1] - observed_last_y;
  double move = sqrt(dx * dx + dy * dy);

  observed_last_x = pos[0];
  observed_last_y = pos[1];

  if (move < STUCK_MOVE_EPS) {
    observed_no_progress_counter++;
  } else {
    observed_no_progress_counter = 0;
  }

  if (observed_no_progress_counter >= DEAD_END_STEPS) {
    observed_no_progress_counter = 0;
    return true;
  }

  return false;
}

static void reset_leader_history(void) {
  WbNodeRef leader_node;

  if (leader_name != NULL && strcmp(leader_name, my_name) == 0)
    leader_node = my_node;
  else
    leader_node = other_node;

  const double *leader_pos = wb_supervisor_node_get_position(leader_node);

  for (int i = 0; i < LEADER_HISTORY_SIZE; i++) {
    leader_history_x[i] = leader_pos[0];
    leader_history_y[i] = leader_pos[1];
  }

  leader_history_index = 0;
  leader_history_filled = LEADER_HISTORY_SIZE;
}

static void reset_stuck_counters(void) {
  stuck_check_counter = 0;
  no_progress_counter = 0;

  observed_check_counter = 0;
  observed_no_progress_counter = 0;

  const double *my_pos = wb_supervisor_node_get_position(my_node);

  last_leader_x = my_pos[0];
  last_leader_y = my_pos[1];

  WbNodeRef leader_node;

  if (leader_name != NULL && strcmp(leader_name, my_name) == 0)
    leader_node = my_node;
  else
    leader_node = other_node;

  const double *leader_pos = wb_supervisor_node_get_position(leader_node);

  observed_last_x = leader_pos[0];
  observed_last_y = leader_pos[1];
}

static void swap_leader(void) {
  if (strcmp(leader_name, "e-puck") == 0)
    leader_name = "e-puck(1)";
  else
    leader_name = "e-puck";

  reset_stuck_counters();
  reset_loop_detector();
  reset_leader_history();

  printf("[%s] Zmiana lidera. Nowy lider: %s\n", my_name, leader_name);
}

static const char *choose_leader(void) {
  /*
    Prosty wspólny wybór lidera.
    Oba roboty używają czasu symulacji, więc powinny wybrać tę samą stronę.
  */
  int seed = (int)(wb_robot_get_time() * 1000.0);

  if (seed % 2 == 0)
    return "e-puck";
  else
    return "e-puck(1)";
}

static bool search_or_leader_step(double v[8]) {
  const unsigned char *image = wb_camera_get_image(cam);

  int green_count = 0;
  int green_x_sum = 0;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int r = wb_camera_image_get_red(image, width, x, y);
      int g = wb_camera_image_get_green(image, width, x, y);
      int b = wb_camera_image_get_blue(image, width, x, y);

      if (is_green(r, g, b)) {
        green_count++;
        green_x_sum += x;
      }
    }
  }

  // Jeżeli cel jest widoczny
  if (green_count >= GREEN_PIXEL_MIN) {
    int green_cx = green_x_sum / green_count;
    int img_cx = width / 2;
    int offset = green_cx - img_cx;

    bool touching = v[0] > 800.0 || v[7] > 800.0;

    if (touching || green_count >= GREEN_PIXEL_CLOSE) {
      set_speed(0.0, 0.0);
      printf("[%s] >>> CEL OSIAGNIETY! <<<\n", my_name);
      return true;
    }

    double front_max = v[0] > v[7] ? v[0] : v[7];
    double front_diag = v[1] > v[6] ? v[1] : v[6];
    double front_val = front_max > front_diag ? front_max : front_diag;

    if (!front_blocked && front_val > WALL_ON) {
      front_blocked = true;
      avoid_direction = (offset < 0) ? -1 : 1;
      avoid_steps_left = AVOID_STEPS;
    } else if (front_blocked && front_val < WALL_OFF) {
      front_blocked = false;
      avoid_steps_left = 0;
    }

    if (avoid_steps_left > 0) {
      double turn = BASE_SPEED * 0.5;

      if (avoid_direction < 0)
        set_speed(-turn, turn);
      else
        set_speed(turn, -turn);

      avoid_steps_left--;
    } else {
      double approach = 1.0 - ((double)green_count / (double)GREEN_PIXEL_CLOSE);

      if (approach < 0.25)
        approach = 0.25;

      double spd = BASE_SPEED * approach;
      double steer = (double)offset / (double)img_cx;
      double correction = steer * spd * 0.6;

      set_speed(spd + correction, spd - correction);
    }

    return false;
  }

  // Jeżeli celu nie widać — jazda prawą ścianą
  double front_val = v[0] > v[7] ? v[0] : v[7];
  double diag_val = v[1] > v[6] ? v[1] : v[6];
  double fv = front_val > diag_val ? front_val : diag_val;

  if (!front_blocked && fv > WALL_ON)
    front_blocked = true;

  if (front_blocked && fv < WALL_OFF)
    front_blocked = false;

  double left_speed = BASE_SPEED;
  double right_speed = BASE_SPEED;

  bool wall_right = v[2] > WALL_ON;

  if (front_blocked) {
    left_speed = -BASE_SPEED * 0.6;
    right_speed = BASE_SPEED * 0.6;
  } else if (wall_right) {
    if (v[2] > WALL_ON * 1.5) {
      left_speed = BASE_SPEED * 0.8;
      right_speed = BASE_SPEED;
    } else if (v[2] < WALL_ON * 0.8) {
      left_speed = BASE_SPEED;
      right_speed = BASE_SPEED * 0.8;
    }
  } else {
    left_speed = BASE_SPEED;
    right_speed = BASE_SPEED * 0.4;
  }

  set_speed(left_speed, right_speed);
  return false;
}

static void follower_step(double v[8]) {
  WbNodeRef leader_node;

  if (strcmp(leader_name, my_name) == 0)
    leader_node = my_node;
  else
    leader_node = other_node;

  const double *my_pos = wb_supervisor_node_get_position(my_node);
  const double *leader_pos = wb_supervisor_node_get_position(leader_node);

  double leader_dx = leader_pos[0] - my_pos[0];
  double leader_dy = leader_pos[1] - my_pos[1];
  double leader_distance = sqrt(leader_dx * leader_dx + leader_dy * leader_dy);

  double time_after_team = wb_robot_get_time() - team_start_time;

  /*
    Po spotkaniu follower chwilę stoi, żeby lider zrobił odstęp.
  */
  if (time_after_team < FORMATION_DELAY) {
    set_speed(0.0, 0.0);
    return;
  }

  /*
    Zapis aktualnej pozycji lidera.
  */
  leader_history_x[leader_history_index] = leader_pos[0];
  leader_history_y[leader_history_index] = leader_pos[1];

  leader_history_index = (leader_history_index + 1) % LEADER_HISTORY_SIZE;

  if (leader_history_filled < LEADER_HISTORY_SIZE)
    leader_history_filled++;

  int delay = FOLLOW_HISTORY_DELAY;

  if (delay >= leader_history_filled)
    delay = leader_history_filled - 1;

  int target_index = leader_history_index - delay;

  if (target_index < 0)
    target_index += LEADER_HISTORY_SIZE;

  double target_x = leader_history_x[target_index];
  double target_y = leader_history_y[target_index];

  double dx = target_x - my_pos[0];
  double dy = target_y - my_pos[1];

  double target_distance = sqrt(dx * dx + dy * dy);

  if (target_distance < TARGET_REACHED_DISTANCE) {
    set_speed(0.0, 0.0);
    return;
  }

  /*
    Jeżeli follower jest bardzo blisko lidera, nie pcha go.
  */
  if (leader_distance < TOO_CLOSE_DISTANCE) {
    set_speed(-BASE_SPEED * 0.12, -BASE_SPEED * 0.12);
    return;
  }

  double target_angle = atan2(dy, dx);
  double my_yaw = get_robot_yaw(my_node);
  double error = normalize_angle(target_angle - my_yaw);

  bool emergency_front =
    v[0] > 750.0 || v[7] > 750.0 || v[1] > 800.0 || v[6] > 800.0;

  if (emergency_front) {
    set_speed(-BASE_SPEED * 0.10, -BASE_SPEED * 0.10);
    return;
  }

  double speed;

  if (leader_distance < FOLLOW_DISTANCE)
    speed = BASE_SPEED * 0.12;
  else
    speed = BASE_SPEED * 0.30;

  if (fabs(error) > 1.0) {
    double turn = clamp(error * 1.0, -1.2, 1.2);
    set_speed(-turn, turn);
    return;
  }

  double turn = clamp(error * 1.2, -1.3, 1.3);

  double left = speed - turn;
  double right = speed + turn;

  set_speed(left, right);
}

int main(void) {
  wb_robot_init();

  my_name = wb_robot_get_name();

  if (strcmp(my_name, "e-puck") == 0)
    other_name = "e-puck(1)";
  else
    other_name = "e-puck";

  /*
    W pliku .wbt najlepiej:
    DEF ROBOT_A E-puck { name "e-puck" ... }
    DEF ROBOT_B E-puck { name "e-puck(1)" ... }
  */
  if (strcmp(my_name, "e-puck") == 0) {
    my_node = wb_supervisor_node_get_from_def("ROBOT_A");
    other_node = wb_supervisor_node_get_from_def("ROBOT_B");
  } else {
    my_node = wb_supervisor_node_get_from_def("ROBOT_B");
    other_node = wb_supervisor_node_get_from_def("ROBOT_A");
  }

  if (my_node == NULL || other_node == NULL) {
    printf("[%s] BLAD: Nie moge znalezc robotow przez DEF.\n", my_name);
    printf("Dodaj w .wbt: DEF ROBOT_A i DEF ROBOT_B przy robotach.\n");
    wb_robot_cleanup();
    return 1;
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  set_speed(0.0, 0.0);

  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  for (int i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);

  width = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam);

  printf("[%s] Start. Drugi robot: %s\n", my_name, other_name);

  Mode mode = MODE_SEARCH;

  while (wb_robot_step(TIME_STEP) != -1) {
    double v[8];

    for (int i = 0; i < 8; i++)
      v[i] = wb_distance_sensor_get_value(ps[i]);

    double d = distance_between_robots();

    /*
      Spotkanie robotów.
    */
    if (mode == MODE_SEARCH && d < MEET_DISTANCE) {
      leader_name = choose_leader();

      team_start_time = wb_robot_get_time();

      reset_stuck_counters();
      reset_loop_detector();
      reset_leader_history();

      if (strcmp(my_name, leader_name) == 0) {
        mode = MODE_LEADER;
        printf("[%s] Spotkanie. Zostalem liderem.\n", my_name);
      } else {
        mode = MODE_FOLLOWER;
        printf("[%s] Spotkanie. Podazam za liderem: %s\n", my_name, leader_name);
      }
    }

    if (mode == MODE_SEARCH) {
      bool reached = search_or_leader_step(v);

      if (reached)
        break;
    }

    else if (mode == MODE_LEADER) {
      double time_after_team = wb_robot_get_time() - team_start_time;

      bool stuck_by_position = false;
      bool stuck_by_loop = false;

      if (time_after_team > LEADER_STUCK_GRACE_TIME) {
        stuck_by_position = leader_is_stuck(v);
        stuck_by_loop = leader_made_counterclockwise_loop();
      }

      if (stuck_by_position || stuck_by_loop) {
        if (stuck_by_loop)
          printf("[%s] Wykryto okrazenie w lewo - slepy zaulek.\n", my_name);
        else
          printf("[%s] Wykryto brak postepu.\n", my_name);

        printf("[%s] UTKNALEM - wysylam sygnal zmiany lidera.\n", my_name);

        swap_leader();

        team_start_time = wb_robot_get_time();

        if (strcmp(my_name, leader_name) == 0) {
          mode = MODE_LEADER;
          printf("[%s] Nadal jestem liderem.\n", my_name);
        } else {
          mode = MODE_FOLLOWER;
          printf("[%s] Przechodze w tryb followera.\n", my_name);
        }

        continue;
      }

      bool reached = search_or_leader_step(v);

      if (reached)
        break;
    }

    else if (mode == MODE_FOLLOWER) {
      if (observed_leader_is_stuck()) {
        printf("[%s] Odebralem/wykrylem sygnal: lider utknal. Zmiana lidera.\n", my_name);

        swap_leader();

        team_start_time = wb_robot_get_time();

        if (strcmp(my_name, leader_name) == 0) {
          mode = MODE_LEADER;
          printf("[%s] Teraz ja jestem liderem.\n", my_name);
        } else {
          mode = MODE_FOLLOWER;
          printf("[%s] Nadal jestem followerem.\n", my_name);
        }

        continue;
      }

      follower_step(v);
    }
  }

  wb_robot_cleanup();
  return 0;
}