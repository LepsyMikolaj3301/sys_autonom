#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define TIME_STEP         64
#define MAX_SPEED         6.28
#define BASE_SPEED        4.0

/* Progi scian z histereza */
#define WALL_ON           150.0
#define WALL_OFF           80.0

/* Detekcja celu */
#define GREEN_PIXEL_MIN    30
#define GREEN_PIXEL_CLOSE  2000

/* Omijanie sciany przy widocznym celu */
#define AVOID_STEPS        8

/* Spotkanie — scisley prog + potwierdzenie wielokrokowe */
#define MEET_DISTANCE        0.09
#define MEET_CONFIRM_STEPS   3

/* Podazanie */
#define FOLLOW_DISTANCE      0.15
#define TOO_CLOSE_DISTANCE   0.10
#define CATCH_UP_DISTANCE    0.25

/* Opoznienie formacji po spotkaniu / zamianie */
#define FORMATION_DELAY          2.0
#define LEADER_STUCK_GRACE_TIME  2.5

/* Wykrywanie utknięcia przez brak postepu */
#define STUCK_CHECK_STEPS   15
#define STUCK_MOVE_EPS      0.035
#define DEAD_END_STEPS      6

/* Wykrywanie okrazenia w slepym zaulku */
#define LOOP_TURN_LIMIT     5.2
#define LOOP_MIN_TIME       1.5
#define LOOP_RESET_DISTANCE 0.12

/* Historia pozycji lidera — sekwencyjne waypoint-y */
#define LEADER_PATH_SIZE      200
#define MIN_FOLLOW_GAP         8
#define WAYPOINT_REACHED_DIST  0.025

typedef enum {
  MODE_SEARCH   = 0,
  MODE_LEADER   = 1,
  MODE_FOLLOWER = 2
} Mode;

/* ================================================================ */
/*  Zmienne globalne                                                 */
/* ================================================================ */

static WbDeviceTag left_motor, right_motor;
static WbDeviceTag ps[8];
static WbDeviceTag cam;
static WbDeviceTag emitter, receiver;

static int width, height;

/* Stan nawigacji (resetowany przy zmianie trybu) */
static bool front_blocked   = false;
static int  avoid_steps_left = 0;
static int  avoid_direction  = 0;

/* Tozsamosc */
static const char *my_name;
static const char *other_name;
static const char *leader_name = NULL;

static WbNodeRef my_node;
static WbNodeRef other_node;

/* Flaga osiagniecia celu (ustawiana przez radio) */
static bool goal_reached = false;

/* Potwierdzenie spotkania — licznik kolejnych krokow bliskosci */
static int meet_confirm_counter = 0;

/* Wykrywanie utknięcia lidera (wlasne liczniki) */
static double last_leader_x = 0.0;
static double last_leader_y = 0.0;
static int    stuck_check_counter  = 0;
static int    no_progress_counter  = 0;

/* Czas rozpoczecia jazdy zespolowej */
static double team_start_time = 0.0;

/* Detektor okrazenia lidera (slepy zaulek) */
static double last_yaw_for_loop     = 0.0;
static double accumulated_left_turn = 0.0;
static double loop_start_x          = 0.0;
static double loop_start_y          = 0.0;
static double loop_start_time       = 0.0;

/* Historia pozycji lidera (bufor kolowy) */
static double leader_path_x[LEADER_PATH_SIZE];
static double leader_path_y[LEADER_PATH_SIZE];
static int    path_write_idx  = 0;
static int    follow_read_idx = 0;
static int    path_count      = 0;

/* ================================================================ */
/*  Funkcje pomocnicze                                               */
/* ================================================================ */

static double clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void set_speed(double left, double right) {
  left  = clamp(left,  -MAX_SPEED, MAX_SPEED);
  right = clamp(right, -MAX_SPEED, MAX_SPEED);
  wb_motor_set_velocity(left_motor,  left);
  wb_motor_set_velocity(right_motor, right);
}

static bool is_green(int r, int g, int b) {
  return g > 80 && g > r * 1.5 && g > b * 1.5;
}

static double normalize_angle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
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
  return atan2(m[3], m[0]);
}

static bool is_front_blocked_now(double v[8]) {
  double front_max  = v[0] > v[7] ? v[0] : v[7];
  double front_diag = v[1] > v[6] ? v[1] : v[6];
  double front_val  = front_max > front_diag ? front_max : front_diag;
  return front_val > WALL_ON;
}

/* ================================================================ */
/*  Funkcje resetujace                                               */
/* ================================================================ */

static void reset_navigation_state(void) {
  front_blocked    = false;
  avoid_steps_left = 0;
  avoid_direction  = 0;
}

static void reset_stuck_counters(void) {
  stuck_check_counter = 0;
  no_progress_counter = 0;

  const double *pos = wb_supervisor_node_get_position(my_node);
  last_leader_x = pos[0];
  last_leader_y = pos[1];
}

static void reset_loop_detector(void) {
  const double *pos = wb_supervisor_node_get_position(my_node);

  last_yaw_for_loop     = get_robot_yaw(my_node);
  accumulated_left_turn = 0.0;
  loop_start_x          = pos[0];
  loop_start_y          = pos[1];
  loop_start_time       = wb_robot_get_time();
}

static void reset_leader_path(void) {
  WbNodeRef leader_node;

  if (leader_name != NULL && strcmp(leader_name, my_name) == 0)
    leader_node = my_node;
  else
    leader_node = other_node;

  const double *pos = wb_supervisor_node_get_position(leader_node);

  for (int i = 0; i < LEADER_PATH_SIZE; i++) {
    leader_path_x[i] = pos[0];
    leader_path_y[i] = pos[1];
  }

  path_write_idx  = 0;
  follow_read_idx = 0;
  path_count      = 0;
}

/* ================================================================ */
/*  Zmiana lidera                                                    */
/* ================================================================ */

static void swap_leader(void) {
  if (strcmp(leader_name, "e-puck") == 0)
    leader_name = "e-puck(1)";
  else
    leader_name = "e-puck";

  reset_stuck_counters();
  reset_loop_detector();
  reset_leader_path();
  reset_navigation_state();

  printf("[%s] Zmiana lidera. Nowy lider: %s\n", my_name, leader_name);
}

/* ================================================================ */
/*  Komunikacja radiowa (emitter / receiver)                         */
/* ================================================================ */

static void send_radio(const char *type, const char *value) {
  char msg[128];

  if (value != NULL)
    snprintf(msg, sizeof(msg), "%s %s %s", my_name, type, value);
  else
    snprintf(msg, sizeof(msg), "%s %s", my_name, type);

  wb_emitter_send(emitter, msg, strlen(msg) + 1);
  printf("[%s] RADIO TX: %s\n", my_name, msg);
}

static void process_radio(Mode *mode) {
  while (wb_receiver_get_queue_length(receiver) > 0) {
    const char *data = (const char *)wb_receiver_get_data(receiver);

    char sender[32], type[32], value[32];
    sender[0] = type[0] = value[0] = '\0';
    sscanf(data, "%31s %31s %31s", sender, type, value);

    /* Ignoruj wlasne echo. */
    if (strcmp(sender, my_name) != 0) {
      printf("[%s] RADIO RX: %s\n", my_name, data);

      if (strcmp(type, "LEADER") == 0) {
        /*
          Potwierdzenie spotkania od drugiego robota.
          Jesli jeszcze jestesmy w SEARCH — przejdz do zespolu.
        */
        if (*mode == MODE_SEARCH &&
            (strcmp(value, "e-puck") == 0 ||
             strcmp(value, "e-puck(1)") == 0)) {
          leader_name     = (strcmp(value, "e-puck") == 0)
                            ? "e-puck" : "e-puck(1)";
          team_start_time = wb_robot_get_time();

          reset_navigation_state();
          reset_stuck_counters();
          reset_loop_detector();
          reset_leader_path();

          if (strcmp(my_name, leader_name) == 0) {
            *mode = MODE_LEADER;
            printf("[%s] RADIO: Spotkanie. Jestem liderem.\n", my_name);
          } else {
            *mode = MODE_FOLLOWER;
            printf("[%s] RADIO: Spotkanie. Podazam za: %s\n",
                   my_name, leader_name);
          }
        }
      }

      else if (strcmp(type, "STUCK") == 0) {
        /*
          Lider zglosil utknięcie — zamiana rol.
          Reagujemy tylko jesli jestesmy followerem.
        */
        if (*mode == MODE_FOLLOWER) {
          printf("[%s] RADIO: lider utknal. Zmiana rol.\n", my_name);

          swap_leader();
          team_start_time = wb_robot_get_time();

          if (strcmp(my_name, leader_name) == 0) {
            *mode = MODE_LEADER;
            printf("[%s] Teraz ja jestem liderem.\n", my_name);
          } else {
            *mode = MODE_FOLLOWER;
          }
        }
      }

      else if (strcmp(type, "GOAL") == 0) {
        printf("[%s] RADIO: Drugi robot osiagnal cel!\n", my_name);
        set_speed(0.0, 0.0);
        goal_reached = true;
      }
    }

    wb_receiver_next_packet(receiver);
  }
}

/* ================================================================ */
/*  Detekcja utknięcia / okrazenia lidera                            */
/* ================================================================ */

static bool leader_is_stuck(double v[8]) {
  const double *pos = wb_supervisor_node_get_position(my_node);

  stuck_check_counter++;

  if (stuck_check_counter < STUCK_CHECK_STEPS)
    return false;

  stuck_check_counter = 0;

  double dx   = pos[0] - last_leader_x;
  double dy   = pos[1] - last_leader_y;
  double move = sqrt(dx * dx + dy * dy);

  last_leader_x = pos[0];
  last_leader_y = pos[1];

  bool front = is_front_blocked_now(v);
  bool side_or_front =
    v[0] > WALL_ON || v[1] > WALL_ON || v[2] > WALL_ON ||
    v[5] > WALL_ON || v[6] > WALL_ON || v[7] > WALL_ON;

  if (move < STUCK_MOVE_EPS && (front || side_or_front))
    no_progress_counter++;
  else
    no_progress_counter = 0;

  if (no_progress_counter >= DEAD_END_STEPS) {
    no_progress_counter = 0;
    return true;
  }

  return false;
}

static bool leader_made_counterclockwise_loop(void) {
  const double *pos = wb_supervisor_node_get_position(my_node);

  double current_yaw = get_robot_yaw(my_node);
  double delta       = normalize_angle(current_yaw - last_yaw_for_loop);

  last_yaw_for_loop = current_yaw;

  if (delta > 0.0)
    accumulated_left_turn += delta;

  double dx   = pos[0] - loop_start_x;
  double dy   = pos[1] - loop_start_y;
  double dist = sqrt(dx * dx + dy * dy);

  double elapsed = wb_robot_get_time() - loop_start_time;

  if (dist > LOOP_RESET_DISTANCE) {
    reset_loop_detector();
    return false;
  }

  if (elapsed > LOOP_MIN_TIME && accumulated_left_turn > LOOP_TURN_LIMIT)
    return true;

  return false;
}

/* ================================================================ */
/*  Logika lidera / samodzielnego szukania celu                      */
/* ================================================================ */

static bool search_or_leader_step(double v[8]) {
  const unsigned char *image = wb_camera_get_image(cam);

  int green_count = 0;
  int green_x_sum = 0;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int r = wb_camera_image_get_red(image,   width, x, y);
      int g = wb_camera_image_get_green(image, width, x, y);
      int b = wb_camera_image_get_blue(image,  width, x, y);

      if (is_green(r, g, b)) {
        green_count++;
        green_x_sum += x;
      }
    }
  }

  /* ── Cel widoczny ── */
  if (green_count >= GREEN_PIXEL_MIN) {
    int green_cx = green_x_sum / green_count;
    int img_cx   = width / 2;
    int offset   = green_cx - img_cx;

    bool touching = v[0] > 800.0 || v[7] > 800.0;

    if (touching || green_count >= GREEN_PIXEL_CLOSE) {
      set_speed(0.0, 0.0);
      printf("[%s] >>> CEL OSIAGNIETY! <<<\n", my_name);
      return true;
    }

    /* Omijanie sciany przy widocznym celu */
    double front_max  = v[0] > v[7] ? v[0] : v[7];
    double front_diag = v[1] > v[6] ? v[1] : v[6];
    double front_val  = front_max > front_diag ? front_max : front_diag;

    if (!front_blocked && front_val > WALL_ON) {
      front_blocked   = true;
      avoid_direction = (offset < 0) ? -1 : 1;
      avoid_steps_left = AVOID_STEPS;
    } else if (front_blocked && front_val < WALL_OFF) {
      front_blocked   = false;
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
      if (approach < 0.25) approach = 0.25;

      double spd        = BASE_SPEED * approach;
      double steer      = (double)offset / (double)img_cx;
      double correction = steer * spd * 0.6;

      set_speed(spd + correction, spd - correction);
    }

    return false;
  }

  /* ── Celu nie widac — jazda prawa sciana ── */
  double front_val = v[0] > v[7] ? v[0] : v[7];
  double diag_val  = v[1] > v[6] ? v[1] : v[6];
  double fv        = front_val > diag_val ? front_val : diag_val;

  if (!front_blocked && fv > WALL_ON)  front_blocked = true;
  if ( front_blocked && fv < WALL_OFF) front_blocked = false;

  double left_speed  = BASE_SPEED;
  double right_speed = BASE_SPEED;

  if (front_blocked) {
    /* Sciana z przodu — skrec w lewo */
    left_speed  = -BASE_SPEED * 0.6;
    right_speed =  BASE_SPEED * 0.6;
  } else if (v[2] > WALL_ON * 1.5) {
    /* Za blisko prawej sciany — lekko w lewo */
    left_speed  = BASE_SPEED * 0.8;
    right_speed = BASE_SPEED;
  } else if (v[2] > WALL_ON) {
    /* Dobry kontakt z prawa sciana — prosto */
    left_speed  = BASE_SPEED;
    right_speed = BASE_SPEED;
  } else {
    /* Brak prawej sciany — skrec w prawo by ja znalezc */
    left_speed  = BASE_SPEED;
    right_speed = BASE_SPEED * 0.4;
  }

  set_speed(left_speed, right_speed);
  return false;
}

/* ================================================================ */
/*  Zapis pozycji lidera do bufora kolowego                          */
/* ================================================================ */

static void record_leader_position(void) {
  WbNodeRef leader_node;

  if (leader_name != NULL && strcmp(leader_name, my_name) == 0)
    leader_node = my_node;
  else
    leader_node = other_node;

  const double *pos = wb_supervisor_node_get_position(leader_node);

  leader_path_x[path_write_idx] = pos[0];
  leader_path_y[path_write_idx] = pos[1];

  path_write_idx = (path_write_idx + 1) % LEADER_PATH_SIZE;

  if (path_count < LEADER_PATH_SIZE)
    path_count++;
}

/* ================================================================ */
/*  Logika followera — sekwencyjne sledzenie waypointow              */
/* ================================================================ */

static void follower_step(double v[8]) {
  WbNodeRef leader_node;

  if (strcmp(leader_name, my_name) == 0)
    leader_node = my_node;
  else
    leader_node = other_node;

  const double *my_pos     = wb_supervisor_node_get_position(my_node);
  const double *leader_pos = wb_supervisor_node_get_position(leader_node);

  double ldx         = leader_pos[0] - my_pos[0];
  double ldy         = leader_pos[1] - my_pos[1];
  double leader_dist = sqrt(ldx * ldx + ldy * ldy);

  /* Czekaj na opoznienie formacji po spotkaniu / zamianie */
  double time_after = wb_robot_get_time() - team_start_time;

  if (time_after < FORMATION_DELAY) {
    set_speed(0.0, 0.0);
    return;
  }

  /* Anti-push: cofnij sie jesli za blisko lidera */
  if (leader_dist < TOO_CLOSE_DISTANCE) {
    set_speed(-BASE_SPEED * 0.15, -BASE_SPEED * 0.15);
    return;
  }

  /* Unikanie scian — nadrzedne wzgledem waypointow */
  bool front_wall = v[0] > WALL_ON || v[7] > WALL_ON ||
                    v[1] > WALL_ON * 1.3 || v[6] > WALL_ON * 1.3;

  if (front_wall) {
    /* Skrec w strone z wieksza przestrzenia */
    double right_blocked = v[1] + v[2];
    double left_blocked  = v[5] + v[6];

    if (right_blocked > left_blocked)
      set_speed(-BASE_SPEED * 0.3, BASE_SPEED * 0.3);   /* lewo */
    else
      set_speed(BASE_SPEED * 0.3, -BASE_SPEED * 0.3);   /* prawo */

    return;
  }

  /* Pobierz biezacy waypoint docelowy */
  double tx = leader_path_x[follow_read_idx];
  double ty = leader_path_y[follow_read_idx];
  double dx = tx - my_pos[0];
  double dy = ty - my_pos[1];
  double target_dist = sqrt(dx * dx + dy * dy);

  /* Awansuj przez osiagniete waypointy */
  int advances = 0;

  while (target_dist < WAYPOINT_REACHED_DIST && advances < 10) {
    int next = (follow_read_idx + 1) % LEADER_PATH_SIZE;

    /* Zachowaj minimalny odstep od wskaznika zapisu */
    int gap = (path_write_idx - next + LEADER_PATH_SIZE) % LEADER_PATH_SIZE;

    if (gap < MIN_FOLLOW_GAP)
      break;

    follow_read_idx = next;
    tx = leader_path_x[follow_read_idx];
    ty = leader_path_y[follow_read_idx];
    dx = tx - my_pos[0];
    dy = ty - my_pos[1];
    target_dist = sqrt(dx * dx + dy * dy);
    advances++;
  }

  /* Wszystkie waypointy osiagniete — czekaj */
  if (target_dist < WAYPOINT_REACHED_DIST) {
    set_speed(0.0, 0.0);
    return;
  }

  /* Kieruj sie ku waypointowi */
  double target_angle = atan2(dy, dx);
  double my_yaw       = get_robot_yaw(my_node);
  double error        = normalize_angle(target_angle - my_yaw);

  /* Predkosc zalezy od dystansu do lidera — im dalej, tym szybciej */
  double speed;

  if (leader_dist > CATCH_UP_DISTANCE)
    speed = BASE_SPEED * 0.95;          /* daleko — doganiaj pelna predkoscia */
  else if (leader_dist > FOLLOW_DISTANCE)
    speed = BASE_SPEED * 0.70;          /* normalna jazda za liderem */
  else
    speed = BASE_SPEED * 0.30;          /* blisko — zwolnij */

  /* Ostry skret — obroc sie w miejscu */
  if (fabs(error) > 1.0) {
    double turn = clamp(error * 1.0, -1.5, 1.5);
    set_speed(-turn, turn);
    return;
  }

  /* Sterowanie proporcjonalne */
  double turn = clamp(error * 1.5, -1.5, 1.5);
  set_speed(speed - turn, speed + turn);
}

/* ================================================================ */
/*  Sprawdzenie celu dla followera                                   */
/* ================================================================ */

static bool is_goal_reached(double v[8]) {
  const unsigned char *image = wb_camera_get_image(cam);
  int green_count = 0;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int r = wb_camera_image_get_red(image,   width, x, y);
      int g = wb_camera_image_get_green(image, width, x, y);
      int b = wb_camera_image_get_blue(image,  width, x, y);

      if (is_green(r, g, b))
        green_count++;
    }
  }

  /* Cel musi byc widoczny (zielone piksele) — sam dotyk sensora
     nie wystarczy, bo moze to byc drugi robot a nie cel. */
  if (green_count < GREEN_PIXEL_MIN)
    return false;

  bool touching = v[0] > 800.0 || v[7] > 800.0;
  return touching && green_count >= GREEN_PIXEL_CLOSE;
}

/* ================================================================ */
/*  Program glowny                                                   */
/* ================================================================ */

int main(void) {
  wb_robot_init();

  my_name = wb_robot_get_name();

  if (strcmp(my_name, "e-puck") == 0)
    other_name = "e-puck(1)";
  else
    other_name = "e-puck";

  /*
    W pliku .wbt:
    DEF ROBOT_A E-puck { name "e-puck"   ... supervisor TRUE }
    DEF ROBOT_B E-puck { name "e-puck(1)" ... supervisor TRUE }
  */
  if (strcmp(my_name, "e-puck") == 0) {
    my_node    = wb_supervisor_node_get_from_def("ROBOT_A");
    other_node = wb_supervisor_node_get_from_def("ROBOT_B");
  } else {
    my_node    = wb_supervisor_node_get_from_def("ROBOT_B");
    other_node = wb_supervisor_node_get_from_def("ROBOT_A");
  }

  if (my_node == NULL || other_node == NULL) {
    printf("[%s] BLAD: Nie moge znalezc robotow przez DEF.\n", my_name);
    printf("Dodaj w .wbt: DEF ROBOT_A i DEF ROBOT_B przy robotach.\n");
    wb_robot_cleanup();
    return 1;
  }

  /* Silniki */
  left_motor  = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor,  INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  set_speed(0.0, 0.0);

  /* Czujniki odleglosci */
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  for (int i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  /* Kamera */
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);

  width  = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam);

  /* Radio (emitter + receiver — wbudowane w E-puck proto, kanal 1) */
  emitter  = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  printf("[%s] Start. Drugi robot: %s\n", my_name, other_name);

  Mode mode = MODE_SEARCH;

  /* ── Petla glowna ── */

  while (wb_robot_step(TIME_STEP) != -1) {
    double v[8];

    for (int i = 0; i < 8; i++)
      v[i] = wb_distance_sensor_get_value(ps[i]);

    /* Odbierz wiadomosci radiowe */
    process_radio(&mode);

    if (goal_reached)
      break;

    /* Zapisz pozycje lidera gdy jestesmy w zespole */
    if (mode == MODE_LEADER || mode == MODE_FOLLOWER)
      record_leader_position();

    /* ── Detekcja spotkania (MODE_SEARCH) ── */
    if (mode == MODE_SEARCH) {
      double d = distance_between_robots();

      if (d < MEET_DISTANCE)
        meet_confirm_counter++;
      else
        meet_confirm_counter = 0;

      if (meet_confirm_counter >= MEET_CONFIRM_STEPS) {
        meet_confirm_counter = 0;

        /* Deterministyczny wybor lidera — zawsze "e-puck" */
        leader_name     = "e-puck";
        team_start_time = wb_robot_get_time();

        reset_navigation_state();
        reset_stuck_counters();
        reset_loop_detector();
        reset_leader_path();

        send_radio("LEADER", leader_name);

        if (strcmp(my_name, leader_name) == 0) {
          mode = MODE_LEADER;
          printf("[%s] Spotkanie. Zostalem liderem.\n", my_name);
        } else {
          mode = MODE_FOLLOWER;
          printf("[%s] Spotkanie. Podazam za liderem: %s\n",
                 my_name, leader_name);
        }
      }
    }

    /* ── MODE_SEARCH ── */
    if (mode == MODE_SEARCH) {
      bool reached = search_or_leader_step(v);

      if (reached)
        break;
    }

    /* ── MODE_LEADER ── */
    else if (mode == MODE_LEADER) {
      double time_after = wb_robot_get_time() - team_start_time;

      bool stuck_pos  = false;
      bool stuck_loop = false;

      if (time_after > LEADER_STUCK_GRACE_TIME) {
        stuck_pos  = leader_is_stuck(v);
        stuck_loop = leader_made_counterclockwise_loop();
      }

      if (stuck_pos || stuck_loop) {
        if (stuck_loop)
          printf("[%s] Wykryto okrazenie — slepy zaulek.\n", my_name);
        else
          printf("[%s] Wykryto brak postepu.\n", my_name);

        printf("[%s] UTKNALEM — wysylam STUCK.\n", my_name);

        send_radio("STUCK", my_name);
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

      if (reached) {
        send_radio("GOAL", my_name);
        break;
      }
    }

    /* ── MODE_FOLLOWER ── */
    else if (mode == MODE_FOLLOWER) {
      /* Follower tez sprawdza kamere — moze sam dojsc do celu */
      if (is_goal_reached(v)) {
        set_speed(0.0, 0.0);
        printf("[%s] >>> CEL OSIAGNIETY (follower)! <<<\n", my_name);
        send_radio("GOAL", my_name);
        break;
      }

      follower_step(v);
    }
  }

  wb_robot_cleanup();
  return 0;
}
