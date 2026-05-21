#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdbool.h>

#define TIME_STEP    64
#define MAX_SPEED    6.28
#define BASE_SPEED   4.0

// Progi ścian z histerezą - dwa progi żeby uniknąć wibracji
#define WALL_ON      150.0   // powyżej tego -> ściana WYKRYTA
#define WALL_OFF      80.0   // poniżej tego -> ściana MINIĘTA (musi spaść niżej niż ON)

// Detekcja celu
#define GREEN_PIXEL_MIN    30
#define GREEN_PIXEL_CLOSE  2000

// Liczba kroków skrętu przy omijaniu ściany w trybie namierzania celu
// Robot skręca przez tyle kroków zanim sprawdzi sensory ponownie
#define AVOID_STEPS  8

static bool is_green(int r, int g, int b) {
  return g > 80 && g > r * 1.5 && g > b * 1.5;
}

int main(void) {
  wb_robot_init();

  WbDeviceTag left_motor  = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor,  INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor,  0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag ps[8];
  char ps_names[8][4] = {"ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7"};
  for (int i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  WbDeviceTag cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, TIME_STEP);
  int width  = wb_camera_get_width(cam);
  int height = wb_camera_get_height(cam);

  // --- Stan histerezowy ścian ---
  // Zapamiętujemy czy ściana była wykryta w poprzednim kroku.
  // Ściana znika dopiero gdy czujnik spadnie PONIŻEJ WALL_OFF (nie WALL_ON),
  // więc drobne drgania sensora przy progu nie powodują przełączania.
  bool front_blocked = false;

  // --- Licznik kroków przymusowego skrętu ---
  // Gdy robot zaczyna omijać ścianę, kontynuuje skręt przez AVOID_STEPS kroków
  // niezależnie od sensorów - eliminuje wibracje przy krawędzi ściany.
  int avoid_steps_left = 0;
  int avoid_direction  = 0;   // -1 = lewo, +1 = prawo

  printf("Start nawigacji reaktywnej...\n");

  while (wb_robot_step(TIME_STEP) != -1) {

    double v[8];
    for (int i = 0; i < 8; i++)
      v[i] = wb_distance_sensor_get_value(ps[i]);

    // --- ANALIZA KAMERY ---
    const unsigned char *image = wb_camera_get_image(cam);
    int green_count = 0;
    int green_x_sum = 0;

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int r = wb_camera_image_get_red  (image, width, x, y);
        int g = wb_camera_image_get_green(image, width, x, y);
        int b = wb_camera_image_get_blue (image, width, x, y);
        if (is_green(r, g, b)) {
          green_count++;
          green_x_sum += x;
        }
      }
    }

    // --- CEL WYKRYTY ---
    if (green_count >= GREEN_PIXEL_MIN) {

      int green_cx = green_x_sum / green_count;
      int img_cx   = width / 2;
      int offset   = green_cx - img_cx;   // ujemny = cel po lewej, dodatni = po prawej

      printf("[cel] pikseli=%d offset=%d ps0=%.0f ps7=%.0f\n",
             green_count, offset, v[0], v[7]);

      // Warunek DOTKNIĘCIA
      bool touching = v[0] > 800.0 || v[7] > 800.0;
      if (touching || green_count >= GREEN_PIXEL_CLOSE) {
        wb_motor_set_velocity(left_motor,  0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        printf(">>> CEL OSIAGNIETY! <<<\n");
        break;
      }

      // --- Aktualizacja histerezy ściany (tylko dla przodu) ---
      double front_max = v[0] > v[7] ? v[0] : v[7];
      double front_diag = v[1] > v[6] ? v[1] : v[6];
      double front_val = front_max > front_diag ? front_max : front_diag;

      if (!front_blocked && front_val > WALL_ON) {
        // Ściana właśnie wykryta - zacznij skręt i zapamiętaj kierunek
        front_blocked    = true;
        avoid_direction  = (offset < 0) ? -1 : 1;  // skręć w stronę celu
        avoid_steps_left = AVOID_STEPS;
        printf("[omijanie] Ściana! Skręcam %s przez %d kroków\n",
               avoid_direction < 0 ? "LEWO" : "PRAWO", AVOID_STEPS);
      } else if (front_blocked && front_val < WALL_OFF) {
        // Ściana minięta (histereza: musi spaść poniżej WALL_OFF)
        front_blocked    = false;
        avoid_steps_left = 0;
        printf("[omijanie] Droga wolna\n");
      }

      if (avoid_steps_left > 0) {
        // Przymusowy skręt - kontynuuj niezależnie od sensorów
        double turn = BASE_SPEED * 0.5;
        if (avoid_direction < 0) {
          wb_motor_set_velocity(left_motor,  -turn);
          wb_motor_set_velocity(right_motor,  turn);
        } else {
          wb_motor_set_velocity(left_motor,   turn);
          wb_motor_set_velocity(right_motor, -turn);
        }
        avoid_steps_left--;

      } else {
        // Droga wolna -> jedź prosto do celu ze stopniowym hamowaniem
        double approach   = 1.0 - ((double)green_count / (double)GREEN_PIXEL_CLOSE);
        if (approach < 0.25) approach = 0.25;
        double spd        = BASE_SPEED * approach;
        double steer      = (double)offset / (double)img_cx;
        double correction = steer * spd * 0.6;
        wb_motor_set_velocity(left_motor,  spd + correction);
        wb_motor_set_velocity(right_motor, spd - correction);
      }

      continue;
    }

    // --- REAKTYWNA LOGIKA LABIRYNTU (prawa ściana) ---
    // Tutaj też używamy histerezy dla stabilności
    double front_val = v[0] > v[7] ? v[0] : v[7];
    double diag_val  = v[1] > v[6] ? v[1] : v[6];
    double fv        = front_val > diag_val ? front_val : diag_val;

    if (!front_blocked && fv > WALL_ON)  front_blocked = true;
    if ( front_blocked && fv < WALL_OFF) front_blocked = false;

    double left_speed  = BASE_SPEED;
    double right_speed = BASE_SPEED;

    bool wall_right = v[2] > WALL_ON;

    if (front_blocked) {
      left_speed  = -BASE_SPEED * 0.6;
      right_speed =  BASE_SPEED * 0.6;
    } else if (wall_right) {
      if (v[2] > WALL_ON * 1.5) {
        left_speed  = BASE_SPEED * 0.8;
        right_speed = BASE_SPEED;
      } else if (v[2] < WALL_ON * 0.8) {
        left_speed  = BASE_SPEED;
        right_speed = BASE_SPEED * 0.8;
      }
    } else {
      left_speed  = BASE_SPEED;
      right_speed = BASE_SPEED * 0.4;
    }

    wb_motor_set_velocity(left_motor,  left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}