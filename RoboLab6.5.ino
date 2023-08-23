#include "ESPMax.h"
#include "Buzzer.h"
#include "Ultrasound.h"
#include "ESP32PWMServo.h"
#include "SuctionNozzle.h"
#include "Arduino_APDS9960.h"

// Color Sorting using Robotic Arm

Ultrasound ultrasound;  // Instantiate the Ultrasound class

#define RED   1
#define GREEN 2
#define BLUE  3

int r_f = 30;
int g_f = 50;
int b_f = 50;
int R_F = 3000;
int G_F = 2600;
int B_F = 3500;

int red_locations[3][4] ={{-155, -208 ,115 , 1500 },
                            {-120, -250 ,120 , 2400},
                            {-146, -230 ,170 , 1400}};

int blue_locations[3][4] = {{-128 , -177 ,115 , 1500 },
                            {-81, -203 , 120 , 2400},
                            {-104.5, -190.5 ,140 , 1400}};

int green_locations[3][4] = {{- 90 , -142 ,115 , 1500 },
                            {-45, -162.5 , 120 , 2400},
                            {-73.5, -151.5 ,140 , 1400}};


int red_count = 0;
int blue_count = 0;
int green_count = 0;
// Function to Detect Color
int ColorDetect() {
  // Initialize color sensor and wait for color data
  while (!APDS.colorAvailable()) delay(5);
  
  // Variables to store RGB color values and the detected color
  int r, g, b, c;
  APDS.readColor(r, g, b);
  
  // Scale the color values to a range of 0 to 255
  r = map(r, r_f, R_F, 0, 255);
  g = map(g, g_f, G_F, 0, 255);
  b = map(b, b_f, B_F, 0, 255);

  // Determine the color based on the highest color value (e.g., red if r is the highest)
  if (r > g) c = RED;
  else c = GREEN;
  if (c == GREEN && g < b) c = BLUE;
  if (c == RED && r < b) c = BLUE;

  // If the color value is greater than 50, return the detected color; otherwise, return 0
  if (c == BLUE && b > 60) return c;
  else if (c == GREEN && g > 60) return c;
  else if (c == RED && r > 60) return c;
  else return 0;
}

void setup() {
  // Initialize components and set initial positions
  Buzzer_init();
  ESPMax_init();
  Nozzle_init();
  PWMServo_init();
  Valve_on();
  go_home(2000);
  Valve_off();
  delay(100);
  SetPWMServo(1, 1500, 1000);
  Serial.begin(115200);
  Serial.println("start...");
  
  // Initialize color sensor
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS-9960 sensor.");
  }
  
  // Turn off the RGB lights of the ultrasound module
  ultrasound.Color(255, 255, 255, 255, 255, 255);
}

// Variables to store coordinates and color information
int x, y, z;

// int boxes[gridSize][gridSize] = {{},{},{}};

int angle_pul = 1500;
int detect_color = 0;
bool color_detect = true;

void loop() {
  float pos[3];
  int distance = 0;

  if (color_detect) { // Color detection phase
    if (ColorDetect()) { // Check if the color sensor has detected a color
      // Perform multiple detections and take the average to avoid false positives
      float color_num = 0.0;
      for (int i = 0; i < 5; i++) {
        color_num += ColorDetect();
        delay(80);
      }
      color_num = color_num / 5.0;
      color_detect = false;
      if (color_num == 1.0) {
        // Set coordinates for the red placement area
        x = red_locations[red_count][0];
        y = red_locations[red_count][1];
        z = red_locations[red_count][2];
        angle_pul = red_locations[red_count][3];
        red_count = red_count +1 ;
        detect_color = RED;
        Serial.println("Red");
        ultrasound.Color(255, 0, 0, 255, 0, 0);
      }
      else if (color_num == 2.0) {
        // Set coordinates for the green placement area
        x = green_locations[green_count][0];
        y = green_locations[green_count][1];
        z = green_locations[green_count][2];
        angle_pul = green_locations[green_count][3];
        green_count = green_count + 1;
        detect_color = GREEN;
        Serial.println("Green");
        ultrasound.Color(0, 255, 0, 0, 255, 0);
      }
      else if (color_num == 3.0) {
        // Set coordinates for the blue placement area
        x = blue_locations[blue_count][0];
        y = blue_locations[blue_count][1];
        z = blue_locations[blue_count][2];
        angle_pul = blue_locations[blue_count][3];
        blue_count = blue_count +1 ;
        detect_color = BLUE;
        Serial.println("Blue");
        ultrasound.Color(0, 0, 255, 0, 0, 255);
      }
      else {
        // If the detection result is not an integer, no further action is taken
        detect_color = 0;
        color_detect = true;
        ultrasound.Color(255, 255, 255, 255, 255, 255);
      }
    }
    else {
      // If no color is detected, reset the color detection status and turn off the RGB lights
      if (color_detect) detect_color = 0;
      ultrasound.Color(255, 255, 255, 255, 255, 255);
      delay(200);
    }
  }
  else { // Ultrasonic distance detection phase
    // Perform multiple distance readings and take the average
    for (int i = 0; i < 5; i++) {
      distance += ultrasound.GetDistance();
      delay(100);
    }
    int dis = int(distance / 5);
    Serial.print("Distance:");
    Serial.println(dis);
    
    // Check if the distance falls within the range of 60~80mm
    if (60 < dis && dis < 100) {
      // Check if a color is detected and perform the robotic arm actions accordingly
      if (detect_color) {
        setBuzzer(100);
        delay(1000);
        pos[0] = 0; pos[1] = -160; pos[2] = 200;
        set_position(pos, 1500);
        delay(1500);
        int solid_block_h = 105;
        int arch_block_h = 80;
        int h = 200;
        if (red_count > 2 || blue_count > 2 || green_count > 2) {h = arch_block_h;}
        else {h = solid_block_h;} 
        pos[0] = 0; pos[1] = -160; pos[2] = h;
        set_position(pos, 1000);
        Pump_on();
        delay(1000);
        pos[0] = 0; pos[1] = -160; pos[2] = 210;
        set_position(pos, 1000);
        delay(1000);
        pos[0] = x; pos[1] = y; pos[2] = 210;
        set_position(pos, 1500);
        delay(1000);
        SetPWMServo(1, angle_pul, 1000);
        delay(1500);
        pos[0] = x; pos[1] = y; pos[2] = z;
        set_position(pos, 2000);
        delay(2000);
        pos[0] = x; pos[1] = y; pos[2] = z+30;
        set_position(pos, 2000);
        Valve_on();
        delay(3000);
        setBuzzer(100);
        go_home(1500);
        delay(2000);
        Valve_off();
        SetPWMServo(1, 1500, 800);
        delay(200);
        detect_color = 0;
        color_detect = true;
        if (red_count == 3 ){red_count = 0;}
        if (blue_count == 3 ){blue_count = 0;}
        if (green_count == 3 ){green_count = 0;}
        
        ultrasound.Color(255, 255, 255, 255, 255, 255);
        delay(1500);
      }
    }
  }
}
