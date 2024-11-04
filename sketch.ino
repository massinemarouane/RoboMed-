/* 
This code controls the RoboMed medication delivery robot simulation. It handles obstacle detection, movement, 
and position tracking using various sensors and actuators. The robot uses an ultrasonic sensor for detecting obstacles 
and a joystick for directional control. A pair of RGB LEDs simulate motor feedback, and an RFID system signals when 
the robot reaches a designated room. Odometry calculations track the robot’s speed, position, and orientation, 
enabling smooth navigation and movement within a defined hospital environment.
*/

//Robot's sepecification and dimensions
const float Rw = 0.075; // Distance between the center of the robot and each wheel (150mm chassis width)
const int pulses = 13; // Pulses per revolution (as per real encoder)
const float wheel_radius = 0.0325; // Radius of wheels
const float wheel_circumference = 2 * 3.14 * wheel_radius; // Wheels' circumeference in (m)
const float length_per_pulse = wheel_circumference / pulses; // Length per pulse in (m/pulse)

//Ultrasonic Sensor
const int trig = 4; // Trigger pin
const int echo = 1; // Echo pin
float ultrasonic_distance; // Distance variable
long ultrasonic_duration; // Ultrasonic wave time duration

//Joystick
const int vert = 11; // Vertical movement of Joystick (VRy)
const int horz = 12; // Horizontal movement of Joystick (VRx)

  
//Buzzer and RFID pins
const int buzzer = 35; // Buzzer pin
const int warningLED = 19; // RFID Indication LED
const int sw = 39; // RFID Tag (Slide Switch)


//DC Motors represented as RGB LEDs
const int left_Motor_Red = 16; // Left RGB LED red channel
const int left_Motor_Green = 17; // Left RGB LED green channel
const int left_Motor_Blue = 18; // Left RGB LED blue channel
const int right_Motor_Red = 2; // Right RGB LED red channel
const int right_Motor_Green = 42; // Right RGB LED green channel
const int right_Motor_Blue = 41; // Right RGB LED blue channel


//Odometry variables
const int sampling_time = 250; // Sampling time in milliseconds
float theta;          // Angle increment from one position to another in radians
float theta_deg;   // Angle increment from one position to another in degrees
float x_pos = 0;      // Robot's x position
float y_pos = 0;      // Robot's y position
unsigned long prevTime = 0; // Last time position/speed was updated
float v_r = 0;       // Speed of right motor
float v_l = 0;       // Speed of left motor
float v = 0;         // Speed at the center of the robot
int rpm_r = 0;       // RPM of right motor
int rpm_l = 0;       // RPM of left motor
float rpm = 0;       // RPM at the center of the robot
float d_r = 0;       // Distance measured from right encoder
float d_l = 0;       // Distance measured from left encoder
float d = 0;         // Distance from the center of the robot
int num_pulses_left = 0;   // Number of pulses left motor
int num_pulses_right = 0;  //  Number of pulses right motor
int prev_pulses_right = 0; // Store pulses from right motor each sampling time
int prev_pulses_left = 0;  // Store pulses from left motor each sampling time
bool isStopped = false;    // Flag to control prints when robot stops


void setup() {
  Serial.begin(115200);

  // Ultrasonic sensor pins
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  // DC Motors pins
  pinMode(left_Motor_Red, OUTPUT);
  pinMode(left_Motor_Green, OUTPUT);
  pinMode(left_Motor_Blue, OUTPUT);
  pinMode(right_Motor_Red, OUTPUT);
  pinMode(right_Motor_Green, OUTPUT);
  pinMode(right_Motor_Blue, OUTPUT);

  // Buzzer pin
  pinMode(buzzer, OUTPUT);

  // RFID pins
  pinMode(warningLED, OUTPUT);
  pinMode(sw, INPUT_PULLUP);

  Serial.println("RoboMed Simulation Begins!");
}

void loop() {
  // Step 1: Ultrasonic Sensing Begins
  checkObstacleAndFinalDestination();

  delay(1000);

  // Step 2: If no obstacle, joystick inputs for direction
  if (!isStopped) {
    handleJoystick();
  }

  // Step 3: Updates and display odometric values every sampling interval
  updatePositionAndSpeed();
}

void checkObstacleAndFinalDestination() {
  /*
    Check for obstacles and if desired destination is reach
  */

  digitalWrite(trig, LOW);
  delayMicroseconds(10);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  ultrasonic_duration = pulseIn(echo, HIGH);
  ultrasonic_distance = (ultrasonic_duration * 0.0343) / 2;

  bool isSwitchPressed = digitalRead(sw) == HIGH;

  if ((ultrasonic_distance > 0 && ultrasonic_distance < 20) || isSwitchPressed) {
    if (!isStopped) {
      digitalWrite(warningLED, HIGH);
      setColor_left(0,  0, 0);
      setColor_right(0,  0, 0);
      digitalWrite(buzzer, HIGH);
      Serial.println("\nRobot Stopped Movement");
      if (isSwitchPressed){
        Serial.println("\nReached Room");
      }
      isStopped = true;
      delay(500); 
    }
  } else {
    if (isStopped) {
      isStopped = false;
      digitalWrite(warningLED, LOW);
      digitalWrite(buzzer, LOW);
      Serial.println("\n\nCleared for Movement");
      delay(500); 
    }
  }
}


void handleJoystick() {
  /*
    Handles joystick inputs for turning and movement
  */

  int xVal = analogRead(horz);
  int yVal = analogRead(vert);

  if (yVal > 3000) { // Move Forward
    Serial.println("\n\nRobot moving forward");
    setColor_left(255,  0, 0);
    setColor_right(255,  0, 0);
    num_pulses_right++;
    num_pulses_left++;
  } else if (yVal < 1000) { // Move Backward
    Serial.println("\n\nRobot moving backward");
    setColor_left(0,  0, 255);
    setColor_right(0,  0, 255);
    num_pulses_right--;
    num_pulses_left--;
  } else if (xVal < 1000) { // Turn Right
    Serial.println("\n\nRobot turning right");
    setColor_left(255,  0, 0);
    setColor_right(0,  0, 255);
    num_pulses_right--;
    num_pulses_left++;
  } else if (xVal > 3000) { // Turn Left
    Serial.println("\n\nRobot turning left");
    setColor_right(255,  0, 0);
    setColor_left(0,  0, 255);
    num_pulses_right++;
    num_pulses_left--;
  } else {
    Serial.println("\n\nNo motion");
    setColor_right(0,  0, 0);
    setColor_left(0,  0, 0);
  }
}


void updatePositionAndSpeed() {
  /*
    Updates robot's position and speed
  */
  
  unsigned long currentTime = millis();
  if (currentTime - prevTime >= sampling_time) 
  {
    d_r = length_per_pulse * num_pulses_right;
    d_l = length_per_pulse * num_pulses_left;
    d = abs(d_r + d_l) / 2;
    float d_temp = (d_r + d_l) / 2;

    int delta_pulses_right = num_pulses_right - prev_pulses_right;
    int delta_pulses_left = num_pulses_left - prev_pulses_left;
    float d_r_increment;
    float d_l_increment;

    d_r_increment = length_per_pulse * delta_pulses_right;
    d_l_increment = length_per_pulse * delta_pulses_left;

    theta += (d_r_increment - d_l_increment) / (2 * Rw);
    theta_deg = (theta * 180) / 3.14;
    x_pos = d_temp * cos(theta);
    y_pos = d_temp * sin(theta);

    v_r = (d_r - prev_pulses_right * length_per_pulse) / (sampling_time * 0.001);
    v_l = (d_l - prev_pulses_left * length_per_pulse) / (sampling_time * 0.001);
    v = abs(v_r + v_l) / 2;

    rpm_r = (delta_pulses_right * 60 * 1000) / (pulses * sampling_time);
    rpm_l = (delta_pulses_left * 60 * 1000) / (pulses * sampling_time);
    rpm = (rpm_r + rpm_l) / 2;

    prev_pulses_right = num_pulses_right;
    prev_pulses_left = num_pulses_left;
    prevTime = currentTime;

    Serial.print("\n\n(X,Y,Distance): ");
    Serial.print(x_pos);
    Serial.print(", ");
    Serial.print(y_pos);
    Serial.print(", ");
    Serial.println(d);
    Serial.print("Speed (m/s): ");
    Serial.print(v);
    Serial.print(", Speed (RPM): ");
    Serial.print(rpm);
    Serial.print(", Heading Angle: ");
    Serial.print(theta_deg);
    delay(sampling_time); 
  }
}

void setColor_left(int redValue, int greenValue,  int blueValue) {
  /*
    Sets left RGB LED color
  */

  analogWrite(left_Motor_Red, redValue);
  analogWrite(left_Motor_Green,  greenValue);
  analogWrite(left_Motor_Blue, blueValue);
}
void setColor_right(int redValue, int greenValue,  int blueValue) {
  /*
    Sets right RGB LED color
  */
  
  analogWrite(right_Motor_Red, redValue);
  analogWrite(right_Motor_Green,  greenValue);
  analogWrite(right_Motor_Blue, blueValue);
}
