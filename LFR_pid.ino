// --- Motor Driver Pins (Example: L298N) ---
const int enA = 9;   // Left motor PWM
const int in1 = 8;   // Left motor Direction 1
const int in2 = 7;   // Left motor Direction 2
const int enB = 3;   // Right motor PWM
const int in3 = 5;   // Right motor Direction 1
const int in4 = 4;   // Right motor Direction 2

// --- Sensor Pins (Example: 5 IR Sensors) ---
const int sensorPins[] = {A0, A1, A2, A3, A4};
int sensorValues[5] = {0, 0, 0, 0, 0};

// --- PID Constants ---
float Kp = 15.0; 
float Ki = 0.0;  
float Kd = 10.0; 

// --- PID Variables ---
int error = 0;
int previousError = 0;
float P = 0, I = 0, D = 0;
float PID_value = 0;

// --- Motor Speeds ---
const int baseSpeed = 150; // Normal forward speed (0-255)
const int maxSpeed = 255;  // Maximum speed limit

void setup() {
  // Motor pin setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Sensor pin setup
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  Serial.begin(9600);
}

void loop() {
  readSensors();
  calculatePID();
  driveMotors();
}

// --- 1. Read Sensors and Calculate Error ---
void readSensors() {
  // Assuming a black line on a white surface (0 = white, 1 = black)
  // You may need to use analogRead() and a threshold depending on your sensors
  int s0 = digitalRead(sensorPins[0]);
  int s1 = digitalRead(sensorPins[1]);
  int s2 = digitalRead(sensorPins[2]);
  int s3 = digitalRead(sensorPins[3]);
  int s4 = digitalRead(sensorPins[4]);

  // Weighted sum to calculate the position error
  // Far left is heavily negative, center is 0, far right is heavily positive
  if (s0==0 && s1==0 && s2==1 && s3==0 && s4==0) error = 0;
  else if (s0==0 && s1==1 && s2==1 && s3==0 && s4==0) error = -1;
  else if (s0==0 && s1==1 && s2==0 && s3==0 && s4==0) error = -2;
  else if (s0==1 && s1==1 && s2==0 && s3==0 && s4==0) error = -3;
  else if (s0==1 && s1==0 && s2==0 && s3==0 && s4==0) error = -4;
  else if (s0==0 && s1==0 && s2==1 && s3==1 && s4==0) error = 1;
  else if (s0==0 && s1==0 && s2==0 && s3==1 && s4==0) error = 2;
  else if (s0==0 && s1==0 && s2==0 && s3==1 && s4==1) error = 3;
  else if (s0==0 && s1==0 && s2==0 && s3==0 && s4==1) error = 4;
  // If line is lost entirely, retain the previous error to keep turning back
}

// --- 2. Calculate PID Output ---
void calculatePID() {
  P = error;
  I = I + error;
  //To avoid integral windup
  I = constrain(I,-50,50);
  D = error - previousError;
  
  // Standard PID Equation
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  
  previousError = error;
}

// --- 3. Adjust Motor Speeds ---
void driveMotors() {
  int leftSpeed = baseSpeed + PID_value;
  int rightSpeed = baseSpeed - PID_value;

  // Clamp speeds to ensure they don't exceed motor limits (0 to maxSpeed)
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Set Left Motor Speed and Direction (Forward)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, leftSpeed);

  // Set Right Motor Speed and Direction (Forward)
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, rightSpeed);
}
