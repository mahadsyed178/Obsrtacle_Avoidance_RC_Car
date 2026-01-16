



// const int sensorPin = A0;  // DO connected to A0 (analog pin used as digital)
// const int ledPin = 13;     // Built-in LED

// void setup() {
//   pinMode(sensorPin, INPUT);
//   pinMode(ledPin, OUTPUT);
//   Serial.begin(9600);
// }

// void loop() {
//   int state = digitalRead(sensorPin); // Read HIGH or LOW

//   if (state == LOW) {  // LOW usually means "obstacle detected"
//     digitalWrite(ledPin, HIGH);
//     Serial.println("Obstacle Detected!");
//   } else {
//     digitalWrite(ledPin, LOW);
//     Serial.println("Clear");
//   }

//   delay(100);
// }







// #include <Servo.h>

// Servo myServo;  // create a servo object

// void setup() {
//   myServo.attach(9); // attach servo to pin 9
// }

// void loop() {
//   // Move from 0° to 90°
//   myServo.write(0);   // go to 0 degrees
//   delay(4000);        // wait 1 second
//   myServo.write(90);  // go to 90 degrees
//   delay(4000);        // wait 1 second
// }







// HC-SR04 Ultrasonic Sensor with Serial Monitor
// Trig -> A0
// Echo -> 2

// #define TRIG_PIN A0
// #define ECHO_PIN 2

// long duration;
// float distance;

// void setup() {
//   pinMode(TRIG_PIN, OUTPUT);
//   pinMode(ECHO_PIN, INPUT);
//   Serial.begin(9600);
// }

// void loop() {
//   // Send trigger pulse
//   digitalWrite(TRIG_PIN, LOW);
//   delayMicroseconds(2);
//   digitalWrite(TRIG_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIG_PIN, LOW);

//   // Read echo time
//   duration = pulseIn(ECHO_PIN, HIGH);

//   // Calculate distance in cm
//   distance = duration * 0.0343 / 2;

//   // Print to Serial Monitor
//   Serial.print("Distance: ");
//   Serial.print(distance);
//   Serial.println(" cm");

//   // Object detection threshold
//   if (distance > 0 && distance < 20) { // adjust 20 cm as needed
//     Serial.println("Object detected!");
//   }

//   delay(500);
// }











// #include <Servo.h>

// // Pin definitions
// #define TRIG_PIN A0
// #define ECHO_PIN A1
// #define SERVO_PIN 11

// Servo myServo;
// long duration;
// float distance;

// void setup() {
//   Serial.begin(9600);
//   pinMode(TRIG_PIN, OUTPUT);
//   pinMode(ECHO_PIN, INPUT);
//   myServo.attach(SERVO_PIN);
//   myServo.write(0); // start at 0 degrees
// }

// void loop() {
//   // Send trigger pulse
//   digitalWrite(TRIG_PIN, LOW);
//   delayMicroseconds(2);
//   digitalWrite(TRIG_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIG_PIN, LOW);

//   // Read echo time
//   duration = pulseIn(ECHO_PIN, HIGH);

//   // Calculate distance (cm)
//   distance = duration * 0.0343 / 2;

//   // Print to Serial Monitor
//   Serial.print("Distance: ");
//   Serial.print(distance);
//   Serial.println(" cm");

//   // If obstacle detected within 20 cm, move servo
//   if (distance > 0 && distance < 20) {
//     Serial.println("Obstacle detected! Moving servo...");
//     myServo.write(90);  // move to 90 degrees
//     delay(1000);
//     myServo.write(0);   // return to 0 degrees
//     delay(1000);
//   }

//   delay(200); // small delay before next reading
// }




// #include <Servo.h>

// // Motor driver pins
// #define ENA 2  // Motor A enable
// #define IN1 3
// #define IN2 4
// #define ENB 5  // Motor B enable
// #define IN3 6
// #define IN4 7

// // Ultrasonic pins
// #define TRIG_PIN A0
// #define ECHO_PIN A1

// // Servo pin
// #define SERVO_PIN 11

// Servo scanServo;
// long duration;
// float distance;

// // --- Motor Control Functions ---
// void forward() {
//   digitalWrite(IN1, HIGH);
//   digitalWrite(IN2, LOW);
//   digitalWrite(IN3, HIGH);
//   digitalWrite(IN4, LOW);
// }

// void backward() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);
//   digitalWrite(IN3, LOW);
//   digitalWrite(IN4, HIGH);
// }

// void left() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);
//   digitalWrite(IN3, HIGH);
//   digitalWrite(IN4, LOW);
// }

// void right() {
//   digitalWrite(IN1, HIGH);
//   digitalWrite(IN2, LOW);
//   digitalWrite(IN3, LOW);
//   digitalWrite(IN4, HIGH);
// }

// void stopCar() {
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, LOW);
//   digitalWrite(IN3, LOW);
//   digitalWrite(IN4, LOW);
// }

// // --- Ultrasonic Distance Function ---
// float getDistance() {
//   digitalWrite(TRIG_PIN, LOW);
//   delayMicroseconds(2);
//   digitalWrite(TRIG_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIG_PIN, LOW);

//   duration = pulseIn(ECHO_PIN, HIGH, 20000); // timeout 20ms
//   return duration * 0.0343 / 2; // cm
// }

// void setup() {
//   Serial.begin(9600);

//   // Motor pins
//   pinMode(ENA, OUTPUT);
//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
//   pinMode(ENB, OUTPUT);
//   pinMode(IN3, OUTPUT);
//   pinMode(IN4, OUTPUT);

//   // Enable motors (PWM full speed)
//   analogWrite(ENA, 200); // speed control
//   analogWrite(ENB, 200);

//   // Ultrasonic pins
//   pinMode(TRIG_PIN, OUTPUT);
//   pinMode(ECHO_PIN, INPUT);

//   // Servo setup
//   scanServo.attach(SERVO_PIN);
//   scanServo.write(90); // center
//   delay(500);
// }

// void loop() {
//   float frontDist = getDistance();
//   Serial.print("Front Distance: ");
//   Serial.println(frontDist);

//   if (frontDist > 20 || frontDist == 0) { 
//     forward();
//   } else {
//     stopCar();
//     delay(200);

//     // Scan right
//     scanServo.write(0);
//     delay(500);
//     float rightDist = getDistance();
//     Serial.print("Right Distance: ");
//     Serial.println(rightDist);

//     // Scan left
//     scanServo.write(180);
//     delay(500);
//     float leftDist = getDistance();
//     Serial.print("Left Distance: ");
//     Serial.println(leftDist);

//     // Return servo to center
//     scanServo.write(90);
//     delay(200);

//     // Decide direction
//     if (rightDist > leftDist) {
//       right();
//       delay(500);
//     } else {
//       left();
//       delay(500);
//     }
//   }
// }



#include <Servo.h>

// Motor driver pins (L298N)
#define ENA 6    // PWM speed control for left motor
#define IN1 2
#define IN2 3
#define ENB 5    // PWM speed control for right motor
#define IN3 4
#define IN4 7

// Ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 11

// Servo pin
#define SERVO_PIN 9

Servo scanServo;
long duration;
float distance;
int motorSpeed = 130; // Adjust 0–255 for speed

// --- Motor Control Functions ---
void forward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void left() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// --- Distance Measurement ---
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 20000); // timeout after 20 ms
  if (duration == 0) return 999; // No reading → assume far
  return duration * 0.0343 / 2; // cm
}

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo setup
  scanServo.attach(SERVO_PIN);
  scanServo.write(90); // Center position
  delay(500);
}

void loop() {
  float frontDist = getDistance();
  Serial.print("Front: ");
  Serial.println(frontDist);

  if (frontDist > 20) {
    forward();
  } else {
    stopCar();
    delay(200);

    // Look right
    scanServo.write(30);
    delay(500);
    float rightDist = getDistance();
    Serial.print("Right: ");
    Serial.println(rightDist);

    // Look left
    scanServo.write(150);
    delay(500);
    float leftDist = getDistance();
    Serial.print("Left: ");
    Serial.println(leftDist);

    // Return to center
    scanServo.write(90);
    delay(200);

    // Decide turn
    if (rightDist > leftDist) {
      right();
      delay(500);
    } else {
      left();
      delay(500);
    }
  }
}




