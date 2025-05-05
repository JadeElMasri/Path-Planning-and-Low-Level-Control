#include <Arduino.h>

#define R_IS  8     
#define REN   9
#define PWM_R 10     
#define LEN   12
#define L_IS  13

#define L_IS_two  0 
#define REN_two   1
#define PWM_L  2     
#define LEN_two  4
#define L_IS_two_2  5

#define R_IS_two  6  
#define PWM_R_two 7  

#define PWM_L_two 3  

#define ENCODER_A 20  
#define ENCODER_B 21  

volatile unsigned long pulseCount1 = 0; 
volatile unsigned long pulseCount2 = 0;  

unsigned long previousMillis = 0;
unsigned long startTime = 0;
bool timerStarted = false;
const long interval = 100; 
bool stepApplied = false;
int step_speed = 90;  

float Kp_r = 0.05, Ki_r = 0.036, Kd_r = 0.04;  
float error_r, last_error_r = 0;
float integral_r = 0;
float derivative_r;
float target_rpm = 28;  
int pwm_r = 0;  

// PID Parameters for Left Motor
float Kp_l = 0.07, Ki_l = 0.05, Kd_l = 0.02;  
float error_l, last_error_l = 0;
float integral_l = 0;
float derivative_l;
int pwm_l = 0;  

void countPulse1() { pulseCount1++; }  // Left Wheel
void countPulse2() { pulseCount2++; }  // Right Wheel

void motorControl(char command) {
    switch (command) {
        case 'w':  // Move forward (Both Motors)
            if (!stepApplied) {
                Serial.println("Activating Both Motors...");
                stepApplied = true;
                startTime = millis();  
                timerStarted = true;
                pwm_r = step_speed;
                pwm_l = step_speed;

                digitalWrite(R_IS, HIGH);
                digitalWrite(L_IS, LOW);
                digitalWrite(R_IS_two, LOW);
                digitalWrite(L_IS_two, HIGH);

                analogWrite(PWM_R, pwm_r);
                analogWrite(PWM_L, 0);
                analogWrite(PWM_R_two, 0);
                analogWrite(PWM_L_two, pwm_l);
            }
            break;

        case 'x':  // Stop motors
            stepApplied = false;
            timerStarted = false;  
            digitalWrite(R_IS, LOW);
            digitalWrite(L_IS, LOW);
            digitalWrite(R_IS_two, LOW);
            digitalWrite(L_IS_two, LOW);

            analogWrite(PWM_R, 0);
            analogWrite(PWM_L, 0);
            analogWrite(PWM_R_two, 0);
            analogWrite(PWM_L_two, 0);
            pwm_r = 0;
            pwm_l = 0;
            break;
    }
}

void setup() {
    // Set motor control pins as outputs
    pinMode(PWM_R, OUTPUT);
    pinMode(R_IS, OUTPUT);
    pinMode(L_IS, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);

    pinMode(PWM_L, OUTPUT);
    pinMode(L_IS_two, OUTPUT);
    pinMode(L_IS_two_2, OUTPUT);
    pinMode(REN_two, OUTPUT);
    pinMode(LEN_two, OUTPUT);

    pinMode(PWM_R_two, OUTPUT);
    pinMode(R_IS_two, OUTPUT);

    pinMode(PWM_L_two, OUTPUT);
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), countPulse1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), countPulse2, RISING);

    digitalWrite(REN, HIGH);
    digitalWrite(LEN, HIGH);
    digitalWrite(REN_two, HIGH);
    digitalWrite(LEN_two, HIGH);

    Serial.begin(9600);
    Serial.println("Time (s),RPM Left,RPM Right");
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();  
        motorControl(command);
    }

    if (timerStarted) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;

            unsigned long pulseCopy1 = pulseCount1;  // Left motor encoder count
            unsigned long pulseCopy2 = pulseCount2;  // Right motor encoder count

            float rpm1 = (pulseCopy1 * 60) / interval;  // Left RPM
            float rpm2 = (pulseCopy2 * 60) / interval;  // Right RPM

            // PID Control for Right Motor
            error_r = target_rpm - rpm2;
            integral_r += error_r * (interval / 1000.0);
            derivative_r = (error_r - last_error_r) / (interval / 1000.0);
            last_error_r = error_r;
            float correction_r = (Kp_r * error_r) + (Ki_r * integral_r) + (Kd_r * derivative_r);

            // PID Control for Left Motor
            error_l = target_rpm - rpm1;
            integral_l += error_l * (interval / 1000.0);
            derivative_l = (error_l - last_error_l) / (interval / 1000.0);
            last_error_l = error_l;
            float correction_l = (Kp_l * error_l) + (Ki_l * integral_l) + (Kd_l * derivative_l);

            // Apply corrections to PWM values
            pwm_r += correction_r;
            pwm_l += correction_l;

            pwm_r = constrain(pwm_r, 0, 255);
            pwm_l = constrain(pwm_l, 0, 255);

            analogWrite(PWM_R, pwm_r);
            analogWrite(PWM_L, 0);
            analogWrite(PWM_R_two, 0);
            analogWrite(PWM_L_two, pwm_l);

            Serial.print((currentMillis - startTime) / 1000.0);  
            Serial.print(",");
            Serial.print(rpm1);  // Left wheel RPM
            Serial.print(",");
            Serial.println(rpm2);  // Right wheel RPM

            // Reset encoder counts
            pulseCount1 = 0;
            pulseCount2 = 0;
        }
    }
}
