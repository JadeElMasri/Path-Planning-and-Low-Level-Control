#include <ros.h>
#include <geometry_msgs/Twist.h>

#define R_IS  0
#define REN   1
#define PWM_R 2
#define PWM_L 3
#define LEN   4
#define L_IS  5

#define R_IS_two  8
#define REN_two   9
#define PWM_R_two 10
#define PWM_L_two 11
#define LEN_two   12
#define L_IS_two  13

#define ENCODER_A 19  // Encoder for right wheel
#define ENCODER_B 21  // Encoder for left wheel

const int max_pwm = 60;  
const float max_linear_speed = 0.5; 
const float max_angular_speed = 2.0; 

ros::NodeHandle nh;

void moveMotors(float linear_x, float angular_z) {
    int leftSpeed, rightSpeed;

    rightSpeed = (linear_x + angular_z * 0.5) * (max_pwm / max_linear_speed);
    leftSpeed  = (linear_x - angular_z * 0.5) * (max_pwm / max_linear_speed);

    rightSpeed = constrain(rightSpeed, -max_pwm, max_pwm);
    leftSpeed  = constrain(leftSpeed, -max_pwm, max_pwm);

    if (rightSpeed > 0) {
        digitalWrite(R_IS, HIGH);
        digitalWrite(L_IS, LOW);
        analogWrite(PWM_R, rightSpeed);
    } else {
        digitalWrite(R_IS, LOW);
        digitalWrite(L_IS, HIGH);
        analogWrite(PWM_R, -rightSpeed);
    }

    if (leftSpeed > 0) {
        digitalWrite(R_IS_two, HIGH);
        digitalWrite(L_IS_two, LOW);
        analogWrite(PWM_L, leftSpeed);
    } else {
        digitalWrite(R_IS_two, LOW);
        digitalWrite(L_IS_two, HIGH);
        analogWrite(PWM_L, -leftSpeed);
    }
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_msg) {
    float linear_x = cmd_msg.linear.x;
    float angular_z = cmd_msg.angular.z;

    moveMotors(linear_x, angular_z);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmdVelCallback);

void setup() {
    pinMode(PWM_R, OUTPUT);
    pinMode(R_IS, OUTPUT);
    pinMode(PWM_L, OUTPUT);
    pinMode(L_IS, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);

    pinMode(PWM_R_two, OUTPUT);
    pinMode(R_IS_two, OUTPUT);
    pinMode(PWM_L_two, OUTPUT);
    pinMode(L_IS_two, OUTPUT);
    pinMode(REN_two, OUTPUT);
    pinMode(LEN_two, OUTPUT);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    digitalWrite(REN, HIGH);
    digitalWrite(LEN, HIGH);
    digitalWrite(REN_two, HIGH);
    digitalWrite(LEN_two, HIGH);

    nh.initNode();
    nh.subscribe(sub);

    Serial.begin(57600);
    Serial.println("Motor Controller Ready!");
}

void loop() {
    nh.spinOnce();  
    delay(10);
}
