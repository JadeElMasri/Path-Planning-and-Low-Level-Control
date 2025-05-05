#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>
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

int speed_value = 100;

ros::NodeHandle nh;

void motorControl(char command) {
    switch (command) {
        case 'w':  
            digitalWrite(R_IS, HIGH); digitalWrite(L_IS, LOW);
            digitalWrite(R_IS_two, LOW); digitalWrite(L_IS_two, HIGH);
            analogWrite(PWM_R, speed_value); analogWrite(PWM_L, 0);
            analogWrite(PWM_R_two, 0); analogWrite(PWM_L_two, speed_value);
            break;
        case 's':  
            digitalWrite(R_IS, LOW); digitalWrite(L_IS, HIGH);
            digitalWrite(R_IS_two, HIGH); digitalWrite(L_IS_two, LOW);
            analogWrite(PWM_R, 0); analogWrite(PWM_L, speed_value);
            analogWrite(PWM_R_two, speed_value); analogWrite(PWM_L_two, 0);
            break;
        case 'a':  
            digitalWrite(R_IS, LOW); digitalWrite(L_IS, HIGH);
            digitalWrite(R_IS_two, LOW); digitalWrite(L_IS_two, HIGH);
            analogWrite(PWM_R, speed_value / 2); analogWrite(PWM_L, 0);
            analogWrite(PWM_R_two, speed_value); analogWrite(PWM_L_two, 0);
            break;
        case 'd':  
            digitalWrite(R_IS, HIGH); digitalWrite(L_IS, LOW);
            digitalWrite(R_IS_two, HIGH); digitalWrite(L_IS_two, LOW);
            analogWrite(PWM_R, 0); analogWrite(PWM_L, speed_value);
            analogWrite(PWM_R_two, 0); analogWrite(PWM_L_two, speed_value / 2);
            break;
        case 'x':  // Stop
            digitalWrite(R_IS, LOW); digitalWrite(L_IS, LOW);
            digitalWrite(R_IS_two, LOW); digitalWrite(L_IS_two, LOW);
            analogWrite(PWM_R, 0); analogWrite(PWM_L, 0);
            analogWrite(PWM_R_two, 0); analogWrite(PWM_L_two, 0);
            break;
    }
}

void commandCallback(const std_msgs::String& msg) {
    if (strlen(msg.data) > 0) {
        char command = msg.data[0];  
        motorControl(command);
    }
}

ros::Subscriber<std_msgs::String> sub("/teleop", &commandCallback);

void setup() {
    pinMode(PWM_R, OUTPUT); pinMode(R_IS, OUTPUT);
    pinMode(PWM_L, OUTPUT); pinMode(L_IS, OUTPUT);
    pinMode(REN, OUTPUT);   pinMode(LEN, OUTPUT);

    pinMode(PWM_R_two, OUTPUT); pinMode(R_IS_two, OUTPUT);
    pinMode(PWM_L_two, OUTPUT); pinMode(L_IS_two, OUTPUT);
    pinMode(REN_two, OUTPUT);   pinMode(LEN_two, OUTPUT);

    digitalWrite(REN, HIGH);  digitalWrite(LEN, HIGH);
    digitalWrite(REN_two, HIGH);  digitalWrite(LEN_two, HIGH);

    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
}
