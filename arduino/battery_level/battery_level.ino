#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#define ANALOG_IN_PIN A0

ros::NodeHandle nh;

std_msgs::Float32 battery_msg;
ros::Publisher battery_pub("battery_percentage", &battery_msg);

float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
float ref_voltage = 5.0;

int adc_value = 0;
const int numReadings = 10;

void setup() {
  nh.initNode();
  nh.advertise(battery_pub);
}

void loop() {
  float total_percentage = 0.0;
  float total_voltage = 0.0;

  for (int i = 0; i < numReadings; i++) {
    adc_value = analogRead(ANALOG_IN_PIN);
    adc_voltage = (adc_value * ref_voltage) / 1024.0;
    in_voltage = adc_voltage / (R2 / (R1 + R2));

    float percentage;
    if (in_voltage >= 12.6) {
      percentage = 100.0;
    } else if (in_voltage < 11.8) {
      percentage = 0.0;
    } else {
      percentage = (in_voltage - 11.8) / (12.6 - 11.8) * 100.0;
    }

    total_percentage += percentage;
    total_voltage += in_voltage;

    delay(50);
  }

  float avg_percentage = total_percentage / numReadings;
  battery_msg.data = avg_percentage;
  battery_pub.publish(&battery_msg);
  nh.spinOnce();
  delay(1000);
}
