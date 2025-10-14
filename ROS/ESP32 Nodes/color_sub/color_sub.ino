#include <ros.h>
#include <std_msgs/String.h>

#define LED 9

//AINDA NAO ESTA FUNCIONAL (ACHO)

ros::NodeHandle nh;

void messageCb( const std_msgs::String& toggle_msg){
  digitalWrite(LED, HIGH-digitalRead(LED));   // blink the led
}

ros::Subscriber<std_msgs::String> if()sub("cor_detectada", &messageCb );

void setup()
{
  pinMode(LED, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}