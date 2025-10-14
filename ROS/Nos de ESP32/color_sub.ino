#include <ros.h>
#include <std_msgs/String.h>

#define LED 9

//AINDA NAO ESTA FUNCIONAL (ACHO)

ros::NodeHandle nh;

void callback( const std_msgs::String& cor_detectada){
  if(cor_detectada == "Blue") digitalWrite(LED, HIGH-digitalRead(LED));   // pisca o LED se detectar cor azul
}

ros::Subscriber<std_msgs::String> sub("cor_detectada", &callback );

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