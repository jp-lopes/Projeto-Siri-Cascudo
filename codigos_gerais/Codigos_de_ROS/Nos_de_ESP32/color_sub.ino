#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>

#define LED 9

ros::NodeHandle nh;

void callback( const std_msgs::String& cor_detectada){
  if(strcmp(cor_detectada.data,"Blue")==0) {
    digitalWrite(LED, HIGH);  // pisca o LED se detectar cor azul
    nh.loginfo("Azul detectado");
  }
  else {
    digitalWrite(LED, LOW);
    nh.loginfo("Azul nao foi detectado");
  }
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