#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

//Módulo LED
#define PIN_WS2812B 13
#define NUM_PIXELS 8          //The number of LEDs (pixels) on WS2812B LED strip
//LDR e Ultrassonico
#define ldr 25
#define echoPin 27
#define trigPin 26
//Servos
#define servo1Pin 18
#define servo2Pin 19
#define servo3Pin 21
#define servo4Pin 22
#define servo5Pin 23
#define servo6Pin 33
//Cores que podem ser detectadas
#define NADA -1
#define AZUL 0
#define VERMELHO 1
#define VERDE 2
//Constantes
#define SOUND_SPEED 0.034
#define BAUD_RATE 115200


//Variáveis globais
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

long duration;
float distanceCm;
int luz;
int cor = NADA;
volatile bool andar = false;

TaskHandle_t Task1;
TaskHandle_t Task2;

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

ros::NodeHandle nh;

//Atualiza a cor de acordo com o que foi detectado pela camera
void cor_callback(const std_msgs::String& cor_detectada){
  if(strcmp(cor_detectada.data,"Blue")==0) {
    cor = AZUL;
  } else if(strcmp(cor_detectada.data,"Red")==0) {
    cor = VERMELHO;
  } else if(strcmp(cor_detectada.data,"Green")==0) {
    cor = VERDE;
  } else cor = NADA; 
}

void comando_callback(const std_msgs::String& comando_detectado){
  if(!andar && strcmp(comando_detectado.data,"ANDAR")==0) {
    andar = true;
  } 
}

ros::Subscriber<std_msgs::String> sub_cor("cor_detectada", &cor_callback);
ros::Subscriber<std_msgs::String> sub_comandos("comandos", &comando_callback);

void setup()
{
  Serial.begin(BAUD_RATE);
  nh.getHardware()->setPort(&Serial);
  nh.getHardware()->setBaud(BAUD_RATE);

  //inicializa nó da esp
  delay(4000);
  nh.initNode();
  nh.subscribe(sub_cor);
  nh.subscribe(sub_comandos);
  delay(2000);

  //inicializa componentes
  ws2812b.begin();
  ws2812b.setBrightness(10);
  //pinMode(ldr, INPUT);
  //pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);
  servo5.attach(servo5Pin);
  servo6.attach(servo6Pin);

  //Aguarda até o ROS conectar, se ainda não estiver conectado
  while (!nh.connected()) {
    nh.loginfo("Tentando conectar com o ROS...");
    nh.spinOnce();   // tenta sincronizar
    delay(1);
  }
  nh.loginfo("ROS conectado!");

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 2, &Task1, 0);                           
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(Task2code, "Task2", 10000, NULL, 1, &Task2, 1);          
  delay(500); 

}

// Task1code: controla os sensores e os leds
// Codigo para as funcoes que usam menos delay 
void Task1code( void * pvParameters ){
  while(true){
    if(nh.connected()){
      //COLOCAR ACOES AQUI:
      atualizarLEDS();
      // luz = analogRead(ldr);

      // digitalWrite(trigPin, LOW);
      // delayMicroseconds(2);
      // digitalWrite(trigPin, HIGH);
      // delayMicroseconds(10);
      // digitalWrite(trigPin, LOW);

      // duration = pulseIn(echoPin, HIGH);

      // distanceCm = duration * SOUND_SPEED/2;
    } else {
      //Se o ROS estiver desconectado por algum motivo, tenta novamente
      nh.loginfo("ROS desconectado, tentando conectar novamente...");
      nh.initNode();
      nh.subscribe(sub_cor);
      nh.subscribe(sub_comandos);
      delay(100);
    }

    nh.spinOnce();    // Atualiza cor detectada pela camera
    delay(100);       // Rate: 10Hz    
  }
}


//Task2code: controla o andar do robô
void Task2code( void * pvParameters ){
  while(true){

    if(nh.connected()){
      //COLOCAR FUNCOES DE MOVIMENTO AQUI
      if(andar) {
        nh.loginfo("Dando um passo...");
        DaUmPasso();
      }
      else setPosicaoPadrao();
    }

    delay(100);
  }
}

void loop(){
}

void atualizarLEDS(){
  switch (cor){
    case NADA:
      ws2812b.clear();
      ws2812b.show();
      break;
    case AZUL:
      nh.loginfo("Setando LED azul");
      for(int i=0;i<NUM_PIXELS;i++) {
        ws2812b.setPixelColor(i, ws2812b.Color(0,0,255));  
        ws2812b.show();
      }
      break;
    case VERMELHO:
      nh.loginfo("Setando LED vermelho");
      for(int i=0;i<NUM_PIXELS;i++) {
        ws2812b.setPixelColor(i, ws2812b.Color(255,0,0));  
        ws2812b.show();
      }
      break;
    case VERDE:
      nh.loginfo("Setando LED verde");
      for(int i=0;i<NUM_PIXELS;i++) {
        ws2812b.setPixelColor(i, ws2812b.Color(0,255,0));  
        ws2812b.show();
      }
      break;
    default:
      ws2812b.clear();
      ws2812b.show();
      break;
  }
}

void DaUmPasso(){
    servo1.write(0);
    delay(100);
    servo2.write(0);
    delay(100);
    servo3.write(0);
    delay(100);
    servo4.write(0);
    delay(100);
    servo5.write(0);
    delay(100);
    servo6.write(0);
    delay(100);

    servo1.write(180);
    delay(100);
    servo2.write(180);
    delay(100);
    servo3.write(180);
    delay(100);
    servo4.write(180);
    delay(100);
    servo5.write(180);
    delay(100);
    servo6.write(180);
    delay(100);
    andar = false;
}

void setPosicaoPadrao(){
    servo1.write(45);
    delay(100);
    servo2.write(45);
    delay(100);
    servo3.write(45);
    delay(100);
    servo4.write(45);
    delay(100);
    servo5.write(45);
    delay(100);
    servo6.write(45);
    delay(100);
}