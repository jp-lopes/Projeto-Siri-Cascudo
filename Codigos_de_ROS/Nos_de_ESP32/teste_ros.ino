#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

#define PIN_WS2812B 33  // The ESP32 pin GPIO16 connected to WS2812B
#define NUM_PIXELS 8   // The number of LEDs (pixels) on WS2812B LED strip
#define ldr 25
#define echoPin 27
#define trigPin 26
#define servo1Pin 18
#define servo2Pin 19
#define servo3Pin 21
#define servo4Pin 22
#define servo5Pin 2
#define servo6Pin 4
#define SOUND_SPEED 0.034

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

long duration;
float distanceCm;
int luz;

TaskHandle_t Task1;
TaskHandle_t Task2;

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

ros::NodeHandle nh;

#define NADA -1
#define AZUL 0
#define VERMELHO 1
#define VERDE 2

int cor;

void color_callback( const std_msgs::String& cor_detectada){
  
  if(strcmp(cor_detectada.data,"Blue")==0) {
    cor = AZUL;
  } else if(strcmp(cor_detectada.data,"Red")==0) {
    cor = VERMELHO;
  } else if(strcmp(cor_detectada.data,"Green")==0) {
    cor = VERDE;
  } else {
    cor = NADA;
  }

  Serial.print("Cor detectada: ");
  switch (cor){
    case NADA:
      Serial.println("nenhuma");
      break;
    case AZUL:
      Serial.println("azul");
      break;
    case VERMELHO:
      Serial.println("vermelho");
      break;
    case VERDE:
      Serial.println("verde");
      break;
    default:
      Serial.println("(??)");
      break;
  }
}

ros::Subscriber<std_msgs::String> sub("cor_detectada", &color_callback );

void setup()
{
  Serial.begin(9600);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. (eu não sei se pode mudar, então deixei do jeito que ta)*/
                    "Task1",     /* name of task. (mesma coisa)*/
                    10000,       /* Stack size of task (não sei oq isso faz)*/
                    NULL,        /* parameter of the task (tbm não sei oq isso faz)*/
                    0,           /* priority of the task (deixar a task de andar com mais prioridade)*/
                    &Task1,      /* Task handle to keep track of created task (não faço a menor ideia do que isso seja)*/
                    0);          /* pin task to core 0 (auto explicativo)*/                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. (eu não sei se pode mudar, então deixei do jeito que ta)*/
                    "Task2",     /* name of task. (mesma coisa)*/
                    10000,       /* Stack size of task (nao sei oq isso faz)*/
                    NULL,        /* parameter of the task (tbm nao sei oq isso faz)*/
                    1,           /* priority of the task (deixar a task de andar com mais prioridade)*/
                    &Task2,      /* Task handle to keep track of created task (não faço a menor ideia do que isso seja)*/
                    1);          /* pin task to core 1 (auto explicativo)*/
    delay(500); 

  ws2812b.begin();
  pinMode(ldr, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);
  servo5.attach(servo5Pin);
  servo6.attach(servo6Pin);
  // inicializa nó da esp
  nh.initNode();
  nh.subscribe(sub);
}

// Task1code: controla os sensores e os leds
// Codigo para as funcoes que usam menos delay 
void Task1code( void * pvParameters ){
  for(;;){
    luz = analogRead(ldr);

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);

    distanceCm = duration * SOUND_SPEED/2;

    Serial.print("Distance (cm): ");
    Serial.println(distanceCm);

    // Modulo LED
    switch (cor){
      case NADA:
        ws2812b.show(clear);
        break;
      case AZUL:
        ws2812b.setPixelColor(255, ws2812b.Color(0,0,255));  
        ws2812b.show(); 
        break;
      case VERMELHO:
        ws2812b.setPixelColor(255, ws2812b.Color(255,0,0));  
        ws2812b.show(); 
        break;
      case VERDE:
        ws2812b.setPixelColor(255, ws2812b.Color(0,255,0));  
        ws2812b.show(); 
        break;
      default:
        ws2812b.show(clear);
        break;
    }
    nh.spinOnce();
    delay(1);         // A taxa de recepcao de dados da rasp deve ser alta 
}


//Task2code: controla o andar do robô
void Task2code( void * pvParameters ){
  for(;;){
    Serial.println(luz);
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
  }
}

void loop(){
}