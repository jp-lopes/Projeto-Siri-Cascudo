#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

// PINOS 
// Módulo LED
#define PIN_WS2812BONE  13
#define PIN_WS2812BTWO  14
#define NUM_PIXELS      8          //The number of LEDs (pixels) on WS2812B LED strip

// LDR e Ultrassonico
#define ldr             25
#define echoPin         27
#define trigPin         26

// SERVOS
// Pinos 
#define QUADDIR_PIN     18
#define PEDIR_PIN       19
#define QUADESQ_PIN     21
#define PEESQ_PIN       22
#define GARRADIR_PIN    23
#define GARRAESQ_PIN    33

// Indices no vetor de servo 
#define QUADDIR     0 
#define PEDIR       1
#define QUADESQ     2
#define PEESQ       3
#define GARRADIR    4
#define GARRAESQ    5

// VARIAVEIS 
// MODULO LED 
#define NADA -1
#define AZUL 0
#define VERMELHO 1
#define VERDE 2
#define AMARELO 3 
#define BRANCO 4 
#define ROXO 5
#define maxDistanciaCor 100   // A distancia maxima que atualiza a cor, em cm. 
#define minDistanceMove 15    // Se a distancia for menor que essa, nao ocorre o movimento. 

// COMANDOS 
#define DINHEIRO 0
#define BALADA 1
#define MOLUSCO 2
#define CARAMBA 3
#define ANDAR 4
#define SENSOR 5 
#define PLANCTON 6

Adafruit_NeoPixel ws2812bONE(NUM_PIXELS, PIN_WS2812BONE, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ws2812bTWO(NUM_PIXELS, PIN_WS2812BTWO, NEO_GRB + NEO_KHZ800);

// CONSTANTES GERAIS 
#define SOUND_SPEED 0.034
#define BAUD_RATE 115200
#define ESQ 0
#define DIR 1 

// VARIAVEIS PARA REGULAR OS SERVOS
Servo servo[6]; 
#define wait_pos_leg  90            // Angulo de espera dos servos do quadril -> Posicao "neutra"
#define wait_pos_feet 90            // Angulo de espera dos servos do pe 
#define wait_pos_claw 90            // Angulo de espera dos servos da garra 
// Essas posicoes sao as que os servos devem ser colocados quando forem ser inseridos no robo. Uma vez que e utilizada como posicao 0 (padrao) dos servos. 
// Posicoes maximas e minimas: 
#define ang_closed_garra 110 
#define ang_open_garra 90
#define MovEnable 1       /* Define se pode ocorrer o movimento */

// ESTADOS A SEREM ATUALIZADOS:
float distanceDetectedCm;
int luminosidade; 
int cor = NADA;
int comando = NADA;

// TASK HANDLE
TaskHandle_t Task1;
TaskHandle_t Task2;

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
  if(strcmp(comando_detectado.data,"dinheiro")==0) {
    comando = DINHEIRO;
  } else if(strcmp(comando_detectado.data,"plancton")==0) {
    comando = PLANCTON;
  } else if(strcmp(comando_detectado.data,"molusco")==0) {
    comando = MOLUSCO;
  } else if(strcmp(comando_detectado.data,"caramba")==0) {
    comando = CARAMBA;
  } else if(strcmp(comando_detectado.data,"sensor")==0) {
    comando = SENSOR; 
  } else if(strcmp(comando_detectado.data,"andar")==0) {
    comando = ANDAR; 
  } else if(strcmp(comando_detectado.data,"balada")==0) {
    comando = BALADA; 
  } 
  else comando = NADA;
}

ros::Subscriber<std_msgs::String> sub_cor("cor_detectada", &cor_callback);
ros::Subscriber<std_msgs::String> sub_comandos("comandos", &comando_callback);

void setup()
{
  delay(3000); // Garante que a ESP32 esteja pronta antes de iniciar Serial
  Serial.begin(BAUD_RATE);
  nh.getHardware()->setPort(&Serial);
  nh.getHardware()->setBaud(BAUD_RATE);

  //inicializa nó da esp
  delay(4000);
  nh.initNode();
  nh.subscribe(sub_cor);
  nh.subscribe(sub_comandos);
  delay(2000);

  // Inicializa componentes
  ws2812bONE.begin();
  ws2812bONE.setBrightness(10);
  ws2812bTWO.begin();
  ws2812bTWO.setBrightness(10);
  pinMode(ldr, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  servo[QUADDIR].attach(QUADDIR_PIN);
  servo[PEDIR].attach(PEDIR_PIN);
  servo[QUADESQ].attach(QUADESQ_PIN);
  servo[PEESQ].attach(PEESQ_PIN);
  servo[GARRADIR].attach(GARRADIR_PIN);
  servo[GARRAESQ].attach(GARRAESQ_PIN);

  // Inicializa as task para utilizar o DualCore 
  // Create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(Task1code, "Task1", 8192, NULL, 2, &Task1, 0);                           
  delay(500); 

  // Create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(Task2code, "Task2", 8192, NULL, 1, &Task2, 1);          
  delay(500); 

  //Aguarda até o ROS conectar, se ainda não estiver conectado
  while (!nh.connected()) {
    nh.loginfo("Tentando conectar com o ROS...");
    nh.spinOnce();   // tenta sincronizar
    delay(1);
  }
  nh.loginfo("ROS conectado!");

  setCor(VERDE);
  delay(2000);
  setCor(NADA);

}

// Task1code: controla os sensores e os leds
// Codigo para as funcoes que usam menos delay 
void Task1code( void * pvParameters ){
  int tempo = 50; 
  while(true){
    if(nh.connected()){
      nh.spinOnce();    // Atualiza cor detectada pela camera
      delay(100);       // Rate: 10Hz    
    } else {
      //Se o ROS estiver desconectado por algum motivo, tenta novamente
      nh.loginfo("ROS desconectado, tentando conectar novamente...");
      nh.initNode();
      nh.subscribe(sub_cor);
      nh.subscribe(sub_comandos);
      delay(100);
    }
  }
}


//Task2code: controla o andar do robô
void Task2code( void * pvParameters ){
  while(true){

    if(nh.connected()){
      int tempo = 50;
      detectaDistancia();
      if(comando==DINHEIRO) {
        setCor(VERDE);
        moveServosMaiores(6); 
        bateGarras(6);
        comando = NADA; 
      } 
      else if(comando==ANDAR) {
        moveServosMaiores(5);
        comando = NADA; 
      } 
      else if(comando==SENSOR) {
        sensorRe(); 
        comando = NADA;
      } 
      else if(comando==BALADA) {
        modoBalada(6); 
        comando = NADA;
      } 
      else if(comando==MOLUSCO) {
        setCor(AZUL); 
        bateGarras(6);
        delay(2000);
        comando = NADA;
      } 
      else setCor(ROXO); 
      
      vTaskDelay(tempo / portTICK_PERIOD_MS); // Suspender a task por um intervalo de tempo 
      /* Isso permite que o EspWatchdog retorne o controle para a Esp32 */
    }

    delay(100);
  }
}

void loop(){
}

// FUNCOES: 

// FUNCOES DO SERVOMOTOR: 
// Coloca as posicoes padroes dos servos (ou qualquer posicao)
void setPosicaoPadrao(){
  int tempo = 100; 
  int print = 1;      // Variavel que indica se printa ou nao no monitor serial (Tomar cuidado para nao haver requisicoes do monitor serial ao mesmo tempo)
  int posicao[6];
  
  posicao[PEDIR] = 90; 
  posicao[PEESQ] = 90; 
  posicao[QUADDIR] = 90; 
  posicao[QUADESQ] = 90;
  posicao[GARRADIR] = 90;
  posicao[GARRAESQ] = 90;
  
  if(print) {
    Serial.print("Setando posicao no servo Pe Direita...");
    Serial.println(posicao[PEDIR]); 
  }
  servo[PEDIR].write(posicao[PEDIR]);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  if(print) {
    Serial.print("Setando posicao no servo Pe Esquerda..."); 
    Serial.println(posicao[PEESQ]);
  }
  servo[PEESQ].write(posicao[PEESQ]);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  if(print) {
    Serial.print("Setando posicao no servo Quadril Direita..."); 
    Serial.println(posicao[QUADDIR]); 
  }
  servo[QUADDIR].write(posicao[QUADDIR]);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  if(print) {
    Serial.print("Setando posicao no servo Quadril Esquerda...");  
    Serial.println(posicao[QUADESQ]); 
  }
  servo[QUADESQ].write(posicao[QUADESQ]);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  if(print) {
    Serial.print("Setando posicao no servo Garra Esquerda..."); 
    Serial.println(posicao[GARRAESQ]);
  }
  servo[GARRAESQ].write(posicao[GARRAESQ]);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  if(print) { 
    Serial.print("Setando posicao no servo Garra Direita...");  
    Serial.println(posicao[GARRADIR]);  
  }
  servo[GARRADIR].write(posicao[GARRADIR]);
  vTaskDelay(tempo / portTICK_PERIOD_MS);
}

// Bate as garras por uma certa quantidade de vezes fornecida 
// O angulo maximo a qual as garras devem atingir nao deve gerar o choque das garras, Assim como o angulo minimo nao pode gerar distensao (choque dentro das garras)
void bateGarras(int qtdeVezes) {
  int tempo = 10;       // O tempo entre as iteracoes do loop 
  int pos_garra_esq;
  int pos_garra_dir;
  int incremento = (ang_closed_garra - ang_open_garra)/10;    // O incremento a cada iteracao do loop, mudar o denominador para maior ou para menor para regular 
  // Garante que ap posicao maxima e minima nunca seja ultrapassada   -> TOMAR CUIDADO COM ISSO! 

  // Setar os servos para posicao de espera 
  servo[GARRADIR].write(wait_pos_claw);
  servo[GARRAESQ].write(wait_pos_claw);         // Depois definir se a posicao de espera da garra e a mesma que o angulo minimo ocupado pela garra

  // Movimento de bater as garras
  for(int i=0; i<qtdeVezes ; i++) {
    pos_garra_dir = ang_open_garra;
    pos_garra_esq = ang_open_garra;
    vTaskDelay(tempo / portTICK_PERIOD_MS);
  // Transicao nao tao brusca para fechar a garra
    while((pos_garra_dir<=ang_closed_garra) && (pos_garra_esq<=ang_closed_garra)) {
      servo[GARRADIR].write(pos_garra_dir); 
      servo[GARRAESQ].write(pos_garra_esq);
      pos_garra_dir+= incremento;
      pos_garra_esq+= incremento; 
      vTaskDelay(tempo / portTICK_PERIOD_MS);
    }
    
    vTaskDelay(tempo / portTICK_PERIOD_MS);

    pos_garra_dir = ang_closed_garra; 
    pos_garra_esq = ang_closed_garra; 
    // Transicao nao tao brusca para abrir a garra
    while((pos_garra_dir>=ang_open_garra) && (pos_garra_esq>=ang_open_garra)) {
      servo[GARRADIR].write(pos_garra_dir); 
      servo[GARRAESQ].write(pos_garra_esq);
      pos_garra_dir-= incremento;
      pos_garra_esq-= incremento; 
      vTaskDelay(tempo / portTICK_PERIOD_MS);
    }
    pos_garra_dir = ang_open_garra;
    pos_garra_esq = ang_open_garra;
    vTaskDelay(tempo / portTICK_PERIOD_MS);
  }
}

// Oscila para a direita e para a esquerda uma certa quantidade de vezes 
void oscilaLados(int qtdeOscilacoes) {
    // Setar os servos para a posicao de espera 
    servo[PEESQ].write(wait_pos_feet);
    servo[PEDIR].write(wait_pos_feet);

    // Variaveis para regular a posicao dos servos 
    int pos_maxima =  40;
    int range = pos_maxima - 0;         // A abrangencia do movimento, utilizado para determinar o incremento  
    int pos_cur_esq = wait_pos_leg;     // Permite a regulacao da posicao dos servos 
    int pos_cur_dir = wait_pos_leg;     // Podemos utilizar para atualizar a posicao e como criterio de break do loop
    int incremento = 10; 
    int tempo = 10; 

    // Executa as duas oscilacoes 
    for(int i = 0; i<qtdeOscilacoes; i++) {

      vTaskDelay(tempo / portTICK_PERIOD_MS);

        // Direita para cima: 
        servo[PEESQ].write(wait_pos_feet);   // Seta o pe esquerdo para ficar no chao 
        for(int j = 0; (pos_cur_dir<=pos_maxima) ; j ++) { 
            servo[PEDIR].write(pos_cur_dir); 
            pos_cur_dir+= incremento;
            vTaskDelay(tempo / portTICK_PERIOD_MS);
            if((pos_cur_dir>=pos_maxima)) break; 
        }
    
        // Esquerda para cima: 
        servo[PEDIR].write(wait_pos_feet);  // Seta pe direito para ficar no chao
        for(int j = 0; (pos_cur_esq<=pos_maxima) ; j++) {
            servo[PEESQ].write(pos_cur_esq);
            pos_cur_esq += incremento;
            vTaskDelay(tempo / portTICK_PERIOD_MS);
            if((pos_cur_esq>=pos_maxima)) break; 
        }
    }
}

// Move os servos (funcao teste de movimento)
void moveServosMaiores(int qtde_passos) {
  int tempo = 100; 

  for(int i=0; i<qtde_passos ;i++) {
  // LADO DIREITO 

  servo[QUADDIR].write(0);
  vTaskDelay(tempo / portTICK_PERIOD_MS);
  servo[PEDIR].write(0);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  servo[QUADDIR].write(90);
  vTaskDelay(tempo / portTICK_PERIOD_MS);
  servo[PEDIR].write(90);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  vTaskDelay(2000); 
  
  // LADO ESQUERDO 
  servo[QUADESQ].write(0);
  vTaskDelay(tempo / portTICK_PERIOD_MS);
  servo[PEESQ].write(0);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  servo[QUADESQ].write(90);
  vTaskDelay(tempo / portTICK_PERIOD_MS);
  servo[PEESQ].write(90);
  vTaskDelay(tempo / portTICK_PERIOD_MS);
  }
  
}

// FUNCOES DE SENSORES: 
void detectaDistancia() {
  int duration; 
  int print = 0; 
  digitalWrite(trigPin, LOW);
  vTaskDelay(2);
  digitalWrite(trigPin, HIGH);
  vTaskDelay(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceDetectedCm = duration * SOUND_SPEED/2;
  if(print) {
    Serial.print("Distancia detectada no ultrassonico: "); 
    Serial.println(distanceDetectedCm); 
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

void detectaLuz() {
  int print = 0; 
  luminosidade = analogRead(ldr);
  if(print) {
    Serial.print("Luminosidade detectada: ");
    Serial.println(luminosidade);
  } 
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

// FUNCOES DO MODULO LED:
void modoBalada(int qtdeVezes) {
  ws2812bONE.clear(); 
  ws2812bTWO.clear(); 
  int tempo = 10;
  
  for(int i=0; i<qtdeVezes; i++) {
  // Cor Vermelha
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {         
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(255, 0, 0));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(255, 0, 0));  
    ws2812bONE.show();                                          
    ws2812bTWO.show(); 
    vTaskDelay(tempo / portTICK_PERIOD_MS);
  }
  delay(100); 
  // Cor Verde 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {         
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(0, 255, 0));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(0, 255, 0));
    ws2812bONE.show();                                     
    ws2812bTWO.show();
    vTaskDelay(tempo / portTICK_PERIOD_MS); 
  }
  delay(100); 
  // Cor Azul 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {        
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(0, 0, 255));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(0, 0, 255)); 
    ws2812bONE.show();                                          
    ws2812bTWO.show(); 
    vTaskDelay(tempo / portTICK_PERIOD_MS);
  }
  delay(100);
  // Cor Branca 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {        
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(255, 255, 255));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(255, 255, 255)); 
    ws2812bONE.show();                                          
    ws2812bTWO.show(); 
    vTaskDelay(tempo / portTICK_PERIOD_MS);
  }
  delay(100); 
  }
}

// Setar uma cor para os Leds (usado para evitar repeticao de codigo)
void setCor(int corLigar) {
  int cores[3];
  switch (corLigar) {
    case VERMELHO: 
      cores[0] = 255;
      cores[1] = 0;
      cores[2] = 0;
    break; 
    case VERDE: 
      cores[0] = 0;
      cores[1] = 255;
      cores[2] = 0;
    break; 
    case AZUL: 
      cores[0] = 0;
      cores[1] = 0;
      cores[2] = 255;
    break; 
    case AMARELO: 
      cores[0] = 255;
      cores[1] = 255;
      cores[2] = 0;
    break; 
    case BRANCO: 
      cores[0] = 255;
      cores[1] = 255;
      cores[2] = 255;
    break; 
    case ROXO:
      cores[0] = 255;
      cores[1] = 0;
      cores[2] = 255;
    break; 
    default:
      ws2812bONE.clear();
      ws2812bTWO.clear();
      ws2812bONE.show(); 
      ws2812bTWO.show();
      return; 
    break; 
  }

  for(int i=0; i<NUM_PIXELS; i++) {
    ws2812bONE.setPixelColor(i, ws2812bONE.Color(cores[0],cores[1],cores[2]));  
    ws2812bTWO.setPixelColor(i, ws2812bTWO.Color(cores[0],cores[1],cores[2]));  
    ws2812bONE.show(); 
    ws2812bTWO.show(); 
  }
}

void sensorRe() {
  detectaDistancia();

  while(distanceDetectedCm<40) { 

    if(distanceDetectedCm<10) {
      setCor(VERMELHO); 
    } else if(distanceDetectedCm>=10 && distanceDetectedCm<20) {
      setCor(AMARELO);
    } else setCor(VERDE);

    detectaDistancia(); 
  }
}


// 2. CODIGOS PRONTOS E TESTADOS: 
// Atualiza os modulos LEDs para a cor que foi detectada pela camera 