#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"

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
#define maxDistanciaCor 100   // A distancia maxima que atualiza a cor, em cm. 
#define minDistanceMove 15    // Se a distancia for menor que essa, nao ocorre o movimento. 

Adafruit_NeoPixel ws2812bONE(NUM_PIXELS, PIN_WS2812BONE, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ws2812bTWO(NUM_PIXELS, PIN_WS2812BTWO, NEO_GRB + NEO_KHZ800);

// CONSTANTES GERAIS 
#define SOUND_SPEED 0.034
#define BAUD_RATE 115200
#define ESQUERDA 0
#define DIREITA 1 

// VARIAVEIS PARA REGULAR OS SERVOS
Servo servo[6]; 
#define pos_wait_pe       90            // Angulo de espera dos servos do quadril -> Posicao "neutra"
#define pos_wait_quadril  90            // Angulo de espera dos servos do pe 
#define pos_wait_garra    90            // Angulo de espera dos servos da garra 
// Essas posicoes sao as que os servos devem ser colocados quando forem ser inseridos no robo. Uma vez que e utilizada como posicao 0 (padrao) dos servos. 
// Posicoes maximas e minimas: 
#define ang_closed_garra 110 
#define ang_open_garra 90
#define MovEnable 1       /* Define se pode ocorrer o movimento */

// COMANDOS
#define NADA -1

#define DINHEIRO 0
#define BALADA 1
#define MOLUSCO 2
#define CARAMBA 3
#define ANDAR 4
#define SENSOR 5 
#define PLANCTON 6

// CORES
#define AZUL 0
#define VERMELHO 1
#define VERDE 2
#define AMARELO 3 
#define BRANCO 4 
#define ROXO 5


// ESTADOS A SEREM ATUALIZADOS:
float distanceDetectedCm;
int luminosidade; 
int cor = NADA;
int comando = NADA;
volatile bool andando = false;
volatile bool ROSconectado = false;
int pos_cur_pedir;  
int pos_cur_quaddir; 
int pos_cur_peesq;
int pos_cur_quadesq;

/*
Logica para andando: 
- Quando ja estiver andando e for dado o comando de andar, nao reseta o andar. 
- Impede que ande quando a distancia entre obstaculo for menor que a distancia minima realize o andar (poderia ser so um AND dentro de um if, por isso talvez nem seja necessario)
*/

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
  } else if(strcmp(cor_detectada.data,"Yellow")==0) {
    cor = AMARELO;
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
  Serial.begin(BAUD_RATE);
  nh.getHardware()->setPort(&Serial);
  nh.getHardware()->setBaud(BAUD_RATE);
  //inicializa nó da esp
  delay(1000);
  nh.initNode();
  nh.subscribe(sub_cor);
  nh.subscribe(sub_comandos);

  //Aguarda até o ROS conectar, se ainda não estiver conectado
  while (!nh.connected()) {
    nh.loginfo("Tentando conectar com o ROS...");
    nh.spinOnce();
    delay(10);
  }
  nh.loginfo("ROS conectado!");

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
  nh.loginfo("Componentes inicializados!");

  // Inicializa as tasks para utilizar o DualCore 
  xTaskCreatePinnedToCore(Task1code, "Task1", 8192, NULL, 2, &Task1, 0);                           
  xTaskCreatePinnedToCore(Task2code, "Task2", 8192, NULL, 1, &Task2, 1);          
}

void loop(){}

// Task1code: controla os sensores
void Task1code(void *pvParameters){
  while(true){
    detectaDistancia();
    nh.spinOnce();
    ROSconectado = nh.connected();
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

//Task2code: controla o andar do robô
void Task2code( void * pvParameters ){
  const TickType_t tempoTicks = 50 / portTICK_PERIOD_MS;
  const TickType_t rate_10hz = 100 / portTICK_PERIOD_MS;

  while(true){
    if(ROSconectado){
      nh.loginfo(distancia);
      //DaUmPassoFrente();
    }
    vTaskDelay(rate_10hz);
  }
}

// --------------------------------------------------- ################# FUNCOES DO SERVOMOTOR: #################### ----------------------------------------------------------------

// Executar essa funcao toda vez que um loop de andar for executado, para retornar os quadris a sua posicao neutra
void retornaPosicaoPadrao() {
  int qtde_iteracoes = 10; 
  int incrementoquaddir = (pos_wait_quadril - pos_cur_quaddir)/qtde_iteracoes;
  int incrementoquadesq = (pos_wait_quadril - pos_cur_quadesq)/qtde_iteracoes;  
  int intervalo = 100;

  /* Retorna o quadril direito para a posicao padrao */
  for(int i=0; i<qtde_iteracoes; i++) {
    servo[QUADDIR].write(pos_cur_quaddir); 
    pos_cur_quaddir += incrementoquaddir; 
    vTaskDelay(intervalo); 
  }

  /* Retorna o quadril esquerdo para a posicao padrao */
  for(int i=0; i<qtde_iteracoes; i++) {
    servo[QUADESQ].write(pos_cur_quadesq);
    pos_cur_quadesq += incrementoquadesq; 
    vTaskDelay(intervalo); 
  }

}

// Bate as garras por uma certa quantidade de vezes fornecida 
// O angulo maximo a qual as garras devem atingir nao deve gerar o choque das garras, Assim como o angulo minimo nao pode gerar distensao (choque dentro das garras)
// VER COM A MARIA EDUARDA OS ANGULOS DA SIMULACAO
void bateGarras(int qtdeVezes) {
  int tempo = 10;       // O tempo entre as iteracoes do loop 
  int pos_garra_esq;
  int pos_garra_dir;
  int incremento = (ang_closed_garra - ang_open_garra)/10;    // O incremento a cada iteracao do loop, mudar o denominador para maior ou para menor para regular 
  // Garante que ap posicao maxima e minima nunca seja ultrapassada   -> TOMAR CUIDADO COM ISSO! 

  // Setar os servos para posicao de espera 
  servo[GARRADIR].write(pos_wait_garra);
  servo[GARRAESQ].write(pos_wait_garra);         // Depois definir se a posicao de espera da garra e a mesma que o angulo minimo ocupado pela garra

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
    servo[PEESQ].write(pos_wait_pe);
    servo[PEDIR].write(pos_wait_pe);

    // Variaveis para regular a posicao dos servos 
    int pos_maxima =  40;
    int range = pos_maxima - 0;             // A abrangencia do movimento, utilizado para determinar o incremento  
    int pos_cur_esq = pos_wait_quadril;     // Permite a regulacao da posicao dos servos 
    int pos_cur_dir = pos_wait_quadril;     // Podemos utilizar para atualizar a posicao e como criterio de break do loop
    int incremento = 10; 
    int tempo = 10; 

    // Executa as duas oscilacoes 
    for(int i = 0; i<qtdeOscilacoes; i++) {

      vTaskDelay(tempo / portTICK_PERIOD_MS);

        // Direita para cima: 
        servo[PEESQ].write(pos_wait_pe);   // Seta o pe esquerdo para ficar no chao 
        for(int j = 0; (pos_cur_dir<=pos_maxima) ; j ++) { 
            servo[PEDIR].write(pos_cur_dir); 
            pos_cur_dir+= incremento;
            vTaskDelay(tempo / portTICK_PERIOD_MS);
            if((pos_cur_dir>=pos_maxima)) break; 
        }
    
        // Esquerda para cima: 
        servo[PEDIR].write(pos_wait_pe);  // Seta pe direito para ficar no chao
        for(int j = 0; (pos_cur_esq<=pos_maxima) ; j++) {
            servo[PEESQ].write(pos_cur_esq);
            pos_cur_esq += incremento;
            vTaskDelay(tempo / portTICK_PERIOD_MS);
            if((pos_cur_esq>=pos_maxima)) break; 
        }
    }
}

// Da um passo para algum lado 
// ESSA FUNCAO TEM COMO FONTE O MODO "MOONWALK" DO VIDEO https://www.youtube.com/watch?v=VD6sgTo6NOY
void UmSwing() {
  // Setar os servos para posicao de espera 
  servo[PEDIR].write(pos_wait_pe); 
  servo[PEESQ].write(pos_wait_pe); 
  
  /* Valores ja definidos */ 
  int intervalo = 100;          /* Intervalo de tempo entre os passos */
  int pos_max_pes = 150;
  int pos_min_pes = pos_wait_pe; /* A posicao que indica que o pe esta no solo */
  int qtde_iteracoes = 5;

  /* Valores calculados a partir das variaveis definidas */
  int pos_cur_pedir = pos_min_pes; 
  int pos_cur_peesq = pos_min_pes;
  int incrementope = (pos_max_pes - pos_min_pes)/qtde_iteracoes;

  /* Movimento ascendente do pe direito */
  for(int i=0; i<qtde_iteracoes; i++) {
    servo[PEDIR].write(pos_cur_pedir);
    pos_cur_pedir += incrementope; 
    vTaskDelay(intervalo); 
  }
  servo[PEDIR].write(pos_max_pes);

  /* Movimento descendente do pe direito */
  for(int i=0; i<qtde_iteracoes; i++) {
    servo[PEDIR].write(pos_cur_pedir);
    pos_cur_pedir -= incrementope; 
    vTaskDelay(intervalo); 
  }
  servo[PEDIR].write(pos_min_pes);

  /* Movimento ascendente do pe esquerdo */
  for(int i=0; i<qtde_iteracoes; i++) {
    servo[PEESQ].write(pos_cur_peesq);
    pos_cur_peesq += incrementope; 
    vTaskDelay(intervalo); 
  }
  servo[PEESQ].write(pos_max_pes);

  /* Movimento descendente do pe esquerdo */
  for(int i=0; i<qtde_iteracoes; i++) {
    servo[PEESQ].write(pos_cur_peesq);
    pos_cur_peesq -= incrementope; 
    vTaskDelay(intervalo); 
  }
  servo[PEESQ].write(pos_min_pes);

}

// ESSA FUNCAO TEM COMO FONTE O MODO "TIPTOE SWING" DO VIDEO https://www.youtube.com/watch?v=VD6sgTo6NOY
void UmSwingQuadris() {
}

/* ----------------------------------- ######### FUNCAO PRINCIPAL DE ANDAR ########### --------------------------- 
--------- ###### OBSERVACOES SOBRE ESSA FUNCAO ######### ------------- 
- O movimento dos pes esta sincronizado 
- Os quadris se movem em direcoes opostas de varredura de angulo
- Mas os quadris estao "alinhados" pelo diametro, apresentam a mesma diferenca de angulo em relacao a posicao em que o servo esta parado. 
OBS: Essa funcao pode usar apenas dois servomotores simultaneamente, no maximo. 
NAO FAZER NENHUM MOVIMENTO BRUSCO! 
*/
void DaUmPassoFrente() {
  /* Valores ja definidos */ 
  int intervalo = 100;          /* Intervalo de tempo entre os passos */
  int pos_max_quadris = 110;    /* A posicao minima deve ser a posicao padrao */
  int pos_max_pes = 50;
  int pos_min_quadris = pos_wait_quadril; 
  int pos_min_pes = pos_wait_pe; /* A posicao que indica que o pe esta no solo */
  int qtde_iteracoes = 10;

  /* Valores calculados a partir das variaveis definidas */
  int incrementope = (pos_max_pes - pos_min_pes)/qtde_iteracoes;
  int incrementoquad = (pos_max_quadris - pos_min_quadris)/qtde_iteracoes;

  /* ----------------------------------------- MOVIMENTO DA PARTE DIREITA ------------------------------------------ */
  /* Retornar o quadril direito a posicao padrao */
  moveUmServoSuavemente(QUADDIR,pos_cur_quaddir,pos_wait_quadril,qtde_iteracoes); 
  pos_cur_quaddir = pos_wait_quadril; 

  /* Movimentacao ascendente do quadril e movimento ascendente do pe */
  for(int i=0; i<qtde_iteracoes; i++) {
    servo[PEDIR].write(pos_cur_pedir);
    servo[QUADDIR].write(pos_cur_quaddir);
    pos_cur_pedir += incrementope; 
    pos_cur_quaddir += incrementoquad; 
    vTaskDelay(intervalo); 
  }

  /* Movimento descedente do pe para a posicao original */
  moveUmServoSuavemente(PEDIR,pos_cur_pedir,pos_wait_pe,qtde_iteracoes);
  pos_cur_pedir = pos_wait_pe; 
  vTaskDelay(500);

  /* ----------------------------------------------- MOVIMENTO DA PARTE ESQUERDA ------------------------------------------------- */
  /* Retornar o quadril esquerdo a posicao padrao */
  moveUmServoSuavemente(QUADESQ,pos_cur_quadesq,pos_wait_quadril,qtde_iteracoes);
  pos_cur_quadesq = pos_wait_quadril; 

  /* Movimento ascendente do pe e do quadril */
  for(int i=0; i<qtde_iteracoes; i++) {
    servo[PEESQ].write(pos_cur_peesq);
    servo[QUADESQ].write(pos_cur_quadesq);
    pos_cur_peesq += incrementope; 
    pos_cur_quadesq += incrementoquad; 
    vTaskDelay(intervalo); 
  }

  /* Movimento descendente do pe */
  moveUmServoSuavemente(PEESQ,pos_cur_peesq,pos_wait_pe,qtde_iteracoes); 
  pos_cur_peesq = pos_wait_pe; 
}

/* Funcoes auxiliares para o andar principal: 
*/


/* ---------------------------------- #################### FUNCAO DE ANDAR PARA TRAS #################### ---------------------------------------- */
/*
IDEIA: Basicamente a funcao de dar passo para a frente, porem os limites sao invertidos. Varia por uma faixa de angulos "inversa" em relacao ao andar para frente. 
Hipotese: Apenas os angulos do quadril devem se alterar, os angulos dos pes continuam os mesmos. 
*/

void DaUmPassoTras() {
  // PENSAR EM SETAR AS POSICOES MINIMAS COMO A POSICAO PADRAO -> PRECISA FAZER ISSO 
}

/* ------------------ FUNCAO DE DESVIAR PARA UM LADO ---------------------- */
/* 
Desvia para o lado especificado em 90 graus. Apos isso precisaria realizar a verificacao antes de iniciar o andar. 
Video de referencia: https://www.youtube.com/watch?v=VD6sgTo6NOY      - Minuto a

*/
void DesviaUmLado(int side) {
  switch(side) {
    case DIREITA: 



    break; 
    case ESQUERDA: 
    break; 
  }
}

/* ------------------------------- ####################### FUNCOES MODULARES PARA MEXER OS SERVOS ##################### --------------------------------------- */

// Move o servo suavemente de uma posicao para outra. OBS: NAO ATUALIZA A POSICAO, ELA PRECISA SER ATUALIZADA NA FUNCAO EM QUE E CHAMADA. 
void moveUmServoSuavemente(int s, int pos_inicial, int pos_final, int qtde_iteracoes) {
  int incremento = (pos_final - pos_inicial)/qtde_iteracoes; 
  int pos_cur = pos_inicial; 
  int intervalo = 100; 

  for(int i=0; i<qtde_iteracoes; i++) {
    servo[s].write(pos_cur);
    pos_cur += incremento; 
    vTaskDelay(intervalo); 
  }
  servo[s].write(pos_final);
}

// -------------------------------------- ################## FUNCOES DE SENSORES ################### ------------------------------------------
void detectaDistancia() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  int duration = pulseIn(echoPin, HIGH, 30000);
  distanceDetectedCm = duration * SOUND_SPEED / 2;
}

void detectaLuz() {
  luminosidade = analogRead(ldr);
}

// -------------------------------------- ################## FUNCOES DO MODULO LED ################### ------------------------------------------
void modoBalada() {
  ws2812bONE.clear(); 
  ws2812bTWO.clear(); 
  int tempo = 10;

  // Cor Vermelha
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {         
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(255, 0, 0));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(255, 0, 0));  
    ws2812bONE.show();                                          
    ws2812bTWO.show(); 
    vTaskDelay(tempo / portTICK_PERIOD_MS);
  }
  // Cor Verde 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {         
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(0, 255, 0));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(0, 255, 0));
    ws2812bONE.show();                                     
    ws2812bTWO.show();
    vTaskDelay(tempo / portTICK_PERIOD_MS); 
  }
  // Cor Azul 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {        
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(0, 0, 255));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(0, 0, 255)); 
    ws2812bONE.show();                                          
    ws2812bTWO.show(); 
    vTaskDelay(tempo / portTICK_PERIOD_MS);
  }
  // Cor Branca 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {        
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(255, 255, 255));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(255, 255, 255)); 
    ws2812bONE.show();                                          
    ws2812bTWO.show(); 
    vTaskDelay(tempo / portTICK_PERIOD_MS);
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
    case BRANCO: 
      cores[0] = 255;
      cores[1] = 255;
      cores[2] = 255;
    break; 
    default:
      cores[0] = 0; 
      cores[1] = 0;
      cores[2] = 0; 
    break; 
  }

  for(int i=0; i<NUM_PIXELS; i++) {
    ws2812bONE.setPixelColor(i, ws2812bONE.Color(cores[0],cores[1],cores[2]));  
    ws2812bTWO.setPixelColor(i, ws2812bTWO.Color(cores[0],cores[1],cores[2]));  
    ws2812bONE.show(); 
    ws2812bTWO.show(); 
  }
}
