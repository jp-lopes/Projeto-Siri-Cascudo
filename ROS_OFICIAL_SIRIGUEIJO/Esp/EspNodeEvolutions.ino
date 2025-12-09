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


#define echoPin         26
#define trigPin         27
/*
#define echoPin         27
#define trigPin         26
*/

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
// ANGULOS DE ESPERA DOS SERVOMOTORES 
// Posicao padrao para os pes 
#define pos_padrao_esquerdo 70                  // POSICAO CERTA!!
#define pos_padrao_direito  75                  // POSICAO CERTA!! 
#define pos_wait_quadrildireito 85
#define pos_wait_quadrilesquerdo 90             // POSICAO CERTA!!
#define pos_wait_garra_direita    90            
#define pos_wait_garra_esquerda 90
// Essas posicoes sao as que os servos devem ser colocados quando forem ser inseridos no robo. Uma vez que e utilizada como posicao 0 (padrao) dos servos. 
// Posicoes maximas e minimas: 
#define ang_closed_garra 130
#define ang_open_garra 70
#define MovEnable 1       /* Define se pode ocorrer o movimento */
#define erro_baixo_ultrassonico 1       // Para desconsiderar os valores de erro produzidos pelo ultrassonico 

// COMANDOS
#define NADA -1

#define DINHEIRO 0
#define BALADA 1
#define MOLUSCO 2
#define CARAMBA 3
#define ANDAR 4
#define SENSOR 5 
#define PLANCTON 6
#define PARAR 7
#define DANCAR 8
#define DIREITA 9
#define ESQUERDA 10
#define CORCMD 11

// CORES
#define AZUL 0
#define VERMELHO 1
#define VERDE 2
#define AMARELO 3 
#define BRANCO 4 
#define ROXO 5


// ESTADOS A SEREM ATUALIZADOS:
volatile float distanceDetectedCm = 0;
volatile int luminosidade; 
volatile int cor = NADA;
volatile int comando = NADA;
volatile int corAtual = NADA;
volatile bool andando = false;
volatile bool ROSconectado = false;
int pos_cur_pedir;  
int pos_cur_quaddir; 
int pos_cur_peesq;
int pos_cur_quadesq; 
int pos_garra_esq;
int pos_garra_dir;
volatile int print = 0;
int CorCountBalada = 0; 

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
  } else if(strcmp(cor_detectada.data,"White")==0) {
    cor = BRANCO;
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
  } else if(strcmp(comando_detectado.data,"dancar")==0) {
    comando = DANCAR; 
  } else if(strcmp(comando_detectado.data,"direita")==0) {
    comando = DIREITA; 
  } else if(strcmp(comando_detectado.data,"esquerda")==0) {
    comando = ESQUERDA; 
  } else if(strcmp(comando_detectado.data,"COR")==0) {
    comando = CORCMD; 
  }
  else comando = NADA;
}

ros::Subscriber<std_msgs::String> sub_cor("cor_detectada", &cor_callback);
ros::Subscriber<std_msgs::String> sub_comandos("comandos", &comando_callback);

void setup() {
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
  xTaskCreatePinnedToCore(Task2code, "Task2", 8192, NULL, 2, &Task2, 1);   

  setPosicaoPadrao();      
}

void loop(){
    nh.spinOnce();
    ROSconectado = nh.connected();

    // Printa distancia 5 vezes por segundo (a cada 200ms)
    static unsigned long timer = 0;
    if( (millis() - timer > 200) && ROSconectado) { 
      timer = millis();
      char buffer[50];
      float valor = (float)distanceDetectedCm; 
      sprintf(buffer, "Distancia: %.2f cm", valor);
      nh.loginfo(buffer);
      nh.loginfo("ESP node funcionando");
    }

    vTaskDelay(2 / portTICK_PERIOD_MS);

}

// Task1code: controla os sensores
void Task1code(void *pvParameters){
  while(true){
    if(!ROSconectado) {vTaskDelay(20 / portTICK_PERIOD_MS); continue;}
    detectaDistancia();   
    vTaskDelay(40 / portTICK_PERIOD_MS); //25Hz
  }
}

//Task2code: controla o andar do robô
void Task2code( void * pvParameters ){
  const TickType_t tempoTicks = 50 / portTICK_PERIOD_MS;
  while(true){
    if(!ROSconectado) {vTaskDelay(20 / portTICK_PERIOD_MS); continue;}
    if(comando == ANDAR && (distanceDetectedCm>10 || distanceDetectedCm<1)){
      DaUmPassoFrente();
    } 
    else if(comando == BALADA){
      modoBalada();
    }
    else if(comando == DINHEIRO){
      if(corAtual != VERDE) setCor(VERDE);
      UmSwing();
    }
    else if(comando == DANCAR) {
      setCor(CorCountBalada); 
      if(CorCountBalada<3) CorCountBalada++;
      else CorCountBalada = 0; 
      UmSwing();
    }
    else if(comando == DIREITA) {
      DesviaUmLado(DIREITA); 
    } else if(comando == ESQUERDA) {
      DesviaUmLado(ESQUERDA); 
    } else if(comando == CORCMD){
      setCor(cor);
    }
    else {
      retornaPosicaoPadrao();
      setCor(NADA);
    }
    vTaskDelay(tempoTicks);
  }
}

// --------------------------------------------------- ################# FUNCOES DO SERVOMOTOR: #################### ----------------------------------------------------------------
// Coloca os servos nas posicoes padroes 
void setPosicaoPadrao(){
  int tempo = 100; 
  pos_cur_pedir = pos_padrao_direito;  
  pos_cur_quaddir = pos_wait_quadrildireito; 
  pos_cur_peesq = pos_padrao_esquerdo;
  pos_cur_quadesq = pos_wait_quadrilesquerdo; 
  pos_garra_esq = pos_wait_garra_esquerda;
  pos_garra_dir = pos_wait_garra_direita;
  
  if(print) {
    Serial.print("Setando posicao no servo Pe Direita...");
    Serial.println(pos_padrao_direito); 
  }
  servo[PEDIR].write(pos_padrao_direito);
  vTaskDelay(tempo);

  if(print) {
    Serial.print("Setando posicao no servo Pe Esquerda..."); 
    Serial.println(pos_padrao_esquerdo);
  }
  servo[PEESQ].write(pos_padrao_esquerdo);
  vTaskDelay(tempo);

  if(print) {
    Serial.print("Setando posicao no servo Quadril Direita..."); 
    Serial.println(pos_wait_quadrildireito); 
  }
  servo[QUADDIR].write(pos_wait_quadrildireito);
  vTaskDelay(tempo);

  if(print) {
    Serial.print("Setando posicao no servo Quadril Esquerda...");  
    Serial.println(pos_wait_quadrilesquerdo); 
  }
  servo[QUADESQ].write(pos_wait_quadrilesquerdo);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  if(print) {
    Serial.print("Setando posicao no servo Garra Esquerda..."); 
    Serial.println(pos_wait_garra_esquerda);
  }
  servo[GARRAESQ].write(pos_wait_garra_esquerda);
  vTaskDelay(tempo / portTICK_PERIOD_MS);

  if(print) { 
    Serial.print("Setando posicao no servo Garra Direita...");  
    Serial.println(pos_wait_garra_direita);  
  }
  servo[GARRADIR].write(pos_wait_garra_direita); 
  vTaskDelay(tempo / portTICK_PERIOD_MS);
}

// Seta os servos para 90 -> Chamar antes de 
void setPosicao90(){
  int print = 0;                // Variavel que indica se printa ou nao no monitor serial (Tomar cuidado para nao haver requisicoes do monitor serial ao mesmo tempo)
  int tempo = 100; 
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
  vTaskDelay(tempo);

  if(print) {
    Serial.print("Setando posicao no servo Pe Esquerda..."); 
    Serial.println(posicao[PEESQ]);
  }
  servo[PEESQ].write(posicao[PEESQ]);
  vTaskDelay(tempo);

  if(print) {
    Serial.print("Setando posicao no servo Quadril Direita..."); 
    Serial.println(posicao[QUADDIR]); 
  }
  servo[QUADDIR].write(posicao[QUADDIR]);
  vTaskDelay(tempo);

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

// Executar essa funcao toda vez que um loop de andar for terminado, para retornar os quadris a sua posicao neutra
// APENAS DEVE SER EXECUTADO DAS FUNCOES JA TEREM SIDO ATUALIZADAS 
void retornaPosicaoPadrao() {
  int qtde_iteracoes = 10; 
  int intervalo = 100;
  moveUmServoSuavemente(QUADDIR,&pos_cur_quaddir,pos_wait_quadrildireito,qtde_iteracoes,intervalo); 
  moveUmServoSuavemente(QUADESQ,&pos_cur_quadesq,pos_wait_quadrilesquerdo,qtde_iteracoes,intervalo);
  moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes,intervalo); 
  moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes,intervalo);
  moveUmServoSuavemente(GARRADIR,&pos_garra_dir,pos_wait_garra_direita,qtde_iteracoes,intervalo); 
  moveUmServoSuavemente(GARRAESQ,&pos_garra_esq,pos_wait_garra_esquerda,qtde_iteracoes,intervalo);  
}

// Bate as garras por uma certa quantidade de vezes fornecida 
void bateGarras(int qtdeVezes) {
  int intervalo = 100;
  int qtde_iteracoes = 10; 
  
  // Move as garras para a posicao padrao
  MoveDoisServosSuavemente(GARRAESQ,GARRADIR,pos_garra_dir,pos_wait_garra_direita,qtde_iteracoes,intervalo); 
  pos_garra_dir = pos_wait_garra_direita;
  pos_garra_esq = pos_wait_garra_direita; 

  for(int j=0; j<qtdeVezes; j++) {
    MoveDoisServosSuavemente(GARRAESQ,GARRADIR,pos_garra_dir,ang_open_garra,qtde_iteracoes,intervalo); 
    pos_garra_dir = ang_open_garra;
    pos_garra_esq = ang_open_garra;
    MoveDoisServosSuavemente(GARRAESQ,GARRADIR,pos_garra_dir,ang_closed_garra,qtde_iteracoes,intervalo); 
    pos_garra_dir = ang_closed_garra;
    pos_garra_esq = ang_closed_garra; 
  }
  
  // Move as garras para a posicao padrao 
  MoveDoisServosSuavemente(GARRAESQ,GARRADIR,pos_garra_dir,pos_wait_garra_direita,qtde_iteracoes,intervalo); 
  pos_garra_dir = pos_wait_garra_direita;
  pos_garra_esq = pos_wait_garra_direita;
}

// Oscila para a direita e para a esquerda uma certa quantidade de vezes 
void oscilaLados(int qtdeOscilacoes) {
  // Variaveis para regular a posicao dos servos 
  int pos_maxima_pedir =  20;
  int pos_maxima_peesq = 140;
  int incremento = 10; 
  int intervalo = 10; 
  int qtde_iteracoes = 10; 

  // Executa as duas oscilacoes 
  for(int i = 0; i<qtdeOscilacoes; i++) {

    /* Direita para cima */
    moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_maxima_peesq,qtde_iteracoes, intervalo);
    
    /* Retorna as posicoes padroes */
    moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes, intervalo);
    
    /* Esquerda para cima */ 
    moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_maxima_pedir,qtde_iteracoes, intervalo); 
    
    /* Retorna as posicoes padroes */
    moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes, intervalo); 
  } 
}


// ESSA FUNCAO TEM COMO FONTE O MODO "MOONWALK" DO VIDEO https://www.youtube.com/watch?v=VD6sgTo6NOY
void UmSwing() {
  /* Valores ja definidos */ 
  int intervalo = 50;          /* Intervalo de tempo entre os passos */
  int qtde_iteracoes = 5;
  int pos_max_pedir = 40; 
  int pos_max_peesq = 105; 

  // ----------------------------- PRIMEIRA PARTE: Movimento comeca pela parte direita 
  /* Movimento ascendente do pe direito */
  moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_max_pedir,qtde_iteracoes,intervalo); 

  /* Movimento ascendente do pe esquerdo */
  moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_max_peesq,qtde_iteracoes,intervalo); 

  /* Movimento descendente do pe esquerdo */
  moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes,intervalo);

  /* Movimento descendente do pe direito */
  moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes,intervalo);
  
  // ------------------------------ SEGUNDA PARTE: Movimento comeca pela parte esquerda 
  /* Movimento ascendente do pe esquerdo */
  moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_max_peesq,qtde_iteracoes,intervalo); 

  /* Movimento ascendente do pe direito */
  moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_max_pedir,qtde_iteracoes,intervalo); 

  /* Movimento descendente do pe direito */
  moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes,intervalo);

  /* Movimento descendente do pe esquerdo */
  moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes,intervalo);
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
  int intervalo = 50;          /* Intervalo de tempo entre os passos */
  int pos_max_quadril_direita = 105;    /* A posicao minima deve ser a posicao padrao */
  int pos_max_quadril_esquerda = 75; 
  int pos_max_pe_direito = 40;
  int pos_max_pe_esquerdo = 105;  
  /* Espera-se que as posicoes minimas para ambos os pes sejam a mesma, que vai ser a posicao padrao. Sendo minimas as posicoes de menor diferenca em relacao ao solo.*/
  int pos_min_quadrildireito = pos_wait_quadrildireito; // DEPOIS REVER  SE PRECISA DESSA PARTE -> TALVEZ SO ALTERNE ENTRE AS POSICOES MAXIMAS E AS PADROES 
  int pos_min_quadrilesquerdo = pos_wait_quadrilesquerdo; 
  int qtde_iteracoes = 10;

  /* ----------------------------------------- MOVIMENTO DA PARTE DIREITA ------------------------------------------ */
  // Movimento do pe direito para a suspensao 
  moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_max_pe_direito,qtde_iteracoes,intervalo);

  // Movimentar o quadril direito para a posicao maxima 
  moveUmServoSuavemente(QUADDIR,&pos_cur_quaddir,pos_max_quadril_direita,qtde_iteracoes,intervalo); 

  // Movimento do pe direito para retorno para a posicao padrao 
  moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes,intervalo);

  // Retornar o quadril esquerdo para a posicao padrao 
  moveUmServoSuavemente(QUADESQ,&pos_cur_quadesq,pos_wait_quadrilesquerdo,qtde_iteracoes,intervalo); 

  /* ----------------------------------------------- MOVIMENTO DA PARTE ESQUERDA ------------------------------------------------- */
  // Movimentar o pe esquerdo para a suspensao 
  moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_max_pe_esquerdo,qtde_iteracoes,intervalo); 

  // Movimentar o quadril esquerdo para a posicao maxima 
  moveUmServoSuavemente(QUADESQ,&pos_cur_quadesq,pos_max_quadril_esquerda,qtde_iteracoes,intervalo);

  //Movimento do pe esquerdo para retorno para a posicao padrao 
  moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes,intervalo); 
  
  // Move quadril direito para a posicao padrao 
  moveUmServoSuavemente(QUADDIR,&pos_cur_quaddir,pos_wait_quadrildireito,qtde_iteracoes,intervalo);
}

/* ------------------ FUNCAO DE DESVIAR PARA UM LADO ---------------------- */
/* 
Desvia para o lado especificado em 90 graus. Apos isso precisaria realizar a verificacao antes de iniciar o andar. 
Video de referencia: https://www.youtube.com/watch?v=VD6sgTo6NOY      - Minuto a

*/
void DesviaUmLado(int side) {
  int intervalo = 50;          /* Intervalo de tempo entre os passos */
  int pos_max_quadril_direita = 105;    /* A posicao minima deve ser a posicao padrao */
  int pos_max_quadril_esquerda = 75; 
  int pos_max_pe_direito = 40;
  int pos_max_pe_esquerdo = 105;  
  /* Espera-se que as posicoes minimas para ambos os pes sejam a mesma, que vai ser a posicao padrao. Sendo minimas as posicoes de menor diferenca em relacao ao solo.*/
  int pos_min_quadrildireito = pos_wait_quadrildireito; // DEPOIS REVER  SE PRECISA DESSA PARTE -> TALVEZ SO ALTERNE ENTRE AS POSICOES MAXIMAS E AS PADROES 
  int pos_min_quadrilesquerdo = pos_wait_quadrilesquerdo; 
  int qtde_iteracoes = 10;

  switch(side) {
    case ESQUERDA:  
    /* ----------------------------------------- MOVIMENTO DA PARTE DIREITA ------------------------------------------ */
    // Movimento do pe direito para a suspensao 
    moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_max_pe_direito,qtde_iteracoes,intervalo);

    // Movimentar o quadril direito para a posicao maxima 
    moveUmServoSuavemente(QUADDIR,&pos_cur_quaddir,pos_max_quadril_direita,qtde_iteracoes,intervalo); 

    // Movimento do pe direito para retorno para a posicao padrao 
    moveUmServoSuavemente(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes,intervalo);

    /* ----------------------------------------------- MOVIMENTO DA PARTE ESQUERDA ------------------------------------------------- */
    // Move quadril direito para a posicao padrao 
    moveUmServoSuavemente(QUADDIR,&pos_cur_quaddir,pos_wait_quadrildireito,qtde_iteracoes,intervalo);
    break; 
  case DIREITA: 
    /* ----------------------------------------- MOVIMENTO DA PARTE DIREITA ------------------------------------------ */
    // Retornar o quadril esquerdo para a posicao padrao 
    moveUmServoSuavemente(QUADESQ,&pos_cur_quadesq,pos_wait_quadrilesquerdo,qtde_iteracoes,intervalo); 

    /* ----------------------------------------------- MOVIMENTO DA PARTE ESQUERDA ------------------------------------------------- */
    // Movimentar o pe esquerdo para a suspensao 
    moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_max_pe_esquerdo,qtde_iteracoes,intervalo); 

    // Movimentar o quadril esquerdo para a posicao maxima 
    moveUmServoSuavemente(QUADESQ,&pos_cur_quadesq,pos_max_quadril_esquerda,qtde_iteracoes,intervalo);

    //Movimento do pe esquerdo para retorno para a posicao padrao 
    moveUmServoSuavemente(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes,intervalo); 
  break; 
  }
}

/* ------------------------------- ####################### FUNCOES MODULARES PARA MEXER OS SERVOS ##################### --------------------------------------- */

// Move o servo suavemente de uma posicao para outra. 
void moveUmServoSuavemente(int s, int *pos_inicial, int pos_final, int qtde_iteracoes, int intervalo) {
  int incremento = (pos_final - *pos_inicial)/qtde_iteracoes; 
  int pos_cur = *pos_inicial; 

  for(int i=0; i<qtde_iteracoes; i++) {
    if(incremento<0 && pos_cur<=pos_final) break; 
    if(incremento>=0 && pos_cur>=pos_final) break; 
    servo[s].write(pos_cur);
    pos_cur += incremento; 
    vTaskDelay(intervalo); 
  }
  servo[s].write(pos_final);
  *pos_inicial = pos_final; 
}

// OBS: A POSICAO PRECISA SER ATUALIZADA NA FUNCAO QUE E CHAMADA
// Dois servos movem nas mesmas posicoes e taxas 
void MoveDoisServosSuavemente(int s1, int s2, int pos_inicial, int pos_final, int qtde_iteracoes, int intervalo) {
  int incremento = (pos_final - pos_inicial)/qtde_iteracoes; 
  int pos_cur = pos_inicial; 

  for(int i=0; i<qtde_iteracoes; i++) {
    if(incremento<=0 && pos_cur<=pos_final) break; 
    if(incremento>=0 && pos_cur>=pos_final) break; 
    servo[s1].write(pos_cur);
    servo[s2].write(pos_cur);
    pos_cur += incremento; 
    vTaskDelay(intervalo); 
  }
  servo[s1].write(pos_final);
  servo[s2].write(pos_final); 
}


// -------------------------------------- ################## FUNCOES DE SENSORES ################### ------------------------------------------
void detectaDistancia() {
  int duration; 
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
  luminosidade = analogRead(ldr);
  if(print) {
    Serial.print("Luminosidade detectada: ");
    Serial.println(luminosidade);
  } 
  vTaskDelay(10);
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
    vTaskDelay(tempo);
  }
  // Cor Verde 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {         
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(0, 255, 0));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(0, 255, 0));
    ws2812bONE.show();                                     
    ws2812bTWO.show();
    vTaskDelay(tempo); 
  }
  // Cor Azul 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {        
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(0, 0, 255));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(0, 0, 255)); 
    ws2812bONE.show();                                          
    ws2812bTWO.show(); 
    vTaskDelay(tempo);
  }
  // Cor Branca 
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {        
    ws2812bONE.setPixelColor(pixel, ws2812bONE.Color(255, 255, 255));  
    ws2812bTWO.setPixelColor(pixel, ws2812bTWO.Color(255, 255, 255)); 
    ws2812bONE.show();                                          
    ws2812bTWO.show(); 
    vTaskDelay(tempo);
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
  corAtual = corLigar;
}