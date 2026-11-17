/*
Código do Sirigueijo sem dependências de ROS, para usar só com ESP
Quando ligado, fica parado com os olhos acesos na cor branca
Ao passar a mão na frente do ultrassônico, faz uma dancinha e pisca os olhos colorido
*/

#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// PINOS 
// Módulo LED 
#define PIN_WS2812BONE  13
#define PIN_WS2812BTWO  14
#define NUM_PIXELS      8          //The number of LEDs (pixels) on WS2812B LED strip

// Ultrassonico
#define echoPin         26
#define trigPin         27

// SERVOS
// Pinos 
#define QUADDIR_PIN     18
#define PEDIR_PIN       19
#define QUADESQ_PIN     21
#define PEESQ_PIN       22

// Indices no vetor de servo 
#define QUADDIR     0 
#define PEDIR       1
#define QUADESQ     2
#define PEESQ       3
 
// VARIAVEIS 
// MODULO LED 

Adafruit_NeoPixel ws2812bONE(NUM_PIXELS, PIN_WS2812BONE, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ws2812bTWO(NUM_PIXELS, PIN_WS2812BTWO, NEO_GRB + NEO_KHZ800);

// CONSTANTES GERAIS 
#define SOUND_SPEED 0.034
#define BAUD_RATE 115200

// VARIAVEIS PARA REGULAR OS SERVOS
Servo servo[4]; 

// ANGULOS DE ESPERA DOS SERVOMOTORES 
// Posicao padrao para os pes 
#define pos_padrao_esquerdo 70                  
#define pos_padrao_direito  75                  
#define pos_wait_quadrildireito 85
#define pos_wait_quadrilesquerdo 90             

// CORES
#define AZUL 0
#define VERMELHO 1
#define VERDE 2
#define BRANCO 3 

// distancias para acionar a dancinha pelo ultrassônico
#define DISTANCIA_ACIONAMENTO_CM 15
#define DISTANCIA_LIBERACAO_CM   25

// ESTADOS A SEREM ATUALIZADOS:

int pos_cur_pedir;  
int pos_cur_quaddir; 
int pos_cur_peesq;
int pos_cur_quadesq; 

int indiceCorDancinha = 0; 
volatile bool modoDancinhaAtivo = false;

void setup() {
  Serial.begin(BAUD_RATE);

  // Inicializa componentes
  ws2812bONE.begin();
  ws2812bTWO.begin();

  ws2812bONE.setBrightness(10);
  ws2812bTWO.setBrightness(10);

  setCor(BRANCO);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  servo[QUADDIR].attach(QUADDIR_PIN);
  servo[PEDIR].attach(PEDIR_PIN);
  servo[QUADESQ].attach(QUADESQ_PIN);
  servo[PEESQ].attach(PEESQ_PIN);

  setPosicaoPadrao();
  
  // Inicializa as tasks para utilizar o DualCore 
  xTaskCreatePinnedToCore(Task1code, "Task1", 8192, NULL, 2, NULL, 0);                           
  xTaskCreatePinnedToCore(Task2code, "Task2", 8192, NULL, 2, NULL, 1);   

}

// nao faz nada
void loop(){
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// Task1code: controla o ultrassônico
void Task1code(void *pvParameters){
    // Indica se o sensor está preparado para detectar uma mão
    bool sensorArmado = false;
    // Quantidade de leituras consecutivas indicando que o sensor
    // está livre para detectar uma nova passagem
    int leiturasLivres = 0;
    while (true) {
        float distancia = detectaDistanciaV2();
        // Verifica se não há nenhum objeto próximo
        if (distancia > DISTANCIA_LIBERACAO_CM) {
            if (leiturasLivres < 3) {
                leiturasLivres++;
            }
            // Rearma o sensor após três leituras livres
            if (leiturasLivres >= 3) {
                sensorArmado = true;
            }
        }
        else {
            leiturasLivres = 0;
            // Detectou uma mão e o sensor estava preparado
            if (distancia <= DISTANCIA_ACIONAMENTO_CM && sensorArmado) {
                // Alterna o estado da dancinha
                modoDancinhaAtivo = !modoDancinhaAtivo;
                // Impede novas ativações enquanto a mão permanecer próxima
                sensorArmado = false;
                if (modoDancinhaAtivo) {
                    Serial.println("Dancinha ativada!");
                }
                else {
                    Serial.println("Dancinha desativada!");
                }
            }
        }
        // Aguarda antes de realizar uma nova leitura
        vTaskDelay(pdMS_TO_TICKS(60));
    }
}

//Task2code: controla a dancinha e led dos olhos
void Task2code( void * pvParameters ){
    bool precisaRetornar = false;
    while (true) {
        // Verifica se a dancinha está ativada
        if (modoDancinhaAtivo) {
            precisaRetornar = true;
            // Atualiza a cor dos olhos antes do movimento
            setCor(indiceCorDancinha);
            // Alterna entre AZUL, VERMELHO e VERDE
            indiceCorDancinha = (indiceCorDancinha + 1) % 3;
            // Executa um ciclo da dancinha
            UmSwingInterrompivelDancinha();
        }
        else {
            // Se a dancinha acabou de ser desativada
            if (precisaRetornar) {
                // Os olhos ficam brancos imediatamente
                setCor(BRANCO);
                // Retorna os servos à posição padrão
                retornaPosicaoPadrao();
                precisaRetornar = false;
            }
            // Aguarda antes de verificar novamente
            vTaskDelay(pdMS_TO_TICKS(20));
        }
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
  
  Serial.print("Setando posicao no servo Pe Direita...");
  Serial.println(pos_padrao_direito); 
  
  servo[PEDIR].write(pos_padrao_direito);
  vTaskDelay(pdMS_TO_TICKS(tempo));

  Serial.print("Setando posicao no servo Pe Esquerda..."); 
  Serial.println(pos_padrao_esquerdo);
  
  servo[PEESQ].write(pos_padrao_esquerdo);
  vTaskDelay(pdMS_TO_TICKS(tempo));

  Serial.print("Setando posicao no servo Quadril Direita..."); 
  Serial.println(pos_wait_quadrildireito); 
  
  servo[QUADDIR].write(pos_wait_quadrildireito);
  vTaskDelay(pdMS_TO_TICKS(tempo));

  Serial.print("Setando posicao no servo Quadril Esquerda...");  
  Serial.println(pos_wait_quadrilesquerdo); 
  
  servo[QUADESQ].write(pos_wait_quadrilesquerdo);
  vTaskDelay(pdMS_TO_TICKS(tempo));

}

// Executar essa funcao toda vez que uma animacao terminar, para retornar os quadris a sua posicao neutra
// APENAS DEVE SER EXECUTADO DAS FUNCOES JA TEREM SIDO ATUALIZADAS 
void retornaPosicaoPadrao() {
  int qtde_iteracoes = 10; 
  int intervalo = 100;
  moveUmServoSuavementeV2(QUADDIR,&pos_cur_quaddir,pos_wait_quadrildireito,qtde_iteracoes,intervalo); 
  moveUmServoSuavementeV2(QUADESQ,&pos_cur_quadesq,pos_wait_quadrilesquerdo,qtde_iteracoes,intervalo);
  moveUmServoSuavementeV2(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes,intervalo); 
  moveUmServoSuavementeV2(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes,intervalo);
}

// UmSwing(), porem pode ser interrompido no meio da dancinha se a dancinha for desativada
void UmSwingInterrompivelDancinha() {
  /* Valores ja definidos */ 
  int intervalo = 50;          /* Intervalo de tempo entre os passos */
  int qtde_iteracoes = 5;
  int pos_max_pedir = 40; 
  int pos_max_peesq = 105; 

  // ----------------------------- PRIMEIRA PARTE: Movimento comeca pela parte direita 
  /* Movimento ascendente do pe direito */
  moveUmServoSuavementeInterrompivelDancinha(PEDIR,&pos_cur_pedir,pos_max_pedir,qtde_iteracoes,intervalo); 
  if (!modoDancinhaAtivo) return;

  /* Movimento ascendente do pe esquerdo */
  moveUmServoSuavementeInterrompivelDancinha(PEESQ,&pos_cur_peesq,pos_max_peesq,qtde_iteracoes,intervalo); 
  if (!modoDancinhaAtivo) return;

  /* Movimento descendente do pe esquerdo */
  moveUmServoSuavementeInterrompivelDancinha(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes,intervalo);
  if (!modoDancinhaAtivo) return;

  /* Movimento descendente do pe direito */
  moveUmServoSuavementeInterrompivelDancinha(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes,intervalo);
  if (!modoDancinhaAtivo) return;

  // ------------------------------ SEGUNDA PARTE: Movimento comeca pela parte esquerda 
  /* Movimento ascendente do pe esquerdo */
  moveUmServoSuavementeInterrompivelDancinha(PEESQ,&pos_cur_peesq,pos_max_peesq,qtde_iteracoes,intervalo); 
  if (!modoDancinhaAtivo) return;

  /* Movimento ascendente do pe direito */
  moveUmServoSuavementeInterrompivelDancinha(PEDIR,&pos_cur_pedir,pos_max_pedir,qtde_iteracoes,intervalo); 
  if (!modoDancinhaAtivo) return;

  /* Movimento descendente do pe direito */
  moveUmServoSuavementeInterrompivelDancinha(PEDIR,&pos_cur_pedir,pos_padrao_direito,qtde_iteracoes,intervalo);
  if (!modoDancinhaAtivo) return;

  /* Movimento descendente do pe esquerdo */
  moveUmServoSuavementeInterrompivelDancinha(PEESQ,&pos_cur_peesq,pos_padrao_esquerdo,qtde_iteracoes,intervalo);
  if (!modoDancinhaAtivo) return;
}

/* ------------------------------- ####################### FUNCOES MODULARES PARA MEXER OS SERVOS ##################### --------------------------------------- */

// Move o servo suavemente de uma posicao para outra. 
void moveUmServoSuavementeV2(int s, int *pos_inicial, int pos_final, int qtde_iteracoes, int intervalo) {
    if (qtde_iteracoes <= 0) {
        servo[s].write(pos_final);
        *pos_inicial = pos_final;
        return;
    }
    int pos_inicio = *pos_inicial;
    // Não perde tempo caso o servo já esteja na posição desejada
    if (pos_inicio == pos_final) {
        return;
    }
    for (int i = 1; i <= qtde_iteracoes; i++) {
        // Calcula a posição intermediária
        int posicao = pos_inicio +
            ((pos_final - pos_inicio) * i) / qtde_iteracoes;
        servo[s].write(posicao);
        // Atualiza a última posição comandada
        *pos_inicial = posicao;
        vTaskDelay(pdMS_TO_TICKS(intervalo));
    }
}

// Move o servo suavemente de uma posicao para outra. Pode ser interrompido no meio do modo dancinha.
void moveUmServoSuavementeInterrompivelDancinha(int s, int *pos_inicial, int pos_final, int qtde_iteracoes, int intervalo) {
    if (qtde_iteracoes <= 0) {
        return;
    }
    int pos_inicio = *pos_inicial;
    for (int i = 1; i <= qtde_iteracoes; i++) {
        // Verifica se a dancinha continua ativa
        if (!modoDancinhaAtivo) {
            return;
        }
        // Calcula a posição intermediária
        int posicao = pos_inicio + ((pos_final - pos_inicio) * i) / qtde_iteracoes;
        // Envia a posição ao servo
        servo[s].write(posicao);
        // Atualiza a posição realmente comandada
        *pos_inicial = posicao;
        vTaskDelay(pdMS_TO_TICKS(intervalo));
    }
}

// -------------------------------------- ################## FUNCOES DE SENSORES ################### ------------------------------------------

// detecta distancia sem mexer em variavel global e sem usar vTaskDelay
float detectaDistanciaV2() {
    unsigned long duration;
    // Envia o pulso de disparo
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH, 30000UL); // Timeout de 30 ms

    // Nenhum eco recebido
    if (duration == 0) {
        return -1.0;
    }

    // Calcula a distância em centímetros
    float distancia = duration * SOUND_SPEED / 2.0;
    
    //Serial.print("Distancia detectada no ultrassonico: ");
    //Serial.println(distancia);
    
    return distancia;
}


// -------------------------------------- ################## FUNCOES DO MODULO LED ################### ------------------------------------------

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

  }

  ws2812bONE.show(); 
  ws2812bTWO.show();
}