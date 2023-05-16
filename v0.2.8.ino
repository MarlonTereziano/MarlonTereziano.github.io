#include <Arduino.h>
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

#define TRIGGER_PIN 12
#define ECHO_PIN 13
#define MAX_DISTANCE 200
// Pinos de conexão do módulo DFPlayer Mini ao Arduino
#define RX_PIN 3
#define TX_PIN 5

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
const int distanciaMinima = 30;  // Distância mínima em centímetros para acionar a virada

//Giro sensor de presença
const unsigned long TEMPO_LIMITE_GIRO = 500;

//Pinos para chave seletora
const int switchPinA = 6;
const int switchPinB = 7;

// Portas do Arduino para a ponte H MX1608
const int motorA1 = 8;
const int motorA2 = 9;
const int motorB1 = 10;
const int motorB2 = 11;

// Portas do Arduino para os sensores de linha
const int sensorEsquerdo = 2;
const int sensorDireito = 4;

// Velocidades iniciais dos motores
int velocidadeMotorA = 128;  // Ajuste este valor conforme necessário (0 a 255)
int velocidadeMotorB = 128;  // Ajuste este valor conforme necessário (0 a 255)

// Variável para armazenar o último sensor que saiu da linha
int ultimoSensorSaiu = 0;  // 0 para nenhum, 1 para esquerdo e 2 para direito

// Configurando a comunicação serial entre o Arduino e o módulo DFPlayer Mini
SoftwareSerial mySoftwareSerial(RX_PIN, TX_PIN);
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  mySoftwareSerial.begin(9600);

  Serial.println(F("Inicializando o DFPlayer Mini..."));
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("Não foi possível inicializar o DFPlayer Mini!"));
    while (1)
      ;
  }
  Serial.println(F("DFPlayer Mini inicializado com sucesso."));
  myDFPlayer.setTimeOut(1000);  // Define o tempo limite da comunicação serial em 500ms
  myDFPlayer.volume(20);        // Define o volume (0-30)

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(sensorEsquerdo, INPUT);
  pinMode(sensorDireito, INPUT);
  pinMode(switchPinA, INPUT_PULLUP);
  pinMode(switchPinB, INPUT_PULLUP);


  uint8_t folder = 1;
  uint8_t file = 1;
  myDFPlayer.playFolder(folder, file);


  Serial.begin(9600);
}


void loop() {
  int switchStateA = digitalRead(switchPinA);
  int switchStateB = digitalRead(switchPinB);

  int estadoEsquerdo = digitalRead(sensorEsquerdo);
  int estadoDireito = digitalRead(sensorDireito);

  int distancia = sonar.ping_cm();

  if (switchStateA == LOW && switchStateB == HIGH) {
    // Chave na posição A
    // Serial.println("Chave na posição A");
    velocidadeMotorA = 128;
    velocidadeMotorB = 128;


    if (estadoEsquerdo && estadoDireito) {
      // Ambos os sensores estão sobre a linha
      moverParaFrente();
    } else if (estadoEsquerdo && !estadoDireito) {
      // Sensor esquerdo sobre a linha, sensor direito fora da linha
      girarEsquerda();
      ultimoSensorSaiu = 1;
    } else if (!estadoEsquerdo && estadoDireito) {
      // Sensor esquerdo fora da linha, sensor direito sobre a linha
      girarDireita();
      ultimoSensorSaiu = 2;
    } else {
      // Ambos os sensores estão fora da linha
      if (ultimoSensorSaiu == 1) {
        girarEsquerda();
      } else if (ultimoSensorSaiu == 2) {
        girarDireita();
      } else {
        parar();
      }
    }
  } else if (switchStateA == HIGH && switchStateB == LOW) {
    // Chave na posição B
    // Serial.println("Chave na posição B");
    // Execute o código para a posição B
    velocidadeMotorA = 150;
    velocidadeMotorB = 210;

    if (distancia > 0 && distancia < distanciaMinima) {
      virarCarrinho();
    } else {
      moverParaFrente();
    }
  } else {
    // Chave na posição OFF (desligado)
    // Serial.println("Chave desligada");
    // Execute o código para a posição OFF
    parar();
  }
}

void moverParaFrente() {
  analogWrite(motorA1, velocidadeMotorA);
  digitalWrite(motorA2, LOW);
  analogWrite(motorB1, velocidadeMotorB);
  digitalWrite(motorB2, LOW);
}

void girarEsquerda() {
  digitalWrite(motorA1, LOW);
  analogWrite(motorA2, velocidadeMotorA);
  analogWrite(motorB1, velocidadeMotorB);
  digitalWrite(motorB2, LOW);
}

void girarDireita() {
  analogWrite(motorA1, velocidadeMotorA);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  analogWrite(motorB2, velocidadeMotorB);
}

void parar() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void virarCarrinho() {
  unsigned long tempoGiro = millis();

  while (millis() - tempoGiro < TEMPO_LIMITE_GIRO) {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, velocidadeMotorA);
    digitalWrite(motorB1, velocidadeMotorB);
    digitalWrite(motorB2, LOW);
  }

  moverParaFrente();  // Continue movendo o carrinho para frente após virar
}