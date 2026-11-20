/**
 * @file quatron_servo_driver.ino
 * @brief Controlador de atuação dos 12 servo motores do robô quadrúpede Quatron-V0.
 *
 * Recebe pacotes CSV contendo 12 ângulos (em graus, de 0 a 180) via porta serial
 * enviados pelo MATLAB a 115200 bps e aciona os canais 0 a 11 do módulo PWM
 * Adafruit PCA9685 (alimentado por fonte externa de 6V).
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <stdlib.h>
#include <LiquidCrystal.h>

// Configuração opcional de display LCD (pinos digitais do Arduino Mega)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// --- Protótipos das Funções ---
uint16_t limTicks(float us);
uint16_t degreeToPwm(int degree);
uint16_t degreeToPwmInverse(int degree);
void anglesRecived(char *data, uint16_t *angles, uint8_t size);
void sendingAnglesToServos(uint16_t *angles);

// --- Definições de Hardware e Calibração PWM (PCA9685 @ 50 Hz) ---
#define NUM_SERVOS 12
#define NUM_ANGLES 12
#define SERVOMIN 172  // Pulso mínimo em ticks (~0°)
#define SERVOMAX 530  // Pulso máximo em ticks (~180°)
#define ZERO_POSITION_TICK static_cast<int>((SERVOMIN + SERVOMAX) / 2) // Posição neutra (90°)

// Buffer para armazenar os 12 ângulos recebidos (3 juntas x 4 pernas)
uint16_t angles[NUM_ANGLES];

// Mapeamento dos canais do PCA9685:
// [0..2]: Frente-Direita | [3..5]: Frente-Esquerda | [6..8]: Trás-Direita | [9..11]: Trás-Esquerda
uint8_t servoPins[NUM_SERVOS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

// Instância do driver I2C Adafruit PCA9685 (endereço padrão 0x40)
Adafruit_PWMServoDriver pwm;

void setup() {
  Serial.begin(115200); // Mesma taxa de transmissão (baud rate) configurada no MATLAB
  
  pwm.begin();
  pwm.setPWMFreq(50);   // Frequência padrão de operação para servo motores analógicos/digitais (50 Hz)

  // Posiciona todos os 12 servo motores em 90° (postura neutra de inicialização)
  for (int j = 0; j < NUM_SERVOS; j++) {
    pwm.setPWM(servoPins[j], 0, ZERO_POSITION_TICK);
  }
}

void loop() {
  // Aguarda o recebimento de uma linha completa terminada em '\n' (LF)
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    char buffer[100]; // Buffer auxiliar para tokenização da string CSV
    data.toCharArray(buffer, sizeof(buffer));

    // Extrai os 12 ângulos da string e atualiza os sinais PWM no PCA9685
    anglesRecived(buffer, angles, NUM_ANGLES);
    sendingAnglesToServos(angles);
  }
}

/**
 * @brief Converte um ângulo em graus (0 a 180) para ticks PWM do PCA9685 (sentido direto).
 */
uint16_t degreeToPwm(int degree) {
  degree = constrain(degree, 0, 180);
  return map(degree, 0, 180, SERVOMIN, SERVOMAX);
}

/**
 * @brief Converte um ângulo em graus (0 a 180) para ticks PWM invertidos (180 a 0).
 *        Necessário para juntas montadas de forma espelhada no chassi do Quatron-V0.
 */
uint16_t degreeToPwmInverse(int degree) {
  degree = constrain(degree, 0, 180);
  return map(degree, 180, 0, SERVOMIN, SERVOMAX);
}

/**
 * @brief Converte largura de pulso em microssegundos para ticks de 12 bits (4096) a 50 Hz.
 */
uint16_t limTicks(float us) {
  return (uint16_t)((us * 4096) / 20 * pow(10, 3));
}

/**
 * @brief Faz o parsing de uma string CSV ("a0,a1,...,a11") para o vetor de ângulos.
 */
void anglesRecived(char *data, uint16_t *angles, uint8_t size) {
  char *pt = strtok(data, ",");
  uint8_t i = 0;
  while (pt != NULL && i < size) {
    angles[i] = atoi(pt);
    pt = strtok(NULL, ",");
    i++;
  }
}

/**
 * @brief Envia os sinais PWM aos 12 servos respeitando a orientação mecânica de cada junta.
 */
void sendingAnglesToServos(uint16_t *angles) {
  for (int j = 0; j < NUM_ANGLES; j++) {
    // Pernas do lado direito: Frente-Direita (0..2) e Trás-Direita (6..8)
    if ((j >= 0 && j < 3) || (j > 5 && j < 9)) {
      if (j == 0) {
        pwm.setPWM(servoPins[0], 0, degreeToPwm(angles[0]));
      } else {
        pwm.setPWM(servoPins[j], 0, degreeToPwmInverse(angles[j]));
      }
    }

    // Pernas do lado esquerdo: Frente-Esquerda (3..5) e Trás-Esquerda (9..11)
    if ((j > 2 && j < 6) || (j >= 9 && j <= 11)) {
      if (j == 9) {
        pwm.setPWM(servoPins[j], 0, degreeToPwmInverse(angles[j]));
      } else {
        pwm.setPWM(servoPins[j], 0, degreeToPwm(angles[j]));
      }
    }
  }
}
