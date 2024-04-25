#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Definições
#define STOP_OFFSET 300
#define MPU_ADDR 0x68

// Variáveis globais
unsigned long t_amostra = 0;
unsigned long t_parado = 0;
bool parado = true;
float dist = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Inicializa o MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Falha na conexão do MPU6050");
  } else {
    Serial.println("MPU6050 conectado");
  }

  // Calibração
  // Comente esta linha se você não sabe os valores do seu sensor
  mpu.setXAccelOffset(-2267);
  mpu.setYAccelOffset(-1111);
  mpu.setZAccelOffset(504);

  t_amostra = micros();
}

void loop() {
  // Leitura da aceleração no eixo Y
  int16_t ac_y = mpu.getAccelerationY();
  float aux_ac = float(ac_y);

  // Detecção de objeto parado
  if (fabs(aux_ac) < STOP_OFFSET && abs(long(millis() - t_parado)) > 50) {
    parado = true;
    t_amostra = micros();
  } else if (fabs(aux_ac) >= STOP_OFFSET) {
    t_parado = millis();
    parado = false;
  }

  // Conversão da aceleração para m/s^2
  aux_ac = ((aux_ac + 32768.0) * 4.0 / 65536.0 - 2.0) * 9.81;

  // Cálculo da distância
  if (parado) {
    if (dist != 0.0) {
      Serial.println("Distância deslocada (em y): ");
      Serial.print(dist * 100, 2); // Convertendo para centímetros
      Serial.println(" cms em y.");
    }
    dist = 0.0;
  } else {
    t_amostra = micros() - t_amostra;
    dist = calculo_trapezio(dist, aux_ac, t_amostra);
    t_amostra = micros();
  }
}

float calculo_trapezio(float dist, float acel, unsigned long tempo) {
  static float last_acel = 0.0;
  static float last_vel = 0.0;
  float vel;
  float t = (float)tempo / 1000000.0;

  if (dist == 0.0) {
    last_vel = 0.0;
    last_acel = 0.0;
  }

  vel = last_vel + (last_acel + acel) * t / 2.0;
  dist = dist + (last_vel + vel) * t / 2.0;

  last_acel = acel;
  last_vel = vel;

  return dist;
}
