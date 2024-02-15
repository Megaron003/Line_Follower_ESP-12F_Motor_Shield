int motorA_EN = D1;
int motorA_DIR = D3;
int motorB_EN = D2;
int motorB_DIR = D4;

const int sensorPin2 = D5;  // Pino do terceiro sensor IR (D5)
const int sensorPin3 = D6;  // Pino do quarto sensor IR (D6)
const int sensorPin4 = D7;  // Pino do quinto sensor IR (D7)

// Definindo os parâmetros do PID
double kp = 1.0;
double ki = 0.1;
double kd = 0.2;

// Definindo o ajuste para a linha central
double TARGET = 1.0;

// Definindo o intervalo de tempo para o ciclo de controle
unsigned long dt = 100; // em milissegundos

// Variáveis globais
double previous_error = 0;
double integral = 0;

int MAX_SPEED = 255;
int MIN_SPEED = 100;

void setup() {
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Teste de motores iniciando...");
  
  pinMode(motorA_EN, OUTPUT);
  pinMode(motorA_DIR, OUTPUT);
  pinMode(motorB_EN, OUTPUT);
  pinMode(motorB_DIR, OUTPUT);

  pinMode(sensorPin2, INPUT);
  pinMode(sensorPin3, INPUT);
  pinMode(sensorPin4, INPUT);
}

void loop() {
  int value2 = digitalRead(sensorPin2);
  int value3 = digitalRead(sensorPin3);
  int value4 = digitalRead(sensorPin4);
  
  // Calculando o erro
  double error = value3 - TARGET;

  // Calculando o termo integral
  integral += error * dt;

  // Calculando o termo derivativo
  double derivative = (error - previous_error) / dt;

  // Calculando a saída do PID
  double output = kp * error + ki * integral + kd * derivative;

  // Ajustando a velocidade dos motores de acordo com a saída do PID
  int left_speed = constrain(MAX_SPEED - output, MIN_SPEED, MAX_SPEED);
  int right_speed = constrain(MAX_SPEED + output, MIN_SPEED, MAX_SPEED);

  // Definindo a direção dos motores
  if (value2 == HIGH && value3 == LOW && value4 == HIGH) {
    // Se todos os sensores estiverem sobre a linha, siga em frente
    digitalWrite(motorA_EN, HIGH);
    digitalWrite(motorA_DIR, HIGH);
    digitalWrite(motorB_EN, HIGH);
    digitalWrite(motorB_DIR, HIGH);
  } else if (value2 == HIGH && value3 == HIGH && value4 == LOW) {
    // Se o sensor do meio detectar a linha, vire levemente à direita
    analogWrite(motorA_EN, left_speed);
    digitalWrite(motorA_DIR, HIGH);
    analogWrite(motorB_EN, right_speed);
    digitalWrite(motorB_DIR, LOW);
  } else if (value2 == LOW && value3 == HIGH && value4 == HIGH) {
    // Se o sensor do meio detectar a linha, vire levemente à esquerda
    analogWrite(motorA_EN, left_speed);
    digitalWrite(motorA_DIR, LOW);
    analogWrite(motorB_EN, right_speed);
    digitalWrite(motorB_DIR, HIGH);
  } else {
    // Se nenhum sensor detectar a linha, pare
    digitalWrite(motorA_EN, LOW);
    digitalWrite(motorA_DIR, LOW);
    digitalWrite(motorB_EN, LOW);
    digitalWrite(motorB_DIR, LOW);
  }

  // Atualizando a variável de erro anterior
  previous_error = error;

  delay(10); // Pequeno atraso para estabilidade
}
