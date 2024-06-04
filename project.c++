#include <OneWire.h>
#include <DallasTemperature.h>

// Pino onde o sensor DS18B20 está conectado
const int tempSensorPin = 2; // Pino digital 2 (OneWire)
const int led_pins[3] = {8, 9, 10}; // Pinos dos LEDs
const int buzzer = 7; // Pino do buzzer

bool extreme = false; // Indica se as condições são extremas (ambos temperatura e pH fora do intervalo)
bool bad = false; // Indica se pelo menos uma das condições está fora do intervalo

// Inicializa uma instância do sensor DS18B20
OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

// Arrays para armazenar as últimas 10 leituras de temperatura e pH
float temperatureReadings[10];
float pHReadings[10];

int currentIndex = 0; // Índice atual para armazenar a leitura
int numReadings = 0; // Contador de leituras para verificar se já temos 10 leituras

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial
  sensors.begin(); // Inicializa o sensor DS18B20
  pinMode(buzzer, OUTPUT); // Configura o pino do buzzer como saída
  for (int i = 0; i < 3; i++) {
    pinMode(led_pins[i], OUTPUT); // Configura os pinos dos LEDs como saída
    digitalWrite(led_pins[i], LOW); // Inicialmente desliga todos os LEDs
  }
}

void loop() {
  Serial.println("Leitura: " + String(numReadings + 1)); // Imprime o número da leitura atual

  int sensorValue_pH = analogRead(A0); // Lê o valor analógico do sensor de pH
  float pH = map(sensorValue_pH, 0, 1023, 0, 14); // Mapeia a leitura do potenciômetro de pH para o intervalo de 0 a 14

  // Leitura da temperatura do sensor DS18B20
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0); // Obtém a temperatura em graus Celsius

  // Armazena as leituras nos arrays
  temperatureReadings[currentIndex] = temperature;
  pHReadings[currentIndex] = pH;
  
  // Incrementa o índice e o reseta se necessário
  currentIndex = (currentIndex + 1) % 10;
  numReadings = min(numReadings + 1, 10); // Incrementa numReadings até o máximo de 10

  // Apenas verifica as médias se já tivermos 10 leituras
  if (numReadings == 10) {
    float sumTemperature = 0;
    float sumPH = 0;

    // Calcula a soma das leituras
    for (int i = 0; i < 10; i++) {
      sumTemperature += temperatureReadings[i];
      sumPH += pHReadings[i];
    }

    // Calcula as médias
    float avgTemperature = sumTemperature / 10;
    float avgPH = sumPH / 10;

    // Verifica as condições e acende o LED apropriado
    bool temperatureOutOfRange = (avgTemperature < 18.0 || avgTemperature > 28.0);
    bool pHOutOfRange = (avgPH < 7.4 || avgPH > 8.5);

    // Desliga todos os LEDs antes de ligar o correto
    for (int i = 0; i < 3; i++) {
      digitalWrite(led_pins[i], LOW);
    }

    if (temperatureOutOfRange && pHOutOfRange) {
      // Condição extrema: ambos os parâmetros fora dos limites
      bad = false;
      extreme = true;
      digitalWrite(led_pins[2], HIGH);
    } else if (temperatureOutOfRange || pHOutOfRange) {
      // Condição ruim: pelo menos um dos parâmetros fora dos limites
      bad = true;
      extreme = false;
      digitalWrite(led_pins[1], HIGH);
    } else {
      // Condição normal: ambos os parâmetros dentro dos limites
      bad = false;
      extreme = false;
      digitalWrite(led_pins[0], HIGH);
    }

    // Criação do JSON para os arrays de leituras
    String json = "{\"temperaturas\":[";
    for (int i = 0; i < 10; i++) {
      json += String(temperatureReadings[i]);
      if (i < 9) {
        json += ",";
      }
    }

    json += "],\"pH\":[";
    for (int i = 0; i < 10; i++) {
      json += String(pHReadings[i]);
      if (i < 9) {
        json += ",";
      }
    }
    json += "]}";
    
    // Envia o JSON via Serial
    Serial.println(json);
    numReadings = 0; // Reseta o contador de leituras
  }

  // Controle do buzzer baseado nas condições
  if (extreme) {
    tone(buzzer, 250, 200); // Som contínuo para condições extremas
  } else if (bad) {
    tone(buzzer, 500, 200); // Som intermitente para condições ruins
  } else {
    noTone(buzzer); // Sem som para condições normais
  }

  delay(1000); // Aguarda 1 segundo antes de realizar a próxima leitura
}
