#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

const int tempSensorPin = 4; // Pino digital do sensor de temperatura (OneWire)
const int ledPins[3] = {8, 9, 10}; // Pinos dos LEDs
const int buzzerPin = 7; // Pino do buzzer

// Inicializa uma instância do sensor DS18B20
OneWire oneWire(tempSensorPin);
DallasTemperature sensors(&oneWire);

LiquidCrystal_I2C lcd(0x27, 20, 4); // Inicializa os LCDs I2C

// Arrays para armazenar as últimas 10 leituras de temperatura e pH
float temperatureReadings[10];
float pHReadings[10];

float avgReadings[2] = {0, -1}; // Array para armazenar as médias de temperatura e pH

String statusLCD[2]; // Array para armazenar as mensagens exibidas no LCD

int condition = -1; // Representa as condições do ambiente. -1 = Normal/Default, 0 = Ruim (Uma das leituras está fora da faixa ideal), 1 = Extremo (Ambas as leituras estão fora da faixa ideal)
int currentIndex = 0; // Índice atual para armazenar a leitura
int numReadings = 0; // Contador de leituras para verificar se já temos 10 leituras

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial
  sensors.begin(); // Inicializa o sensor DS18B20

  //Inicializando o LCD 
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(buzzerPin, OUTPUT); // Configura o pino do buzzer como saída
  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT); // Configura os pinos dos LEDs como saída
    digitalWrite(ledPins[i], LOW); // Inicialmente desliga todos os LEDs
  }
}

void loop() {
  readSensors(); // Lê os valores dos sensores e armazena nas respectivas listas

  currentIndex = (currentIndex + 1) % 10; // Incrementa o índice e o reseta se necessário
  numReadings = min(numReadings + 1, 10); // Incrementa numReadings até o máximo de 10

  // Apenas verifica as médias se já tivermos 10 leituras
  if (numReadings == 10) {
    updateAvg(); // Atualiza os dados das leituras

    updateStatus(); // Atualiza as variáveis de status do Buzzer, LCD e dos LEDs

    String json = generateJSON(); // Gera o JSON que será exibido no Serial
    Serial.println(json); // Envia o JSON via Serial
    numReadings = 0; // Reseta o contador de leituras
  }

  soundBuzzer(); // Controle do buzzer baseado nas condições
  updateLCD(); // Atualiza os status no LCD
  delay(1000); // Aguarda 1 segundo antes de realizar a próxima leitura
}

void readSensors(){
  int sensorValuePH = analogRead(A0); // Lê o valor analógico do sensor de pH
  float pH = map(sensorValuePH, 0, 1023, 0, 14); // Mapeia a leitura do potenciômetro de pH para o intervalo de 0 a 14

  // Leitura da temperatura do sensor DS18B20
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0); // Obtém a temperatura em graus Celsius

  // Armazena as leituras nos arrays
  temperatureReadings[currentIndex] = temperature;
  pHReadings[currentIndex] = pH;  
}

void updateAvg() {
  float sumTemperature = 0;
  float sumPH = 0;

  // Calcula a soma das leituras
  for (int i = 0; i < 10; i++) {
    sumTemperature += temperatureReadings[i];
    sumPH += pHReadings[i];
  }

  // Atualiza a média das leituras
  avgReadings[0] = sumTemperature / 10;
  avgReadings[1] = sumPH / 10;
}

void updateStatus() {
  // Armazena os resultados da verificação em variáveis locais
  bool temperatureOutOfRange = (avgReadings[0] < 18.0 || avgReadings[0] > 28.0);
  bool pHOutOfRange = (avgReadings[1] < 7.4 || avgReadings[1] > 8.5);

  // Desliga todos os LEDs antes de ligar o correto
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPins[i], LOW);
  }

  if (temperatureOutOfRange && pHOutOfRange) {
    // Condição extrema: ambos os parâmetros fora dos limites
    condition = 1;
    digitalWrite(ledPins[2], HIGH); // Liga o LED vermelho
    // Define as mensagem de temperatura e pH do LCD, respectivamente
    statusLCD[0] = "Critico";
    statusLCD[1] = "Critico";

  } else if (temperatureOutOfRange || pHOutOfRange) {
    // Condição ruim: pelo menos um dos parâmetros fora dos limites
    condition = 0;
    digitalWrite(ledPins[1], HIGH); // Liga o LED amarelo
    // Define as mensagem de temperatura e pH do LCD, respectivamente
    if (temperatureOutOfRange) {
      statusLCD[0] = "Critico";
      statusLCD[1] = "OK";
    } else {
      statusLCD[0] = "OK"; 
      statusLCD[1] = "Critico";
    }

  } else {
    // Condição normal: ambos os parâmetros dentro dos limites
    condition = -1;
    digitalWrite(ledPins[0], HIGH); // Liga o LED verde
    // Define as mensagem de temperatura e pH do LCD, respectivamente
    statusLCD[0] = "OK"; 
    statusLCD[1] = "OK";
  }
}

void soundBuzzer(){
  switch (condition) {
    case 1:
      tone(buzzerPin, 250, 100); // Som para condições extremas
      break;
    case 0:
      tone(buzzerPin, 500, 100); // Som para condições ruins
      break;
    default:
      noTone(buzzerPin); // Sem som para condições normais/não verificadas
      delay(100);
      break;
  }
}

void updateLCD(){
  lcd.clear(); // Limpa o LCD
  showLCD(0, "Leitura: "+String(numReadings + 1)); // Imprime o número da leitura atual;

  if (avgReadings[1] < 0) {
    showLCD(1, "Temp: Verificando");
    showLCD(2, "pH: Verificando");

  } else {
    // Criação de char para o simbolo °
    byte degree[8] = {B00000, B01100, B10010, B10010, B01100, B00000, B00000, B00000};
    lcd.createChar(0, degree);
    
    lcd.setCursor(0, 1);
    lcd.print("Temp: " + String(avgReadings[0], 2)+" ");
    lcd.write(byte(0));
    lcd.print("C");

    if (statusLCD[0].equals("Critico")){
      showLCD(2, "-> "+statusLCD[0]);
      showLCD(3, "pH: " + String(avgReadings[1], 2)+" - "+statusLCD[1]);
    } else {
      lcd.print(" - "+statusLCD[0]);
      showLCD(2, "pH: " + String(avgReadings[1], 2)+" - "+statusLCD[1]);
    }

    
  }
}

void showLCD(int row, String text) {
  lcd.setCursor(0, row); // Define a linha do LCD que será utilizada
  lcd.print(text); // O texto que será exibido no LCD
}

String generateJSON() {
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
  json += "],\"mediaTemp\":" + String(avgReadings[0],2) + ",\"mediapH\":" + String(avgReadings[1],2) + "}";
  return json;
}
