### Projeto de Monitoramento de Temperatura e pH com Arduino

Este projeto monitora a temperatura e o pH de um ambiente utilizando um sensor DS18B20 e um sensor de pH. Os dados são processados e armazenados em arrays, sendo transmitidos em formato JSON via comunicação serial para um aplicativo Python. LEDs e um buzzer são usados para indicar o estado das leituras.

## Simulação
<a href="https://wokwi.com/projects/399438925654312961" target="_blank"><h2>Wokwi</h2></a>

## Componentes Usados

- Arduino Uno (ou equivalente)
- Sensor de Temperatura DS18B20
- Sensor de pH
- Resistores de 4.7kΩ (pull-up para DS18B20)
- LEDs (3 unidades, cores diferentes preferencialmente)
- Buzzer
- Protoboard e Jumpers
- Fonte de Alimentação ou Cabo USB

## Método de Montagem

1. **Conecte o Sensor DS18B20**:
   - Pino GND do DS18B20 ao GND do Arduino.
   - Pino VDD do DS18B20 ao 5V do Arduino.
   - Pino DATA do DS18B20 ao pino digital 2 do Arduino.
   - Conecte um resistor de 4.7kΩ entre os pinos VDD e DATA do DS18B20 (pull-up resistor).

2. **Conecte o Sensor de pH**:
   - Pino VCC do sensor de pH ao 5V do Arduino.
   - Pino GND do sensor de pH ao GND do Arduino.
   - Pino de saída analógica do sensor de pH ao pino A0 do Arduino.

3. **Conecte os LEDs**:
   - Conecte cada LED aos pinos digitais 8, 9, e 10 do Arduino.
   - O catodo de cada LED deve ser conectado ao GND do Arduino.

4. **Conecte o Buzzer**:
   - Pino positivo do buzzer ao pino digital 7 do Arduino.
   - Pino negativo do buzzer ao GND do Arduino.

## Explicação do Código

### Bibliotecas e Definições de Pinos

```
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
```

### Configuração Inicial

No setup, inicializamos a comunicação serial, configuramos os pinos dos LEDs e do buzzer, e iniciamos o sensor DS18B20.

```
void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial
  sensors.begin(); // Inicializa o sensor DS18B20
  pinMode(buzzer, OUTPUT); // Configura o pino do buzzer como saída
  for (int i = 0; i < 3; i++) {
    pinMode(led_pins[i], OUTPUT); // Configura os pinos dos LEDs como saída
    digitalWrite(led_pins[i], LOW); // Inicialmente desliga todos os LEDs
  }
}
```

### Loop Principal

No loop principal, lemos os sensores, armazenamos os valores em arrays, calculamos as médias e verificamos as condições para acionar os LEDs e o buzzer.

```
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
```

### Explicação dos Comandos

- **Bibliotecas e Definições de Pinos**:
  - `#include <OneWire.h>` e `#include <DallasTemperature.h>`: Inclui as bibliotecas necessárias para o sensor de temperatura DS18B20.
  - `const int led_pins[3] = {8, 9, 10};`: Define os pinos dos LEDs.
  - `const int buzzer = 7;`: Define o pino do buzzer.

- **Configuração Inicial**:
  - `Serial.begin(9600);`: Inicializa a comunicação serial.
  - `sensors.begin();`: Inicializa o sensor de temperatura.
  - `pinMode(buzzer, OUTPUT);`: Configura o pino do buzzer como saída.
  - Configuração dos pinos dos LEDs como saídas e os desliga inicialmente.

- **Loop Principal**:
  - Leitura e mapeamento do valor do sensor de pH.
  - Leitura da temperatura do sensor DS18B20.
  - Armazenamento das leituras nos arrays e atualização do índice.
  - Cálculo das médias das leituras após 10 leituras.
  - Verificação das condições e acionamento dos LEDs e buzzer conforme apropriado.
  - Criação e envio do JSON via Serial.


## Colaboradores do Projeto
<div>

<a href="https://github.com/dav0fc" target="_blank" style="text-align: center; margin-right: 10px;">
<img loading="lazy" src="https://avatars.githubusercontent.com/dav0fc" width=120>
<p style="font-size:min(2vh, 36px); margin-top: 10px;">	David Gabriel Gomes Fernandes - RM 556020</p>
</a>

<a href="https://github.com/desenise" target="_blank" style="text-align: center; margin-right: 10px;">
<img loading="lazy" src="https://avatars.githubusercontent.com/desenise" width=120>
<p style="font-size:min(2vh, 36px); margin-top: 10px;">Denise Senise - RM 556006</p>
</a>

<a href="https://github.com/MateusLem" target="_blank" style="text-align: center; margin-right: 10px;">
<img loading="lazy" src="https://avatars.githubusercontent.com/MateusLem" width=120>
<p style="font-size:min(2vh, 36px); margin-top: 10px;">Mateus da Costa Leme - RM 557803</p>
</a>


</div>