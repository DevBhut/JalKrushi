#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>

// DHT11 Sensor Configuration
#define DHTPIN 4
#define DHTTYPE DHT11
#define LED 2
DHT dht(DHTPIN, DHTTYPE);

// pH Sensor Configuration
const int potPin = A0;
float ph;
float phValue = 0;

// TDS Sensor Configuration
#define TdsSensorPin 27
#define VREF 3.3              // Analog reference voltage (Volt) of the ADC
#define SCOUNT 30             // Number of samples for averaging

int analogBuffer[SCOUNT];     // Store the analog values read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;       // Current temperature for compensation

// Median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];

  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }

  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }

  return bTemp;
}

void setup() {
  // General setup
  Serial.begin(115200);

  // DHT11 setup
  pinMode(LED, OUTPUT);
  pinMode(DHTPIN, INPUT_PULLUP);
  dht.begin();

  // pH Sensor setup
  pinMode(potPin, INPUT);

  // TDS Sensor setup
  pinMode(TdsSensorPin, INPUT);

  // Indication LED for initialization
  for (int i = 0; i < 3; i++) {
    delay(400);
    digitalWrite(LED, HIGH);
    delay(400);
    digitalWrite(LED, LOW);
  }
}

void loop() {
  // DHT11 Sensor Readings
  float humidity = dht.readHumidity();
  float tempC = dht.readTemperature();        // Temp in Celsius
  float tempF = dht.readTemperature(true);    // Temp in Fahrenheit

  if (isnan(humidity) || isnan(tempC) || isnan(tempF)) {
    Serial.println("Failed to read from DHT Sensor!");
  } else {
    Serial.println("----------DHT11 Readings----------");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print(" °C  &  ");
    Serial.print(tempF);
    Serial.println(" °F");
    Serial.println("----------------------------------");
  }

  // pH Sensor Readings
  phValue = analogRead(potPin);
  float phVoltage = phValue * (3.3 / 4095.0);
  ph = (3.3 * phVoltage);
  Serial.println("----------pH Sensor Readings----------");
  Serial.print("Raw Value: ");
  Serial.print(phValue);
  Serial.print(" | Voltage: ");
  Serial.print(phVoltage);
  Serial.print(" V | pH: ");
  Serial.println(ph);

  // TDS Sensor Readings
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 2000U) {
    printTimepoint = millis();
    for (int copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }

    int medianAnalogValue = getMedianNum(analogBufferTemp, SCOUNT);
    averageVoltage = medianAnalogValue * (float)VREF / 4096.0;

    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;

    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage -
                255.86 * compensationVoltage * compensationVoltage +
                857.39 * compensationVoltage) * 0.5;

    Serial.println("----------TDS Sensor Readings----------");
    Serial.print("Raw ADC Value: ");
    Serial.println(medianAnalogValue);
    Serial.print("Average Voltage: ");
    Serial.print(averageVoltage, 2);
    Serial.println(" V");
    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.println(" ppm");
    Serial.println("--------------------------------------");
  }

  delay(2000);  // Wait before the next cycle
}
