#include<Arduino.h>

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
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
}

void loop() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {  // Read analog value every 40ms
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  // Read the analog value
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 2000U) {  // Print values every 2 seconds
    printTimepoint = millis();

    // Copy the buffer for processing
    for (int copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }

    // Calculate the median voltage
    int medianAnalogValue = getMedianNum(analogBufferTemp, SCOUNT);
    averageVoltage = medianAnalogValue * (float)VREF / 4096.0;

    // Debugging: Print raw ADC value and voltage
    Serial.print("Raw ADC Value: ");
    Serial.println(medianAnalogValue);
    Serial.print("Average Voltage: ");
    Serial.print(averageVoltage, 2);
    Serial.println(" V");

    // Temperature compensation
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;

    // Convert voltage to TDS value
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage -
                255.86 * compensationVoltage * compensationVoltage +
                857.39 * compensationVoltage) * 0.5;

    // Debugging: Print compensation values and TDS value
    Serial.print("Compensation Coefficient: ");
    Serial.println(compensationCoefficient, 2);
    Serial.print("Compensation Voltage: ");
    Serial.print(compensationVoltage, 2);
    Serial.println(" V");
    Serial.print("TDS Value: ");
    Serial.print(tdsValue, 0);
    Serial.println(" ppm\n");
  }
}