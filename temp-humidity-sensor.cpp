#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 4
#define DHTTYPE DHT11
#define LED 2

DHT dht(DHTPIN, DHTTYPE);

void setup() {
    Serial.begin(9600);
    pinMode(LED, OUTPUT);
    pinMode(DHTPIN, INPUT_PULLUP);

    Serial.println("DHT11 Sensor Testing!");

    for(int i=0; i<3; i++) {
        delay(400);
        digitalWrite(LED, HIGH);
        delay(400);
        digitalWrite(LED, LOW);
    }

    delay(500);

    dht.begin();
}


void loop() {
    // Wait between readings
    delay(2000);

    // Read humidity and temperature 
    float humidity = dht.readHumidity();
    float tempC = dht.readTemperature();        // Temp in Celsius
    float tempF = dht.readTemperature(true);    // Temp in Fahrenheit

    // Check if the readings are valid
    if (isnan(humidity) || isnan(tempC) || isnan(tempF)) {
        Serial.println("Failed to read from DHT Sensor! Failed Values: ");

        Serial.print("Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");

        Serial.print("Temperature: ");
        Serial.print(tempC);
        Serial.print(" °C  &  ");
        Serial.print(tempF);
        Serial.println(" °F");

        return;     // Exit the loop function here
    }

    // Print readings to Serial Monitor
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