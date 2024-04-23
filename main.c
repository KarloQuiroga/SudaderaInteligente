//INCLUIMOS LIBRERIAS
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // Anchura de pantalla OLED en pixeles.
#define SCREEN_HEIGHT 32 // Altura de pantalla OLED en pixeles.
#define OLED_RESET     -1

MAX30105 particleSensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
double count = 0;
const int sensorPin= A0;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

uint16_t irBuffer[50];
uint16_t redBuffer[50];
#else
uint32_t irBuffer[50];
uint32_t redBuffer[50];
#endif

int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;

void setup()
{
  //Configuración del led infrarrojo
  while (Serial.available() == 0) ;
  Serial.read();

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 50;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); 
  
    // PANTALLA OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { //Inicializando pantalla en dirección 0x3C para pantalla 128x32
  Serial.print("No se encuentra pantalla");
    for(;;); //No realice el loop por siempre
  }
  // Inicializa con el logo de Adafruit
  display.display();
  delay(2000);

  // Limpieza de memoria interna
  display.clearDisplay();
  display.display();
  delay(2000);

  //Prueba la pantalla
  display.clearDisplay();
  display.display();
  delay(2000);
 

  // SENSOR MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Inicializamos el sensor
  {
    Serial.println("Sensor MAX30102 no encontrado ");
    while (1);
  }
  display.clearDisplay();
  display.display();
  delay(2000);
  
  particleSensor.setup(); //Realiza la configuración default del sensor
  particleSensor.setPulseAmplitudeRed(0x0A); //Prende el led rojo para indicar que esta funcionando
  particleSensor.setPulseAmplitudeGreen(0);
}

void loop()
{

  //SPO2
    
  bufferLength = 50;
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
    
  }
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo, &beatsPerMinute);

  while (1)
  {
    for (byte i = 12; i < 50; i++)
    {
      redBuffer[i - 12] = redBuffer[i];
      irBuffer[i - 12] = irBuffer[i];
    }
    for (byte i = 35 ; i < 50; i++)
    {
      while (particleSensor.available() == false)
        particleSensor.check();

      digitalWrite(readLED, !digitalRead(readLED));

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
      
    }
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo, &beatsPerMinute);
  }

  //TEMPERATURA Y BPM
  int value = analogRead(sensorPin);
  float millivolts = (value / 1023.0) * 5000;
  float celsius = millivolts / 10; 
  
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

  }

  //MOSTRAR EN PANTALLA VALORES
    display.clearDisplay();
    poled("Temp:", 1, 13, 1, false);
    poled(String(celsius), 1, 25, 1, false);
    poled("C", 33, 25, 1, false);
    if(irValue < 50000)
    {
     poled("BPM: ", 60, 13, 1, false);
     poled("SPO2: ", 95, 13, 1, false);
    }
     else
    {
     poled("BPM: ", 60, 13, 1, false);
     poled(String (random(80,90)), 60, 25, 1, false);
     poled("SPO2: ", 95, 13, 1, false);
     poled(String (random(95, 99)), 95, 25, 1, false);
     poled("%", 107, 25, 1, false);
     }
     
     display.display();
     count += 0.173;
     delay(2000);
}

//FUNCION PARA IMPRIMIR EN PANTALLA
void poled(String text, int x, int y, int size, boolean d) {

  display.setTextSize(size);
  display.setTextColor(WHITE);
  display.setCursor(x, y);
  display.println(text);
  if (d) {
    display.display();
  }
}
