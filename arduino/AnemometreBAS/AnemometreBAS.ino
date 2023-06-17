#include <Adafruit_SleepyDog.h>     //Needed to initiate reboots by the SAMD WatchDog 
#include "OneWire.h"
#include "DallasTemperature.h"
#include <MKRWAN_v2.h>
#include "ArduinoLowPower.h"
LoRaModem modem;
OneWire oneWire(A1);
DallasTemperature ds(&oneWire);
String appEui;
String appKey;
String devAddr;
String nwkSKey;
String appSKey;
char chargehex[16];  // c'est la charge utile
int err;
int connected;
float value;
static uint8_t payload_lora[16];


//Variables used for anemometer device
volatile unsigned long nDetections = 0;

unsigned long nDetections_copy= 0;
unsigned long dt = 0;
float period;
float freq =0;
float windSpeed = 0.0;

const long interval = 20000;
unsigned long previousMillis = 0; 
//End variables for anemometer

#define R3 1500.0
#define R4 2500.0

//*******************************************************************************************
void setup() {

  //Info about low consomption :
  //https://forum.arduino.cc/t/deepsleep-mode-13ua-without-lora-begin-290ua-with-lora-begin/693605
  //https://forum.arduino.cc/t/what-happens-when-the-solder-jumper-of-mkrwan-1310-is-cut/668734

  //WachDog Timer
  //https://create.arduino.cc/projecthub/andreas_waldherr/mkr-wan-1310-iot-operating-at-0-92ma-879793

  // Set digital pins to input to reduce battery consumption
  pinMode(0,INPUT);
  digitalWrite(0, HIGH); 

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  // pour commencer il faut lire dans le circuit son EUI, une sorte d'adresse MAC
  // installer le programme FirstConfiguration qui est dans les exemples de la librairie MKRWAN attention pas MKRWAN_v2!
  // recupérer le code app EUI
  // ensuite ils demandent de mettre à jour le firmware avec le prg
  // 'MKRWANFWUpdate_standalone.ino'
  //

  appEui = "TODO: ENTER APPEUI HERE";
  appKey = "TODO: ENTER APPKEY HERE";
  Serial.begin(115200);
  appKey.trim();
  appEui.trim();
  delay(5000); 
  Serial.println("programme Anemometre BAS 04/2023 to MKRWAN1310");

  ds.requestTemperatures();
  value = ds.getTempCByIndex(0);  // temperature
  Serial.print(" temp :");
  Serial.println(value);

  ds.begin();  // sonde temperature activée

  //Code for anemometer
  attachInterrupt(0, anemo, RISING);
  previousMillis = millis();
}
//*****************************************************************
void loop() {
  uint32_t sleepTime;
  int i;
  //USBDevice.detach();

    // read the input on analog pin 0:
    int sensorValue = analogRead(A4);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.2V):
    float voltage = sensorValue * (3.2 / 1023.0);
  
    float batVoltage = (R3+R4)/R4 * voltage;
  Serial.print(" bat. voltage :");
  Serial.println(batVoltage);

  windSpeed = computeWindSpeed();
  Serial.print(" speed :");
  Serial.println(windSpeed);

  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(0, LOW);
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  connected = 0;
  while (!connected) {
    connected = modem.joinOTAA(appEui, appKey);
    if (!connected) {
      Serial.print("Ca merde :(  ");
      Serial.print("Your module version is: ");
      Serial.println(modem.version());
      Serial.print("Your device EUI is: ");
      Serial.println(modem.deviceEUI());
      Serial.println("connecting via OTAA");
      Serial.print(" appEui :");
      Serial.println(appEui);
      Serial.print(" appKey :");
      Serial.println(appKey);
      //resetFunc(); //call reset
      delay(500);
    }
  }

  float windDir = readWindDirection();

  // emission charge
  modem.setPort(3);
  modem.beginPacket();
  int32_t voltInt = windSpeed * 10000;
  payload_lora[0] = (voltInt & 0xff000000) >> 24;
  payload_lora[1] = (voltInt & 0x00ff0000) >> 16;
  payload_lora[2] = (voltInt & 0x0000ff00) >> 8;
  payload_lora[3] = (voltInt & 0x000000ff);
  int32_t volt1Int = batVoltage * 10000;
  payload_lora[4] = (volt1Int & 0xff000000) >> 24;
  payload_lora[5] = (volt1Int & 0x00ff0000) >> 16;
  payload_lora[6] = (volt1Int & 0x0000ff00) >> 8;
  payload_lora[7] = (volt1Int & 0x000000ff);
  int32_t vitesseInt = value * 10000;
  payload_lora[8] = (vitesseInt & 0xff000000) >> 24;
  payload_lora[9] = (vitesseInt & 0x00ff0000) >> 16;
  payload_lora[10] = (vitesseInt & 0x0000ff00) >> 8;
  payload_lora[11] = (vitesseInt & 0x000000ff);
  int32_t dirInt = windDir * 10000;
  payload_lora[12] = (dirInt & 0xff000000) >> 24;
  payload_lora[13] = (dirInt & 0x00ff0000) >> 16;
  payload_lora[14] = (dirInt & 0x0000ff00) >> 8;
  payload_lora[15] = (dirInt & 0x000000ff);  

  modem.write(payload_lora, sizeof(payload_lora));
  Serial.print(" charge : ");
  for (i = 0; i < 16; i++) {
    Serial.print(payload_lora[i]);
    Serial.print("-");
  }
  Serial.println(" ");
  delay(500);
  // fin emission
  err = modem.endPacket(true);
  if (err > 0) {  // valeur 1 retournée si correct
    Serial.println("Message sent correctly!");
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
  } else {                           // ca renvoie toujours 0
    Serial.println("Error sending message :(");
    delay(100);
  }

  // Go to STANDBY Mode for 37 times 
  for (int i = 0; i < 37; i++) {
    Watchdog.sleep();                           // Enter the Standby Mode for 16,5 seconds (MKR WAN 1310 specific)
    Serial.println(i);
  }

  // Now restart the MKR
  int countdownMS = Watchdog.enable(100);      // WatchDog timeout in 0.1 seconds
  delay(countdownMS+1000);                     // Wait for 1.1 second!!!
  Watchdog.disable();                          // The WatchDog WILL time out and therefore restart the CPU

}

float readWindDirection() {
      // read the input on analog pin 0:
    int sensorValue = analogRead(A6);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.2V):
    float voltage = sensorValue * (3.2 / 1023.0);
    float tolerance = 0.1;

    if (abs(voltage-0.9) < tolerance) {
      return 0.0;
    }
    else if (abs(voltage-0.57) < tolerance) {
      return 45.0;
    }
    else if (abs(voltage-0.28) < tolerance) {
      return 90.0;
    }
    else if (abs(voltage-1.44) < tolerance) {
      return 135.0;
    }
    else if (abs(voltage-2.45) < tolerance) {
      return 180.0;
    }
    else if (abs(voltage-2.77) < tolerance) {
      return 225.0;
    }
    else if (abs(voltage-2.95) < tolerance) {
      return 270.0;
    }
    else if (abs(voltage-1.87) < tolerance) {
      return 315.0;
    }
    else {
      return 0.0;
    }
}

float computeWindSpeed() {
  unsigned long currentMillis = millis();
  dt = currentMillis - previousMillis;
  while (dt<interval) {
    currentMillis = millis();
    dt = currentMillis - previousMillis;
  }

  noInterrupts();
  nDetections_copy = nDetections;
  nDetections = 0;
  interrupts();

  previousMillis = millis();

  if (nDetections_copy>0) {
    period = float(dt)/float(nDetections_copy);
    freq = 1000.0/period/3.0;
    windSpeed = 2.4 * freq * 0.277778;
  }
  else {
    windSpeed = 0.0;
  }

  return windSpeed;
}

void anemo(void)
{
 nDetections++;
}
