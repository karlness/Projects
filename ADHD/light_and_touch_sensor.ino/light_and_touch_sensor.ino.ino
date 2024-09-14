#include <Wire.h>
#include <BH1750.h>
BH1750 lightMeter;

int touchPin = 2; 
int LEDpin = 3; 
bool val; 
bool lightON; 
bool touch; 
void setup(){ 
 Serial.begin(9600); //Touch Sensor
 pinMode(touchPin, INPUT);
 pinMode(LEDpin, OUTPUT);
 
 Serial.begin(9600); //Light Sensor
  Wire.begin();
  lightMeter.begin();
  Serial.println(F("BH1750 Test"));
} 
void loop(){ //Touch Sensor
 val = digitalRead(touchPin);
 if (val == 1 && lightON == 0)
 {touch = 1 - touch;} 
 lightON = val;
 if (touch == 1)
 { 
 Serial.println("TOUCH ON"); 
 }
 else
 { 
 Serial.println("TOUCH OFF"); 
 }
 
 float lux = lightMeter.readLightLevel(); //Light Sensor
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lux");
  delay(1000);

//LED ON and OFF
if (lux > 200 && touch == 1)
digitalWrite(LEDpin, HIGH);
else
digitalWrite(LEDpin, LOW);  
}
