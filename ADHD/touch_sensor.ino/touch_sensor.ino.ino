int touchPin = 2; 
int LEDpin = 3; 
bool val; 
bool lightON; 
bool touch; 


void setup() { 
 Serial.begin(9600); 
 pinMode(touchPin, INPUT);  pinMode(LEDpin, OUTPUT); 
} 
void loop() { 
 val = digitalRead(touchPin);  if (val == 1 && lightON == 0) {  touch = 1 - touch; 
 } 
 lightON = val; 
if (touch == 1) { 
 Serial.println("LED ON");
 digitalWrite(LEDpin, HIGH); 
 } else { 
 Serial.println("LED OFF");  digitalWrite(LEDpin, LOW); 
}
}

