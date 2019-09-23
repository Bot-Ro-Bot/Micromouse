//reads battery voltage and compensates the speed of motor respectively
//warns about the depleting battery voltage
//volatage divider and fed to analog pin of stm32 for measurement

#define senseBattery PB1
#define buzzer PB2

uint8_t volts=0;

void setup() {
pinMode(senseBattery,INPUT);
pinMode(buzzer,OUTPUT);
}

void loop() {
 volts= analogRead(senseBattery);
 Serial.println(volts);
 if(volts<7){
  digitalWrite(buzzer,HIGH);
  delay(100);
  digitalWrite(buzzer,LOW);
 }
}
