//reads battery voltage and compensates the speed of motor respectively
//warns about the depleting battery voltage
//volatage divider and fed to analog pin of stm32 for measurement

#define senseBattery PA1
#define buzzer PA0

#define BUZZER_ON   (GPIOA_BASE->BSRR |= (1 << 0))
#define BUZZER_OFF  (GPIOA_BASE->BSRR |= (1 << 16))

//pins lai io configure gareko xaina hai ..garna birselas ni


float volts = 0;

void setup() {
  Serial.begin(115200);
  pinMode(senseBattery, INPUT);
  pinMode(buzzer,OUTPUT);
}

void loop() {
  volts = analogRead(senseBattery) / 112.81;
  Serial.println(volts, 1);
 
 if (volts < 10.0f) {
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
  }
}
