#define lMotorEnable PA7
#define lMotorForward PB0
#define lMotorBack PB1

#define rMotorEnable PA6
#define rMotorForward PA5
#define rMotorBack PA4

//#define pGain 5
//#define iGain 1.2
//#define dGain 1.3

#define phaseALeft PB8
#define phaseARight PB9

volatile unsigned int pulseCountLeft = 0; //4 byte ko hunxa STM32 ko integer type
volatile unsigned int pulseCountRight = 0;
volatile uint16_t botTraverse = 0;


//duita options ...simple motor offset bata garna ni sakinxa ...ani PID ni garna sakinxa....


class Motor {
  protected:
    byte enablePin, forwardPin, backPin;
  public:
    Motor(byte enablePin, byte forwardPin, byte backPin);
    static uint8_t speed;
    void forward(uint8_t speed);
    void back(uint8_t speed);
    void brake();
    void stop();
};

Motor::Motor(byte enablePin, byte forwardPin, byte backPin) {
  this->enablePin = enablePin;
  this->forwardPin = forwardPin;
  this->backPin = backPin;

  pinMode(enablePin, OUTPUT);
  pinMode(forwardPin, OUTPUT);
  pinMode(backPin, OUTPUT);
}

void Motor::forward(uint8_t speed) {
  analogWrite(enablePin, speed);
  digitalWrite(forwardPin, HIGH);
  digitalWrite(backPin, LOW);
}

void Motor::back(uint8_t speed) {
  analogWrite(enablePin, speed);
  digitalWrite(forwardPin, LOW);
  digitalWrite(backPin, HIGH);
}

void Motor::stop() {
  analogWrite(enablePin, 0);
  digitalWrite(forwardPin, LOW);
  digitalWrite(backPin, LOW);
}

void Motor::brake() {
  analogWrite(enablePin, 255);
  digitalWrite(forwardPin, LOW);
  digitalWrite(forwardPin, LOW);
}

//encoder ko cpr anusar ani gear ratio anusar nikalne
/*  11 cpr bhako magako motor aayo bhane...
     euta revolution maa 11 cpr hunxa if only one edge is taken(RISING/FALLING)
     euta revolution maa 22 cpr hunxa if both the edges is taken(CHANGE)
     magako motor ko gear ratio 1:50 xa..bhanne ley 50 rotations of shaft of motor lida 1 rotation of gearbox ko shaft hunxa(1 rotation of wheel hunxa)
     50*22=1100 rotations chahinxa single wheel ko rotation ko laagi
     if 37mm ko wheel use garyo bhane 2*pi*r= 2*3.141*37/2= 116.217 mm linearly displace hunxa robot
*/
void countPulseRight() {
  pulseCountRight++;
  //  if(pulseCountRight>=1100){
  //    pulseCountRight=0;
  //    botTraverse++;
  //  }
}

void countPulseLeft() {
  pulseCountLeft++;
  //  if(pulseCountLeft>=1100){
  //    pulseCountLeft=0;
  //    botTraverse++;
  //  }
}


Motor *leftMotor, *rightMotor;

void setup() {
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.
  //The following function connects PB3 and PB4 to the
  //alternate output function.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                     //Connects PB3 and PB4 to output function.

  Serial.begin(115200);
   pinMode(phaseALeft, INPUT_PULLDOWN);
  pinMode(phaseARight, INPUT_PULLDOWN);
  attachInterrupt(phaseALeft, countPulseLeft, CHANGE);
  attachInterrupt(phaseARight, countPulseRight, CHANGE);
  leftMotor  = new Motor(lMotorEnable, lMotorForward, lMotorBack);
  rightMotor = new Motor(rMotorEnable, rMotorForward, rMotorBack);


}

void loop() {
  leftMotor->forward(70);
  //rightMotor->forward(100);
  Serial.print(pulseCountRight); Serial.print("\t \t");   Serial.println(pulseCountLeft);

}
