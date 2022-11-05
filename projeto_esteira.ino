//INCLUDE LIBRARIES
#include<Wire.h>
#include<LiquidCrystal_I2C.h>
#include <Servo.h>

//DECLARE GLOBALS
LiquidCrystal_I2C lcd(0x27,16,2);

String expectedMaterial = "";
int period = 3000;
unsigned long time_now = 0;
unsigned long materialTime = 0;

bool showCount = false;
bool switchMessage = false;
bool cycleReady = true;

bool erro = false;
int errorTime = 2000;

const int numOfmat = 4;
String materials[] = {"METAL", "PLASTICO", "PAPEL", "VIDRO"};
int trashCount[] = {0, 0, 0, 0};

Servo servo1;
Servo servo2;
Servo servo3;

//GENERAL DEFINES
#define ID_SENSOR_COUNT 3
#define OP_SENSOR_COUNT 6
#define SERVO_COUNT 3

//PIN DEFINES
#define SERVO_METAL 2
#define SERVO_VIDRO 3
#define SERVO_PLASTICO 4

#define MOTOR_RIGHT_PIN 5
#define MOTOR_LEFT_PIN  6

#define IND_SENS_PIN 53
#define CAP_SENS_VIDRO 51
#define CAP_SENS_PLASTICO 49

#define OP_SENS_FINAL 47
#define OP_SENS_START 35
#define OP_SENS_METAL 45
#define OP_SENS_VIDRO 43
#define OP_SENS_PLASTICO 41
#define OP_SENS_PAPEL 39

#define RESET_BUTTON 37

//CLASS IHM FOR CONTROLLING THE LCD DISPLAY
class IHM{
  private:
  LiquidCrystal_I2C* _lcd; //POINTER FOR THE LCD

  public:
  IHM(LiquidCrystal_I2C* lcd){ //CONSTRUCTOR
    _lcd = lcd;
  };

  void init(){ //INITIATE THE LCD AND PRINT THE INITIAL MESSAGE
    _lcd->init();
    displayECA();
  }

  void displayECA(){ //DISPLAYS THE ECA MESSAGE
    _lcd->setBacklight(HIGH);
    _lcd->setCursor(2,0);
    _lcd->print("Recicladora");
    _lcd->setCursor(0,1);
    _lcd->print("ENG. ECA. 8 SEM");
  }

  void displayCount(){ //DISPLAYS THE COUNTER
    _lcd->setBacklight(HIGH);
    _lcd->setCursor(0,0);

    for(int i = 0; i < 4; i++){
      _lcd->print(materials[i].substring(0,3)+ ": " + trashCount[i] + "  ");

      if(i == 1){
        _lcd->setCursor(0,1);
      }
    }
  }

  displayStatus(){
    _lcd->setBacklight(HIGH);
    _lcd->setCursor(0,0);
    String state = "";

    if(cycleReady){
       state = "PRONTO!!";
    }else{
      state = "ESPERE!!";
    }

    if(erro){
      state = "ERRO!!";
    }

    _lcd->print(state);
  }

  void refresh(){ //REFERESHES THE IHM MESSAGE CHANGING THE COUNT NUMBER OR SWITCHING MEESSAGES
    if(millis() - time_now > period){
      time_now = millis();
      showCount = !showCount;
      _lcd->clear();

      if(showCount){
        displayCount();
        switchMessage = !switchMessage;
      }else{
        if(switchMessage){
          displayECA();
        }else{
          displayStatus();
        }
      }
    }
  }
};

//CLASS FOR CONTROLLING THE MOTOR
class Motor{
  public:
  int _RPWMPIN;
  int _LPWMPIN; 
  bool state = false;
  
  Motor(int RPWMPin, int LPWMPin){
    this->_RPWMPIN = RPWMPin;
    this->_LPWMPIN = LPWMPin;
  } 
  void ClockWise(int pwm){
    int command[] = {pwm, 0};
    controlMotor(command);
  }
  void CounterClockWise(int pwm){
    int command[] = {0, pwm};
    controlMotor(command);
  }
  void fullStop(){
    int command[] = {0, 0};
    controlMotor(command);
    state = false;
  } 
  void controlMotor(int command[]){
    analogWrite(_RPWMPIN,command[0]);
    analogWrite(_LPWMPIN, command[1]);
  }  
  void turnOn(){    
    for(int pwm = 0; pwm < 200; pwm++){
      ClockWise(pwm);
      Serial.println(pwm);
      delay(15);
    }
    state = true;
  } 
};

//CLASS FOR CONTROLLING THE SENSORS
class Sensor{
  private:
  int _sensor_pin; //SENSOR PIN
  String _name; //NAME OF THE SENSOR
  
  public:
  
  Sensor(int pin, String name){ //CONSTRUCTOR
    pinMode(pin, INPUT);
    _sensor_pin = pin;
    _name = name;
  }
  
  bool read(){ //READS THE SENSOR AND RETURN ITS VALUE
    return digitalRead(_sensor_pin);
  }

  String getName(){ //RETURNS SENSORS NAME
    return _name;
  }
};

//CLASS FOR OPTICAL SENSORS
class OpticalSensor : public Sensor{ //INHERIT SENSOR CLASS
  
  public:
    OpticalSensor(int pin, String name) : Sensor(pin, name){ //CONSTRUCTOR
    }
};

//CLASS FOR IDENTIFICATION SENSORS
class IDSensor : public Sensor{ //INHERIT SENSOR CLASS
  
  public:
  String _material; //MATERIAL THAT THIS SENSOR HAS TO IDENTIFY

  IDSensor(int pin, String name, String material) : Sensor(pin, name){ //CONSTRUCTOR
    _material = material;
  }
};

// CLASS FOR CONTROLLING ALL THE ID SENSORS AS AN UNIQUE ARRAY
class IDSensorArray{

  private:
  int _count; //HOW MANY SENSORS THIS ARRAY CONSTAINS
  bool states[ID_SENSOR_COUNT]; //STATES OF ALL SENSORS
  IDSensor* sensors[ID_SENSOR_COUNT]; // ALL THE SENSORS POINTERS
  
  public:
  IDSensorArray(){} //CONSTRUCTOR

  void addNew(IDSensor* sensor){ //ADD A NBW SENSOR TO THE ARRAY
    if(_count < ID_SENSOR_COUNT){
      sensors[_count] = sensor;
      _count++; 
    }
  }

  void checkAll(){ // CHECK ALL SENSORS AND REFRESH THEIR STATES
    for(int i = 0; i < _count; i++){
      bool sensorState = sensors[i]->read();

      if(sensorState){
        states[i] = sensorState;
      }
    }
  } 
    
  String getMaterial(){ //RETURN THE MATERIAL THAT WAS IDENTIFIED BY THE SENSORS
    for(int i = 0; i < ID_SENSOR_COUNT; i++){
      states[i];
      if(states[i]){
        return sensors[i]->_material;
      } 
    }
    return "NONE";
  }
  
  void resetState(){ //RESETS THE STATES OF ALL SENSORS
    for(int i = 0; i < ID_SENSOR_COUNT; i++){
      states[i] = false;
    }
  }
};

// CLASS FOR CONTROLLING ALL THE OP SENSORS AS AN UNIQUE ARRAY
class OPSensorArray{
  private:
  int _count;
  bool states[OP_SENSOR_COUNT];
  OpticalSensor* sensors[OP_SENSOR_COUNT];

  public:
  OPSensorArray(){}
  
  void addNew(OpticalSensor* sensor){
    if(_count < OP_SENSOR_COUNT){
      sensors[_count] = sensor;
      _count++;
    }
  }

    void checkAll(){
      for(int i = 0; i < _count; i++){
        bool sensorState = sensors[i]->read();
        
        if(sensorState){
          states[i] = sensorState;
        }
      }
    }

    String getState(){
      for(int i = 0; i < OP_SENSOR_COUNT; i++){
        if(states[i]){
          return(sensors[i]->getName());
        }
      }
      return "NONE";
    }
    
    void resetState(){
      for(int i = 0; i < OP_SENSOR_COUNT; i++){
        states[i] = false;
      }
    } 
};

class Output{
  private:
  Servo* _servo;
  bool state;
  int openAngle = 0;
  int closedAngle = 70;
  String _name;
  
  public:
  Output(Servo* servo, String name){
    _servo = servo;
    _name = name;
    state = false;
  }

  void setServoState(bool state){
    int angle;
    int mod = 0;

    if(state){
      angle = closedAngle;
    }else{
      angle = openAngle;
    }

    int initAngle = _servo->read();
    Serial.println(angle);
    Serial.println(initAngle);

    for(int i = initAngle; i != angle; i+=mod){

      _servo->write(i);

      if(i > angle){
        mod = -1;
      }else{
        mod = 1;
      }

      delay(15);
    }
  }

  String getMaterial(){
    return _name;
  }
};

class ServoArray{
  private:
  int _count;
  Output* servos[SERVO_COUNT];

  public:
  ServoArray(){}

  void addNew(Output* servo){
    if(_count < SERVO_COUNT){
      servos[_count] = servo;
      _count++;
    }
  }

  void moveServo(String name, bool state){
    for(int i = 0; i < _count; i++){
      if(servos[i]->getMaterial() == name){
        servos[i]->setServoState(state);
      }
    }
  }

  void resetAllServos(){
    for(int i = 0; i < _count; i++){
        moveServo(servos[i]->getMaterial(),true);
    }
  }
};

//CLASSES INITIALIZATIONS
Motor motor(MOTOR_RIGHT_PIN,MOTOR_LEFT_PIN);

IDSensor sensorInd(IND_SENS_PIN, "INDUTIVO","METAL");
IDSensor sensorCap1(CAP_SENS_VIDRO, "CAPACITIVO 1","VIDRO");
IDSensor sensorCap2(CAP_SENS_PLASTICO, "CAPACITIVO 2","PLASTICO");

OpticalSensor sensorOP1(OP_SENS_FINAL, "FINAL");
OpticalSensor sensorOP6(OP_SENS_START,"START");

OpticalSensor sensorOP2(OP_SENS_METAL, "METAL");
OpticalSensor sensorOP3(OP_SENS_VIDRO, "VIDRO");
OpticalSensor sensorOP4(OP_SENS_PLASTICO, "PLASTICO");
OpticalSensor sensorOP5(OP_SENS_PAPEL, "PAPEL");

IDSensorArray IDsensors;
OPSensorArray OPsensors;

IHM ihm(&lcd);

Output servoMetal(&servo1, "METAL");
Output servoVidro(&servo2, "VIDRO");
Output servoPlastico(&servo3, "PLASTICO");


ServoArray Servos;

void setup()
{
  Serial.begin(9600);
  
  servo1.attach(SERVO_METAL);
  servo2.attach(SERVO_VIDRO);
  servo3.attach(SERVO_PLASTICO);

  IDsensors.addNew(&sensorCap1);
  IDsensors.addNew(&sensorCap2);
  IDsensors.addNew(&sensorInd);

  OPsensors.addNew(&sensorOP1);
  OPsensors.addNew(&sensorOP2);
  OPsensors.addNew(&sensorOP3);
  OPsensors.addNew(&sensorOP4);
  OPsensors.addNew(&sensorOP5);
  OPsensors.addNew(&sensorOP6);

  Servos.addNew(&servoMetal);
  Servos.addNew(&servoVidro);
  Servos.addNew(&servoPlastico);

  ihm.init();
}

void loop(){
  
  //ATUALIZA IHM
  ihm.refresh();

  if(digitalRead(RESET_BUTTON)){
    if(erro){
      erro = false;
      cycleReady = true;
      Servos.resetAllServos();
      expectedMaterial = "";
      OPsensors.resetState();
      OPsensors.checkAll();

    }else{
      for(int i = 0; i < numOfmat; i++){
      trashCount[i] = 0;
    }
    }
  }

  if(!motor.state){
    motor.turnOn();
  }
  
  // CONFERE SE O CILO ESTÁ PRONTO
  if(cycleReady && !erro){
    //CONFERE TODOS OS SENSORES
    OPsensors.resetState();
    OPsensors.checkAll();

    //CONFERE O ESTADO DOS SENSORES 
    if(OPsensors.getState() == "START"){
      materialTime = millis();
      cycleReady = false;
      IDsensors.resetState();
      OPsensors.resetState();
    }
  }else if(!erro){
    Serial.println(millis() - materialTime);
     if(millis() - materialTime > errorTime){
       erro = true;
     }

    if(expectedMaterial == ""){
      IDsensors.checkAll(); 
      OPsensors.checkAll();

      if(IDsensors.getMaterial() != "NONE" || OPsensors.getState() == "FINAL"){

        expectedMaterial = IDsensors.getMaterial();

        if(expectedMaterial == "NONE"){
          expectedMaterial = "PAPEL";
        }
        
        Servos.moveServo(expectedMaterial, false);
        materialTime = millis();

        IDsensors.resetState();
        OPsensors.resetState();
      }
    }else{
      OPsensors.resetState();
      OPsensors.checkAll();

      String state = OPsensors.getState();
      if(state == expectedMaterial){
        for(int i = 0; i < numOfmat; i++){
          if(materials[i] == state){
            trashCount[i]++;
          }
        }

        Servos.moveServo(expectedMaterial, true);
        cycleReady = true;

        expectedMaterial = "" ;
        OPsensors.resetState();
      } 
    }
  }
}