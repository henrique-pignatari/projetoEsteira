#include<Wire.h>
#include<LiquidCrystal_I2C.h>
#include <Servo.h>

LiquidCrystal_I2C lcd(0x27,16,2);

String expectedMaterial = "";
int period = 3000;
unsigned long time_now = 0;
bool showCount = false;

const int numOfmat = 4;
String materials[] = {"METAL", "PLASTICO", "PAPEL", "VIDRO"};
int trashCount[] = {0, 0, 0, 0};

class IHM{
  private:
  LiquidCrystal_I2C* _lcd;

  public:
  IHM(LiquidCrystal_I2C* lcd){
    _lcd = lcd;
  };

  void init(){
    _lcd->init();
    displayECA();
  }

  void displayECA(){
    _lcd->setBacklight(HIGH);
    _lcd->setCursor(2,0);
    _lcd->print("Recicladora");
    _lcd->setCursor(0,1);
    _lcd->print("ENG. ECA. 8 SEM");
  }

  void displayCount(){
    _lcd->setBacklight(HIGH);
    _lcd->setCursor(0,0);

    for(int i = 0; i < 4; i++){
      _lcd->print(materials[i].substring(0,3)+ ": " + trashCount[i] + "  ");

      if(i == 1){
        _lcd->setCursor(0,1);
      }
    }
  }

  void refresh(){
    if(millis() - time_now > period){
      time_now = millis();
      showCount = !showCount;
      _lcd->clear();
    }

    if(showCount){
      displayCount();
    }else{
      displayECA();
    }
  }
};

class Motor{
  private:
  int _control_pin;

  public:
  Motor(int pin){
    _control_pin = pin;
    pinMode(pin,OUTPUT);
  }
  
  void turnOn(){
    digitalWrite(_control_pin,HIGH);
  }
  void shutDown(){
    digitalWrite(_control_pin,LOW);
  }

  void setSpeed(int vel){
    Serial.println("Mudando velocidade para " + vel);
    //IMPLEMENTAÇÃO DO METODO
  }
};

class Sensor{
  private:
  int _sensor_pin;
  String _name;
  
  public:
  
  Sensor(int pin, String name){
    pinMode(pin, INPUT);
    _sensor_pin = pin;
    _name = name;
  }
  
  bool read(){
    return digitalRead(_sensor_pin);
  }

  String getName(){
    return _name;
  }
};

class OpticalSensor : public Sensor{
  
  public:
    OpticalSensor(int pin, String name) : Sensor(pin, name){
    }
};

class IDSensor : public Sensor{
  
  public:
  String _material;

  IDSensor(int pin, String name, String material) : Sensor(pin, name){
    _material = material;
  }
};

#define ID_SENSOR_COUNT 3

class IDSensorArray{

  private:
  int _count;
  bool states[ID_SENSOR_COUNT];
  IDSensor* sensors[ID_SENSOR_COUNT];
  
  public:
  IDSensorArray(){}

  void addNew(IDSensor* sensor){
    if(_count < ID_SENSOR_COUNT){
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
    
  String getMaterial(){
    for(int i = 0; i < ID_SENSOR_COUNT; i++){
      states[i];
      if(states[i]){
        return sensors[i]->_material;
      } 
    }
    return "NONE";
  }
  
  void resetState(){
    for(int i = 0; i < ID_SENSOR_COUNT; i++){
      states[i] = false;
    }
  }
};

#define OP_SENSOR_COUNT 5

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

#define SERVO_COUNT 3

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
};

#define MOTOR_PIN 8

#define SERVO_METAL 9
#define SERVO_VIDRO 10
#define SERVO_PLASTICO 11

#define IND_SENS_PIN 2
#define CAP_SENS_VIDRO 3
#define CAP_SENS_PLASTICO 4

#define OP_SENS_FINAL 5
#define OP_SENS_METAL 6
#define OP_SENS_VIDRO 7
#define OP_SENS_PLASTICO 8
#define OP_SENS_PAPEL 12

#define RESET_BUTTON 13

bool cycleReady = true;

Motor motor(MOTOR_PIN);

IDSensor sensorInd(IND_SENS_PIN, "INDUTIVO","METAL");
IDSensor sensorCap1(CAP_SENS_VIDRO, "CAPACITIVO 1","VIDRO");
IDSensor sensorCap2(CAP_SENS_PLASTICO, "CAPACITIVO 2","PLASTICO");

OpticalSensor sensorOP1(OP_SENS_FINAL, "FINAL");

OpticalSensor sensorOP2(OP_SENS_METAL, "METAL");
OpticalSensor sensorOP3(OP_SENS_VIDRO, "VIDRO");
OpticalSensor sensorOP4(OP_SENS_PLASTICO, "PLASTICO");
OpticalSensor sensorOP5(OP_SENS_PAPEL, "PAPEL");

IDSensorArray IDsensors;
OPSensorArray OPsensors;

IHM ihm(&lcd);

Servo servo1;
Servo servo2;
Servo servo3;

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

  Servos.addNew(&servoMetal);
  Servos.addNew(&servoVidro);
  Servos.addNew(&servoPlastico);

  ihm.init();
}

void loop()

{
  //ATUALIZA IHM
  ihm.refresh();
  if(digitalRead(RESET_BUTTON)){
    for(int i = 0; i < numOfmat; i++){
      trashCount[i] = 0;
    }
  }
  
  // CONFERE SE O CILO ESTÁ PRONTO
  if(cycleReady){
    //CONFERE TODOS OS SENSORES
    IDsensors.checkAll(); 
    OPsensors.checkAll();
    
    //CONFERE O ESTADO DOS SENSORES 
    if(IDsensors.getMaterial() != "NONE" || OPsensors.getState() == "FINAL"){

      expectedMaterial = IDsensors.getMaterial();

      if(expectedMaterial == "NONE"){
        expectedMaterial = "PAPEL";
      }
      
      Servos.moveServo(expectedMaterial, false);

      IDsensors.resetState();
      OPsensors.resetState();
      cycleReady = false;
    }
  }else{

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
    } 
    OPsensors.resetState();
  }
}