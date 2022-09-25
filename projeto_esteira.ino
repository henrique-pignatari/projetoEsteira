class Motor{
  private:
  int _control_pin;

  public:
  Motor(int pin){
    Serial.println("Montando o motor");
    _control_pin = pin;
    pinMode(pin,OUTPUT);
  }
  
  void turnOn(){
    Serial.println("Ligando o motor");
    digitalWrite(_control_pin,HIGH);
  }
  void shutDown(){
    Serial.println("Desligando o motor");
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
    //Serial.println("Lendo sensor " + _name);
    return digitalRead(_sensor_pin);
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

#define SENSOR_COUNT 3

class SensorsArray{
  private:
    int _count;
    
  public:
    bool states[SENSOR_COUNT];
    virtual void checkAll();   
    
    void resetState(){
      for(int i = 0; i < SENSOR_COUNT; i++){
        states[i] = false;
      }
    }

    bool getSensorState(int i){
      return states[i];
    }

    int getCount(){
      return _count;
    }

    void setCount(int val){
      _count = val;
    }
};

class IDSensorArray : public SensorsArray{

  private:
  IDSensor* sensors[SENSOR_COUNT];
  
  public:
  IDSensorArray() : SensorsArray(){}

  void addNew(IDSensor* sensor){
    if(getCount() < SENSOR_COUNT){
      sensors[getCount()] = sensor;
      setCount(getCount() + 1); 
    }
  }

  void checkAll(){
      for(int i = 0; i < getCount(); i++){
        bool sensorState = sensors[i]->read();

        if(sensorState){
          states[i] = sensorState;
        }
      }
    } 
  
  String getMaterial(){
    for(int i = 0; i < SENSOR_COUNT; i++){
      getSensorState(i);
      if(getSensorState(i)){
        return sensors[i]->_material;
      } 
    }
    return "NONE";
  }
  
};

class OPSensorArray : public SensorsArray{
  private:
  OpticalSensor* sensors[SENSOR_COUNT];

  public:
  void addNew(OpticalSensor* sensor){
    if(getCount() < SENSOR_COUNT){
      sensors[getCount()] = sensor;
      setCount(getCount() + 1);
    }
  }

    void checkAll(){
      for(int i = 0; i < getCount(); i++){
        bool sensorState = sensors[i]->read();

        if(sensorState){
          states[i] = sensorState;
        }
      }
    } 
};

#define MOTOR_PIN 8

#define IND_SENS_PIN 2
#define CAP_SENS_PIN 3
#define CAP_SENS_PIN_2 4

#define OP_SENS_PIN 5
#define OP_SENS_PIN_2 6

bool cycleReady = true;

Motor motor(MOTOR_PIN);

IDSensor sensorInd(IND_SENS_PIN, "INDUTIVO","METAL");
IDSensor sensorCap1(CAP_SENS_PIN, "CAPACITIVO 1","VIDRO");
IDSensor sensorCap2(CAP_SENS_PIN_2, "CAPACITIVO 2","PLASTICO");

OpticalSensor sensorOP1(OP_SENS_PIN, "OPTICO FINAL");
OpticalSensor sensorOP2(OP_SENS_PIN_2, "OPTICO METAL");

IDSensorArray IDsensors;

void setup()
{
  Serial.begin(9600);
  
  IDsensors.addNew(&sensorCap1);
  IDsensors.addNew(&sensorCap2);
  IDsensors.addNew(&sensorInd);
}

void loop()
{
  if(cycleReady){
    IDsensors.checkAll(); 
    if(sensorOP1.read()){
      Serial.println(IDsensors.getMaterial());
      IDsensors.resetState();
      cycleReady = false;
    }
  }else{
    if(sensorOP2.read()){
      cycleReady = true;
    } 
  }
}
