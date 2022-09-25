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
      return "PAPEL";
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
        for(int i = 0; i < ID_SENSOR_COUNT; i++){
          states[i] = false;
        }
      } 
  };
  
  #define MOTOR_PIN 8
  
  #define IND_SENS_PIN 2
  #define CAP_SENS_PIN 3
  #define CAP_SENS_PIN_2 4
  
  #define OP_SENS_PIN 5
  #define OP_SENS_PIN_2 6
  #define OP_SENS_PIN_3 7
  #define OP_SENS_PIN_4 8
  #define OP_SENS_PIN_5 9

  #define RESET_BUTTON 10
  
  bool cycleReady = true;
  
  Motor motor(MOTOR_PIN);
  
  IDSensor sensorInd(IND_SENS_PIN, "INDUTIVO","METAL");
  IDSensor sensorCap1(CAP_SENS_PIN, "CAPACITIVO 1","VIDRO");
  IDSensor sensorCap2(CAP_SENS_PIN_2, "CAPACITIVO 2","PLASTICO");
  
  OpticalSensor sensorOP1(OP_SENS_PIN, "FINAL");
  
  OpticalSensor sensorOP2(OP_SENS_PIN_2, "METAL");
  OpticalSensor sensorOP3(OP_SENS_PIN_3, "PLASTICO");
  OpticalSensor sensorOP4(OP_SENS_PIN_4, "VIDRO");
  
  IDSensorArray IDsensors;
  OPSensorArray OPsensors;
  
  String expectedMaterial = "";
  String materials[4] = {"METAL", "PLASTICO", "PAPEL", "VIDRO"};
  int trashCount[4] = {0, 0, 0, 0};
  const int numOfmat = 4;
  
  void setup()
  {
    Serial.begin(9600);
    
    IDsensors.addNew(&sensorCap1);
    IDsensors.addNew(&sensorCap2);
    IDsensors.addNew(&sensorInd);
  
    OPsensors.addNew(&sensorOP1);
    OPsensors.addNew(&sensorOP2);
    OPsensors.addNew(&sensorOP3);
    OPsensors.addNew(&sensorOP4);
  }
  
  void loop()
  {
    if(digitalRead(RESET_BUTTON)){
      for(int i = 0; i < numOfmat; i++){
        trashCount[i] = 0;
      }
    }
    
    if(cycleReady){
      IDsensors.checkAll(); 
      OPsensors.checkAll();
      
      if(OPsensors.getState() == "FINAL"){
        expectedMaterial = IDsensors.getMaterial();
        Serial.println(IDsensors.getMaterial());
        IDsensors.resetState();
        OPsensors.resetState();
        cycleReady = false;
      }
    }else{
      Serial.println(OPsensors.getState());
      OPsensors.checkAll();
      String state = OPsensors.getState();
      
      if(state == expectedMaterial){
        for(int i = 0; i < numOfmat; i++){
          if(materials[i] == state){
            trashCount[i]++;
          }
        }
        cycleReady = true;
      } 
      OPsensors.resetState();
      for(int i = 0; i < numOfmat; i++){
        Serial.println(materials[i] + ": " + trashCount[i]);
      }
    }
  }
