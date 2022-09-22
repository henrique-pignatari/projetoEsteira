class Motor{
  int control_pin;

  public:
  Motor(int pin){
    Serial.println("Montando o motor");
    control_pin = pin;
    pinMode(pin,OUTPUT);
  }
  
  void turnOn(){
    Serial.println("Ligando o motor");
    digitalWrite(control_pin,HIGH);
  }
  void shutDown(){
    Serial.println("Desligando o motor");
    digitalWrite(control_pin,LOW);
  }

  void setSpeed(int vel){
    Serial.println("Mudando velocidade para " + vel);
    //IMPLEMENTAÇÃO DO METODO
  }
};

class Sensor{
  int sensor_pin;
  String name;
  
  public:
  Sensor(int pin){
    Serial.println("Montando sensor");
    pinMode(pin, INPUT);
    sensor_pin = pin;
  }

  void defineName(String name){
    Serial.println("Definindo nome do sensor para " + name);
    this->name = name;
  }
  
  bool read(){
    //Serial.println("Lendo sensor " + name);
    return digitalRead(sensor_pin);
  }
};

boolean state = false;
Motor motor(8);
Sensor sensor(2);

void setup()
{
  Serial.begin(9600);
  sensor.defineName("CAPACITIVO");
}

void loop()
{
  if(sensor.read()){
    state = !state;
    
    if(state){
      motor.turnOn();
      delay(250);
    }else{
      motor.shutDown();
      delay(250);
    }
  }
}
