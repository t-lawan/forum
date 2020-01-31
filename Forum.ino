//www.elegoo.com
//2016.12.12
/************************
  Exercise the motor using
  the L293D chip
************************/
// Declare Packages required for the Ethernet
#include <SPI.h>
#include <Ethernet.h>

// ETHERNET SERVERNSET UO

// Define Mac and IP
byte mac[] = {
  0xA8, 0x61, 0x0A, 0xAE, 0x79, 0xD9
};
IPAddress ip(169, 254, 0, 10);
IPAddress myDns(169, 254, 0, 5);
IPAddress gateway(169, 254, 0, 0);
IPAddress subnet(255, 255, 0, 0);

// Set Port to 5000
EthernetServer server(5000);

// Set pin values
#define SENSOR_POWER_PIN 2
#define SENSOR_SIGNAL_PIN 3
#define SENSOR_GROUND_PIN 4
#define MOTOR_SIGNAL_PIN 6

class PumpController {
  private:
    byte signal_pin;
    boolean isOn;
    long startDelayInMinutes = 10;// 20 minutes
    unsigned long startDelay = startDelayInMinutes * 60 * 1000; // microseconds
  public:
    PumpController(byte signal_pin) {
      this->signal_pin = signal_pin;
      this->isOn = false;
    }
    // Change the number
    long maxTimeToRunInMinutesForMaster = 20; // minutes
    long maxTimeToRunInMinutesForUser = 5; // minutes
    unsigned long maxTimeToRunForMaster = maxTimeToRunInMinutesForMaster * 60 * 1000; // microseconds
    unsigned long maxTimeToRunForUser =  maxTimeToRunInMinutesForUser * 60 * 1000; // 15 seconds in microseconds
    unsigned long previousStopTime = 0; // microseconds
    bool user = false;
    
    void init() {
//      Serial.println("PUMP CLASS INIT");
      pinMode(signal_pin, OUTPUT);
    }

    void initStart() {
      if(millis() - previousStopTime > startDelay) {
//          Serial.print("Master start pump: ");
//          Serial.println(millis());
          user = false;
          start();
      }
    }

    void start() {
      previousStopTime = millis();
      isOn = true;
      digitalWrite(signal_pin, HIGH);
//      Serial.print("PUMP RUNNING!");
//      Serial.println(millis());
    }

    void halt() {
      isOn = false;
      digitalWrite(signal_pin, LOW);
      previousStopTime = millis();
//      Serial.print("PUMP STOPPED!");
//      Serial.println(millis());
    }

    boolean isRunning() {
      return isOn;
    }
};

class SensorController {
  private:
    float calibrationFactor;
    byte interrupt;
    byte signal_pin;
    byte power_pin;
    byte ground_pin;
    int numOfRecordingsForAverage = 20;
  public:
    SensorController(byte signal_pin, byte power_pin, byte ground_pin) {
      // Assign Pin Variables
      this->signal_pin = signal_pin;
      this->power_pin = power_pin;
      this->ground_pin = ground_pin;

      // This is the flowrate at which the pump should be activated
      this->limit = 4;

      // This is required to calculate the flowrate 
      this->calibrationFactor = 4.5;
      
      // This stores the last time sensor recorded information
      this->lastRecordedTime = 0;

      // This counts the number of spins in the flow rate sensor
      SensorController::count = 0;

      // This stores the interrupt number for the signal pin
      this->interrupt = digitalPinToInterrupt(signal_pin);
    }
    unsigned long lastRecordedTime = 0;
    unsigned long timeSinceReadingWasLow = 10000;
    float flowRate = 0.0;
    int maxTimeToRun = 1000;
    static volatile byte count;
    float averageValue = 0.0;
    bool wasLow = false;
    float limit = 7.0;

    void init() {
      // Setting up the pin input and outputs
      pinMode(signal_pin, INPUT);
      pinMode(power_pin, OUTPUT);
      pinMode(ground_pin, OUTPUT);

      // Set power pin to high
      digitalWrite(power_pin, HIGH);

      // Set ground pin to low
      digitalWrite(ground_pin, LOW);

      // Attach the interrupt to call increaseCount when the pin goes from high to low
      attachInterrupt(interrupt, increaseCount, FALLING);
    }

    void read() {
           // Initial Logic
          detachInterrupt(interrupt);
          // Calculates the flowrate 
          flowRate = ((1000.0 / (millis() - lastRecordedTime)) * SensorController::count) / calibrationFactor;

          // Sets the last recorded time
          lastRecordedTime = millis();
          // Resets count variable
          SensorController::count = 0;
          
          // Reattaches the interrupt
          attachInterrupt(interrupt, increaseCount, FALLING);


          // Second Try
//          SensorController::count = 0;
//          interrupts();
//          delay(1000);
//          noInterrupts();
//          flowRate = (SensorController::count * 2.25);
          
    }

    void calculateAverage() {
      averageValue = 0.0;

      for(int i = 0; i < numOfRecordingsForAverage; i++){
          delay(maxTimeToRun);
          if((millis() - lastRecordedTime) > maxTimeToRun) {
              read();
              averageValue = averageValue + flowRate;
          }
      }

      averageValue = averageValue/numOfRecordingsForAverage;       
    }

    static void increaseCount() {
      SensorController::count++;
    }

    boolean isLow() {
      calculateAverage();
//      Serial.print("Average Value: ");
//      Serial.println(averageValue);
      // Returns true of the flowrate is lower than defined limit
      if(averageValue - 1.0 < 0) {
        return true;
      } else {
        return false;
      }
      
    }
};


PumpController pump(MOTOR_SIGNAL_PIN);
SensorController sensor(SENSOR_SIGNAL_PIN, SENSOR_POWER_PIN, SENSOR_GROUND_PIN);

// Set time variables
unsigned long previousStopTime = 0;
// Declaration of count variable
volatile byte SensorController::count;

void setup() {
//  Serial.begin(9600);
  // Check for Ethernet calble
  if (Ethernet.linkStatus() == LinkOFF) {
//    Serial.println("Ethernet cable is not connected.");
  }
  // initialize the Ethernet device not using DHCP:
  Ethernet.begin(mac, ip, myDns, gateway, subnet);

  // print Ip address
//  Serial.print("My IP address: ");
//  Serial.println(Ethernet.localIP());

  // start listening for clients
  server.begin();
//  Serial.println("Server is up");
  // initalise pump and sensor
 pump.init();
 sensor.init();
}


void loop() {
//  
//   wait for a new client:
  EthernetClient client = server.available();

  if (client) {
//    Serial.println("CONNECTION SENT");
    unsigned long timeDelay = 30000; // 30 seconds delay in microseconds
    if(!sensor.wasLow) {
      timeDelay = 10000;
    }
//    
//    if(!pump.isRunning()) {
      // For sensor logic
     if(!pump.isRunning() && ((millis() - pump.previousStopTime) > timeDelay)) {
          // if pump was low x amount of minutes ago
        pump.user = true;
        pump.start();
//        Serial.print("USER STARTED PUMP: ");
//        Serial.println(millis()); 
    }

    Ethernet.maintain();
  }

  if(client) {
//    Serial.println("CLIENT Called");
    client.stop();
//    Serial.println("CLIENT TOP");
   }
  // Start pump if is not already on
  if (!pump.isRunning()) {
    pump.initStart();
  }
//
  // If the sensor value is wrong stop pump
  if(pump.isRunning()) {
    if(sensor.isLow()) {
      // Stop pump
      sensor.wasLow = true;
      // Set pump is low
      pump.halt();
    }
  }
  
  // After x seconds set wasLow to false
  if(sensor.wasLow && ((millis() - sensor.lastRecordedTime) > sensor.timeSinceReadingWasLow)) {
     sensor.wasLow = false;
  }

//   Checking if time elapsed since last stop is greater than maxPumpTime
  if(pump.isRunning()) {
      if(!pump.user) {
        if ((millis() - pump.previousStopTime > pump.maxTimeToRunForMaster)) {
          pump.halt();
        }
     } else {
        if ((millis() - pump.previousStopTime > pump.maxTimeToRunForUser)) {
         pump.halt();
        }
      }
   
    }

}
