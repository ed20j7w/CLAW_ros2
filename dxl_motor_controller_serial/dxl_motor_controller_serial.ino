#include <Dynamixel2Arduino.h>
#define PIN_LED         (32u)
#define LED_BUILTIN     PIN_LED
#define DEBUG_SERIAL    Serial
#define DXL_SERIAL      Serial1
#define CONTROL_SERIAL  Serial2

// Dynamixal Motor Config
float DXL_PROTOCOL_VERSION = 2.0;
int MAX_DRIVE_VELOCITY = 445;
int MAX_CLAW_VELOCITY = 445;
int LEFT_DRIVE_ID = 2;
int LEFT_CLAW_ID = 1;
int RIGHT_DRIVE_ID = 3;
int RIGHT_CLAW_ID = 4;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

struct Vector2int
{
  int left, right;
};

class Car{
  private:
    uint8_t left_drive_id;
    uint8_t right_drive_id;
    uint8_t left_claw_id;
    uint8_t right_claw_id;
    uint8_t claw_presets[4][2];
    uint8_t claw_preset = 0;
    float max_drive_velocity;
    Vector2int drive_velocity = {0, 0};
    float max_claw_velocity;
    Vector2int claw_velocity = {0, 0};

    void set_drive_velocity(int linear, int angular){
      // +ang is anti-clockwise/left
      int left = linear - angular/2;
      int right = linear + angular/2;
      
      // limit velocity: -100<vel<100
      left = constrain(left, -100, 100);
      right = constrain(right, -100, 100);
      
      // set % of max vel
      left = int(left*this->max_drive_velocity/100);
      right = int(right*this->max_drive_velocity/100);
      this->drive_velocity.left = left;
      this->drive_velocity.right = right;
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.print("Left drive velocity: ");
      DEBUG_SERIAL.println(left);
      DEBUG_SERIAL.print("Right drive velocity: ");
      DEBUG_SERIAL.println(right);
    }

    void set_claw_velocity(int velocity){
      this->claw_velocity.left = velocity*this->max_claw_velocity/100;
      this->claw_velocity.right = velocity*this->max_claw_velocity/100;
      DEBUG_SERIAL.print("Left claw velocity: ");
      DEBUG_SERIAL.println(this->claw_velocity.left);
      DEBUG_SERIAL.print("Right claw velocity: ");
      DEBUG_SERIAL.println(this->claw_velocity.right);
    }

    // void set_claw_preset(){
    //   // read claw position
    //   // save claw value at this->claw_presets[claw_preset]
    // }

  public:
    Car(int max_drive_velocity, uint8_t left_drive_id, uint8_t right_drive_id, 
        int max_claw_velocity, uint8_t left_claw_id, uint8_t right_claw_id){
      this->max_drive_velocity = max_drive_velocity;
      this->left_drive_id = left_drive_id;
      this->right_drive_id = right_drive_id;
      this->max_claw_velocity = max_claw_velocity;
      this->left_claw_id = left_claw_id;
      this->right_claw_id = right_claw_id;
    }

    // read control serial for linear and angular velocity and claw command
    void process_command(String command){
      // Separate by comma
      int index_comma = command.indexOf(',');
      int index_semi = command.indexOf(';');

      // separate goal velocities
      String linear_vel = command.substring(0, index_comma-1);
      String angular_vel = command.substring(index_comma+1, index_semi-1);
      String claw_command = command.substring(index_semi+1, command.length());

      DEBUG_SERIAL.print("Reading command: ");
      DEBUG_SERIAL.println(command);
      DEBUG_SERIAL.println("Linear vel: " + linear_vel);
      DEBUG_SERIAL.println("Angular vel: " + angular_vel);
      DEBUG_SERIAL.println("CLAW command: " + claw_command);

      // Turn on the LED to indicate that a message was received
      digitalWrite(PIN_LED, HIGH);

      this->set_drive_velocity(linear_vel.toInt(), angular_vel.toInt());
      this->set_claw_velocity(claw_command.toInt());
    }

            
    Vector2int get_drive_velocity(){ return this->drive_velocity; }
    Vector2int get_claw_velocity(){ return this->claw_velocity; }
    Vector2int get_drive_id(){ return {this->left_drive_id, this->right_drive_id}; }
    Vector2int get_claw_id(){ return {this->left_claw_id, this->right_claw_id}; }
};

// setup dynamixel port
Dynamixel2Arduino dxl(DXL_SERIAL);

// initialise car
Car car(MAX_DRIVE_VELOCITY, LEFT_DRIVE_ID, RIGHT_DRIVE_ID, MAX_DRIVE_VELOCITY, LEFT_CLAW_ID, RIGHT_CLAW_ID);

void setup() {
  // Initialize the UART serial communication at 9600 baud rate for ESP32
  CONTROL_SERIAL.begin(9600);
  DEBUG_SERIAL.begin(9600);

  // Initialize the LED pin as an output
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW); // Ensure LED is off initially

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(LEFT_DRIVE_ID);
  dxl.setOperatingMode(LEFT_DRIVE_ID, OP_VELOCITY);
  dxl.torqueOn(LEFT_DRIVE_ID);

  dxl.torqueOff(RIGHT_DRIVE_ID);
  dxl.setOperatingMode(RIGHT_DRIVE_ID, OP_VELOCITY);
  dxl.torqueOn(RIGHT_DRIVE_ID);

  dxl.torqueOff(LEFT_CLAW_ID);
  dxl.setOperatingMode(LEFT_CLAW_ID, OP_VELOCITY);
  dxl.torqueOn(LEFT_CLAW_ID);

  dxl.torqueOff(RIGHT_CLAW_ID);
  dxl.setOperatingMode(RIGHT_CLAW_ID, OP_VELOCITY);
  dxl.torqueOn(RIGHT_CLAW_ID);
}

void loop() {
  // check serial
  if (CONTROL_SERIAL.available() > 0) {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println("Message Received.");
    String received = CONTROL_SERIAL.readStringUntil('\n');
    
    car.process_command(received);
  }

  // Set Goal Velocity RAW value
  dxl.setGoalVelocity(car.get_drive_id().left, car.get_drive_velocity().left);
  dxl.setGoalVelocity(car.get_drive_id().right, car.get_drive_velocity().right);
  dxl.setGoalVelocity(car.get_claw_id().left, car.get_claw_velocity().left);
  dxl.setGoalVelocity(car.get_claw_id().right, car.get_claw_velocity().right);

  // Turn off the LED after a short delay
  static unsigned long lastMillis = 0;
  if (millis() - lastMillis > 2000) {
    lastMillis = millis();
    digitalWrite(PIN_LED, LOW);
  }
}