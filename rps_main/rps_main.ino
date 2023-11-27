//Including libraries for all functionalities
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <Stepper.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// for servo face, 5 = smile, 165 = frown
//Servo setup
int winPos = 5;
int losePos = 165;
Servo faceServo;
int faceServoPos = 85;
int faceServoPin = 19;
int win = 0;

//Lidar setup
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
bool in_range = false;

//Button variables
#define BUTTON_PIN 21
int lastState = HIGH; // the previous state from the input pin
int currentState; 

//Stepper motor variables
const int  steps_per_rev_init = 200;
const int dirPinSymbols = 12;
const int stepPinSymbols = 14;
Stepper symbolsStepper(steps_per_rev_init, dirPinSymbols, stepPinSymbols);

const int dirPinPulse = 15;
const int stepPinPulse = 2;
Stepper pulseStepper(steps_per_rev_init, dirPinPulse, stepPinPulse);

//Symbol 4 times faster than pulse
const int  steps_per_rev = 4 * steps_per_rev_init;

int degPulse = 45;
int ratioCircle = 360 / degPulse;
int stepsPerPulse = 2 * steps_per_rev / ratioCircle; //Down and up

int posSymbol = 0;
int posPulse = 0;

//Adjusted rate for updated Stepper library
int numPulses = 3;
int stepsToRun = stepsPerPulse * numPulses;
float pulseRate = 0.5;
int pulseFreq = (int) 2000 / pulseRate;
float symbolRate = 4;
int symbolFreq = (int) 2000 / symbolRate;

int rateAdjustment = 5;

int resetSymbolSteps;


//Game variables
bool startGame = true;
bool cheat = false;
bool finding = false;
bool drivingForward = false;

int randSymbolPos;

//Wifi variables
const char* ssid = "RPS-Network";
const char* password = "123456789"; 

AsyncWebServer server(80);


//ESPNow Variables
// MAC Address of camera
//Aknazar Cam
uint8_t broadcastAddress_symbol[] = {0x48, 0xE7, 0x29, 0x8A, 0x0D, 0x58};

//Ryan Cam
uint8_t broadcastAddress_frontCam[] = {0xA8, 0x42, 0xE3, 0xCE, 0x6F, 0xC8};

esp_now_peer_info_t peerInfo;

// Define variables to store incoming readings
bool forward;
bool hand;

// Define variables to receive from ESP
int id;
int x;
int y;
int obj;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
struct struct_message_incoming {
  int id;
  int x;
  int y;
  int obj;
};

// Create a struct_message to hold which sensor to turn on
struct_message_incoming incomingReadings;

struct struct_message_outgoing {
  bool forward;
  bool hand;
};

// Create a struct_message to hold incoming sensor readings
struct_message_outgoing outgoingData;

//DC Motor Code
// Motor A
int enA = 26;
int in1 = 4;
int in2 = 0;
 
// // Motor B
int enB = 27;
int in3 = 17;
int in4 = 16;

const int frequency = 400;
const int pwm_channelA = 0; //set ledc channel 0 to GPIO 14
const int pwm_channelB = 1; //set ledc channel 1 to GPIO 12
const int resolution = 8;

int speedGo = 255;
int turnSpeed = 255;

bool dirGo = 0; // 0 = Backward, 1 = Forward
bool dirTurn = 0; // 0 = Left, 1 = Right

int duratGO = 1000; //duration to go in milliseconds
int duratTurn = 2000;

void setup()
{
  Serial.begin(115200);
  //Button setup
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  //Stepper setup
  pulseStepper.setSpeed(pulseRate * 100);
  symbolsStepper.setSpeed(symbolRate * 100);
  Serial.println("Begin setup");

  //ESPNow code
// Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress_symbol, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer symbol");
    return;
  }

    // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress_frontCam, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer front cam");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // //Lidar setup
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }


  // //Wifi code
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  server.on("/cheat",HTTP_POST,[](AsyncWebServerRequest * request){},
    NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {
      String val = String((char *)data_in, len); // takes the given value 
      if (val == "1"){
        cheat = false;
        request->send_P(200, "text/plain", cheat_return().c_str()); 
      }
      if (val == "2"){
        cheat = true;
        request->send_P(200, "text/plain", cheat_return().c_str()); 
      }
      else {
        request->send(200);
      }
  });

  //DC Setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  ledcSetup(pwm_channelA, frequency, resolution);
  ledcAttachPin(enA, pwm_channelA);

  ledcSetup(pwm_channelB, frequency, resolution);
  ledcAttachPin(enB, pwm_channelB);

  faceServo.attach(faceServoPin); 
  faceServo.write(faceServoPos);
  Serial.println("Setup over");
}

//Game Loop
void loop()
{
  //Button push drives full game behavior
  currentState = digitalRead(BUTTON_PIN);

  if(lastState == LOW && currentState == HIGH){
    playGame();
  }

  // save the last state
  lastState = currentState;
}

//Play game function that holds steps of full game
void playGame(){
  findPlayer();
  randSymbolPos = random(0,3);
  delay(2000);
  moveSteppers();
  startHandCam();
  Serial.println("End of game");
}

//Print corresponding gesture from symbol
void printSymbol(int randPos, bool hand){
  if(hand){
    Serial.print("Person picks ");
  } else {
    Serial.print("Robot picks ");

  //Mapping of gesture
  if(randPos == 0){
    Serial.println("Rock");
  } else if (randPos == 1){
    Serial.println("Paper");
  } else{
    Serial.println("Scissors");
  }
}

//stepper code for both steppers, pulse and symbols
void moveSteppers(){

  //Mapping from pulse to symbol mapping, timing ending
  int randPos = randSymbolPos;
  int symbolTotalSteps = (symbolRate/pulseRate) * stepsToRun; //Total steps symbol hand runs with no adjusting
  int adjustedCycles = (symbolTotalSteps / steps_per_rev) - 1; //Cuts off extra steps with int division, then subtracts a cycle
  int symbolStepsPreChoice = adjustedCycles * steps_per_rev;
  int endSymbolPos = (randPos * steps_per_rev / 3) + symbolStepsPreChoice;

  //Loop for pulse
  for(int i = 0; i<stepsToRun; i++)
  {
    //Run symbol multiple times if not at end point
    if(posSymbol < endSymbolPos){
      symbolsStepper.step(-(symbolRate/pulseRate));
      posSymbol+= (symbolRate/pulseRate);
    }

    //Adjust direction for pulse
    int modVal = i % stepsPerPulse;
    if((modVal) >= (steps_per_rev / ratioCircle)){
      pulseStepper.step(-1);
      posPulse -= 1;
    } else {
      pulseStepper.step(1);
      posPulse += 1;
    }
  }

  //Reset values so that new game maps correctly
  int resetSymbolPos = (adjustedCycles + 1) * steps_per_rev;
  resetSymbolSteps = resetSymbolPos - endSymbolPos;
  
  posSymbol = 0;
  posPulse = 0;
}

//Move steppers to reset position
void resetGame(int resetSymbolSteps){
  symbolsStepper.step(-resetSymbolSteps);
}
  

//Outcome of game that prints to Serial and updates servo face
void determineOutcome(){
  //Since directions switched on symbol, new mapping needed
  if(randSymbolPos == 2){
    randSymbolPos = 1;
  } else if(randSymbolPos == 1){
    randSymbolPos = 2;
  }

  Serial.println(obj);
  printSymbol(randSymbolPos, false);
  printSymbol(obj, true);
  //All game scenarios
  if(randSymbolPos == 0 && obj == 1){
    Serial.println("Robot loses!");
    win = 0;
  } else if(randSymbolPos == 0 && obj == 2){
    Serial.println("Robot wins!");
    win = 1;
  } else if(randSymbolPos == 1 && obj == 0){
    Serial.println("Robot wins!");
    win = 1;
  } else if(randSymbolPos == 1 && obj == 2){
    Serial.println("Robot loses!");
    win = 0;
  } else if(randSymbolPos == 2 && obj == 0){
    Serial.println("Robot loses!");
    win = 0;
  } else if(randSymbolPos == 2 && obj == 1){
    Serial.println("Robot wins!");
    win = 1;
  } else {
    Serial.println("Tie game");
    win = 2;
  }

  //Move servo based on win, lose, tie
  if(win == 1){
    faceServo.write(winPos);
    delay(2000);
  } else if(win == 0){
    faceServo.write(losePos);
    delay(2000);
  } else {
    faceServo.write(85);
    delay(2000);
  }
}

//Cheating code, set bool val
String cheat_return(){
  if(cheat){
    return String("Cheating initiated...");
  } else{
    return String("No cheating...");
  }  
}

//Start turning and register front cam to detect
void findPlayer(){
  finding = true;
  // Set values to send
  outgoingData.forward = true;
  outgoingData.hand = false;

  esp_err_t result = esp_now_send(broadcastAddress_frontCam, (uint8_t *) &outgoingData, sizeof(outgoingData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  //Turn cart until finding set to false in callback function
  delay(1000);
  while(finding == true){
    turnCart();
  }
}

//Start detecting on hand cam
void startHandCam(){
  // Set values to send
  outgoingData.forward = false;
  outgoingData.hand = true;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress_symbol, (uint8_t *) &outgoingData, sizeof(outgoingData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received, messages received from front and symbol cam
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  id = incomingReadings.id;
  x = incomingReadings.x;
  y = incomingReadings.y;
  obj = incomingReadings.obj;
  Serial.print("ID: "); Serial.println(id);
  Serial.print("X: "); Serial.println(x);
  Serial.print("Y: "); Serial.println(y);
  Serial.print("Obj: "); Serial.println(obj);
  
  //Reading symbol cam outcome
  if(!finding){
    //Reading done and can determine game outcome
    determineOutcome();
    delay(5000);
    //Reset game
    resetGame(resetSymbolSteps);
  }
  if(obj == 4){
    //Set to false in callback, drive towards player
    finding = false;
    Serial.println("Something detected on the facial cam");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);  
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    driveForward();
  }
}

//Drive towards player while Lidar readings above certain distance
void driveForward(){
  goMotor();
  while(!in_range){
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      if(measure.RangeMilliMeter < 100){
        //Trigger bool val to stop driving
        in_range = true;
      }
    }
  }

  //Shut off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

//DC Motor code to move forward
void goMotor()
 
{
  if (dirGo == 0) {
  // Forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
 
  ledcWrite(pwm_channelA, speedGo);
 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
 
  ledcWrite(pwm_channelB, speedGo);
  Serial.println("Going Forward");

  // delay(duratGO);
  } else {
    // Backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
 
  ledcWrite(pwm_channelA, speedGo);
 
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
 
  ledcWrite(pwm_channelB, speedGo);
  Serial.println("Going Backward");

  // delay(duratGO);
  }
 
}

//Turn cart, used when finding person
void turnCart()
 
{
  // Serial.println("Start of DC Motor code");
  if (dirTurn == 1) {
    // Left
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH); 
  ledcWrite(pwm_channelA, turnSpeed); 
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH); 
  ledcWrite(pwm_channelB, turnSpeed);
  // Serial.println("Turning Left");
  } else {
    // Right
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  ledcWrite(pwm_channelA, turnSpeed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  ledcWrite(pwm_channelB, turnSpeed);  
  // Serial.println("Turning Right");
  }
}