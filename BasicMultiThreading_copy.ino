#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ESP32Servo.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define ANALOG_INPUT_PIN A0

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2 // Specify the on which is your LED
#endif

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

unsigned int localPort = 8888;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back
char recBuffer[100];

EthernetUDP Udp;

short PcMotorValue[2] = {1500,1500};

short RCPin[4] = {34,35,32,33};
short RCValue[4] = {1500,1500,1500,1500};
volatile long startTimeRC[4] = {0,0,0,0};
volatile long currentTimeRC[4] = {0,0,0,0};
volatile long pulseRC[4] = {0,0,0,0};

Servo motorLeft;
Servo motorRight;
#define motorLeftPin 26
#define motorRightPin 25

int minUs = 450;
int maxUs = 2550;

ESP32PWM pwm;

// Define two tasks for Blink & AnalogRead.
void TaskBlink( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void TaskRecieveUDP(void *pvParameters);
void TaskDriveMotor(void *pvParameters);
TaskHandle_t analog_read_task_handle; // You can (don't have to) use this to be able to manipulate a task from somewhere else.
TaskHandle_t udp_read_task_handle; // You can (don't have to) use this to be able to manipulate a task from somewhere else.
TaskHandle_t drive_motor_task_handle; // You can (don't have to) use this to be able to manipulate a task from somewhere else.

void IRAM_ATTR pulseTimer(short idx) {
  currentTimeRC[idx] = micros();
  if(currentTimeRC[idx] > startTimeRC[idx]){
    pulseRC[idx] = currentTimeRC[idx] - startTimeRC[idx];
    startTimeRC[idx] = currentTimeRC[idx];
  }
}

void IRAM_ATTR pulseTimer0() {
  pulseTimer(0);
}

void IRAM_ATTR pulseTimer1() {
  pulseTimer(1);
}

void IRAM_ATTR pulseTimer2() {
  pulseTimer(2);
}

void IRAM_ATTR pulseTimer3() {
  pulseTimer(3);
}

// The setup function runs once when you press reset or power on the board.
void setup() {
  pinMode(RCPin[0], INPUT_PULLUP);
  pinMode(RCPin[1], INPUT_PULLUP);
  pinMode(RCPin[2], INPUT_PULLUP);
  pinMode(RCPin[3], INPUT_PULLUP);
  attachInterrupt(RCPin[0], pulseTimer0, CHANGE);
  attachInterrupt(RCPin[1], pulseTimer1, CHANGE);
  attachInterrupt(RCPin[2], pulseTimer2, CHANGE);
  attachInterrupt(RCPin[3], pulseTimer3, CHANGE);

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);

  motorLeft.attach(motorLeftPin, minUs, maxUs);
	motorRight.attach(motorRightPin, minUs, maxUs);
  
  // Initialize serial communication at 115200 bits per second:
  Ethernet.init(5);
  Ethernet.begin(mac, ip);
  
  Serial.begin(9600);
  // Set up two tasks to run independently.
  uint32_t blink_delay = 1000; // Delay between changing state on LED pin

  Serial.printf("Basic Multi Threading Arduino Example\n");

  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start UDP
  Udp.begin(localPort);
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.

  xTaskCreate(
    TaskBlink
    ,  "Task Blink" // A name just for humans
    ,  2048        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  (void*) &blink_delay // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  2  // Priority
    ,  NULL // Task handle is not used here - simply pass NULL
    );

  xTaskCreate(
    TaskDriveMotor
    ,  "Task Drive Motor" // A name just for humans
    ,  1024        // The stack size can be checked by calling `uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);`
    ,  NULL // Task parameter which can modify the task behavior. This must be passed as pointer to void.
    ,  2  // Priority
    ,  &drive_motor_task_handle // Task handle is not used here - simply pass NULL
    );

  // This variant of task creation can also specify on which core it will be run (only relevant for multi-core ESPs)
  xTaskCreatePinnedToCore(
    TaskAnalogRead
    ,  "Analog Read"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  2  // Priority
    ,  &analog_read_task_handle // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );

  xTaskCreatePinnedToCore(
    TaskRecieveUDP
    ,  "UDP Read"
    ,  2048  // Stack size
    ,  NULL  // When no parameter is used, simply pass NULL
    ,  1  // Priority
    ,  &udp_read_task_handle // With task handle we will be able to manipulate with this task.
    ,  ARDUINO_RUNNING_CORE // Core on which the task will run
    );
}

void loop(){
  // if(analog_read_task_handle != NULL){ // Make sure that the task actually exists
  //   delay(10000);
  //   vTaskDelete(analog_read_task_handle); // Delete task
  //   analog_read_task_handle = NULL; // prevent calling vTaskDelete on non-existing task
  // }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters){  // This is a task.
  uint32_t blink_delay = *((uint32_t*)pvParameters);

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;){ // A Task shall never return or exit.
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    // arduino-esp32 has FreeRTOS configured to have a tick-rate of 1000Hz and portTICK_PERIOD_MS
    // refers to how many milliseconds the period between each ticks is, ie. 1ms.
    delay(blink_delay);
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(blink_delay);
  }
}

void TaskDriveMotor(void *pvParameters){ 
  (void) pvParameters;

  for (;;){
    motorLeft.writeMicroseconds(RCValue[0]);
    motorRight.writeMicroseconds(RCValue[1]);
    delay(10);
  }
}

void TaskAnalogRead(void *pvParameters){  // This is a task.
  (void) pvParameters;

  for (;;){
    if (pulseRC[0] <= maxUs && pulseRC[0] >= minUs){
      RCValue[0] = pulseRC[0];
    }

    if (pulseRC[1] <= maxUs && pulseRC[1] >= minUs){
      RCValue[1] = pulseRC[1];
    }

    if (pulseRC[2] <= maxUs && pulseRC[2] >= minUs){
      RCValue[2] = pulseRC[2];
    }

    if (pulseRC[3] <= maxUs && pulseRC[3] >= minUs){
      RCValue[3] = pulseRC[3];
    }

    Serial.printf("0: %d   1: %d   2: %d   3: %d  ||  PC1: %d   PC2: %d\n", RCValue[0], RCValue[1], RCValue[2], RCValue[3], PcMotorValue[0], PcMotorValue[1]);
    delay(50);
  }
}

void TaskRecieveUDP(void *pvParameters){  // This is a task.
  (void) pvParameters;

  for (;;){
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      IPAddress remote = Udp.remoteIP();
      Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
      if (packetBuffer[0] == 'i' && 
        packetBuffer[1] == 't' &&
        packetBuffer[2] == 's'
      ){
        memcpy(&PcMotorValue[0],packetBuffer+3,2);
        memcpy(&PcMotorValue[1],packetBuffer+5,2);
      }
      // send a reply to the IP address and port that sent us the packet we received
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(ReplyBuffer);
      Udp.endPacket();
    }
    delay(1);
  }
}
