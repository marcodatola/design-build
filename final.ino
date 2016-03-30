SYSTEM_MODE(SEMI_AUTOMATIC);

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */


#define LED_PIN D7 

bool blinkState = false;

// ================================================================
// 							INTERNAL TIMER						===
// ================================================================

#include "SparkIntervalTimer.h"
IntervalTimer myTimer;
volatile int timer=0;

void incrementTimer(){
	Serial.println("Testing whether volatile works:");
	Serial.print(timer); Serial.print("->");
	timer++;
	Serial.print(timer);
}

// ================================================================
//				SWORD TIP PRESSED INTERRUPT ROUTINE				===
// ================================================================
volatile bool swordtipInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void tipPressed() {
    swordtipInterrupt = !swordtipInterrupt;
}

// ==================================================================
// ===						PRESSURE SENSOR						  ===
// ==================================================================
int PS1AnalogPin = A0; // PS1 is connected to analog 0
int PS2AnalogPin = A1; //  PS2 is connected to analog 0 
int LED1pin = 4;     
int LED2pin = 5;
int PS1Reading;      // the analog reading from the FSR resistor divider
int PS2Reading;      // the analog reading from the FSR resistor

int PS1Min = 0;
int PS1Max = 1023;
int PS2Min = 0;
int PS2Max = 1023;

int readPressure(){
  PS1Reading = analogRead(PS1AnalogPin);
  PS1Reading = map(PS1Reading,PS1Min,PS1Max, 0.0,10.0);
  PS1Reading = constrain(PS1Reading, 0.0, 10.0);

  PS2Reading = analogRead(PS2AnalogPin);
  PS2Reading = map(PS2Reading,PS2Min,PS2Max, 0.0,10.0);
  PS2Reading = constrain(PS2Reading, 0.0, 10.0);
  
  if(PS1Reading!=0 ||PS2Reading!=0 )
	  Serial.println("we should be interrupting timer here");
  
  Serial.print("Pressure1 = ");
  Serial.println(PS1Reading);
  Serial.println(" ");

  Serial.print("Pressure2 = ");
  Serial.println(PS2Reading);
  Serial.println(" ");
  
  // For the LEDS
 
  if (PS1Reading > PS2Reading){
    while (PS1Reading != 0){
   digitalWrite(LED1pin, HIGH);
   digitalWrite(LED2pin, LOW);
    }
  }
  else if (PS1Reading < PS2Reading) {
    while (PS2Reading != 0){
   digitalWrite(LED1pin, HIGH);
   digitalWrite(LED2pin, LOW);
    }
  }
  else {
    digitalWrite(LED1pin, LOW);
    digitalWrite(LED2pin, LOW);
    
  }
  Serial.println("about to return from readPressure()");
  return (PS1Reading||PS2Reading) ;
}


// ================================================================
// ===               			DMP				                ===
// ================================================================

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;
VectorFloat gravity; 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===						TCP CONNECTION						===
// ================================================================
#define DATA_QUAT 0.00
#define DATA_EUL 1.00
#define DATA_YPR 2.00
#define DATA_AAR 3.00
#define DATA_AAW 4.00
#define DATA_PRESS 5
#define DATA_TIME 6
#define TRANS_HIT 1
#define TRANS_FAULT 0
#define TRANSMISSION_ATTEMPTS 5
#define FAULT_HIT -1
TCPClient client;
byte server[] = { 192, 168, 1, 28 }; // to the queen(one as a server photon)  wlan IP
bool startstart=false;
bool start=true;
char buff[512]	;
bool receivedStart(){

	int readVal = client.read();
	Serial.print("I have read"); Serial.println(readVal);
	if(readVal==49){//VB proogram sends an ASCII 1. we are unable to convert from ascii to int oin VB side
		Serial.println("WE HAVE RECEIVED THE RIGHT VALUE");
		Serial.println();
		Serial.println();
		Serial.println();
		Serial.println();
		delay(5000);
		client.flush();  //make sure that there is nothing to be read: client.connected() returns true if connection closed and data needs to be read.
		return true;
	}
	else
		Serial.println("ERROR: we received value: "); Serial.print(readVal);
		return false;
}


bool sendIMUPacket(float cmd, float d0 ,float d1, float d2, float d3){
	
	sprintf(buff,"%f\nyaw:\t%f\npitch:\t%f\nroll:\t%f\0",cmd,d0,d1,d2);	
	Serial.println("about to write");	
	return (client.write((const uint8_t*)&buff, sizeof(buff))>0)? true:false;
}

bool sendPressurePacket(int cmd){
	if(cmd){
		sprintf(buff,"Pressure\n%d\n%d\t%d\0",DATA_PRESS,PS1Reading,PS2Reading);
		
	}
	else{
		sprintf(buff,"Pressure\n%d\n%d\t%d\0",DATA_PRESS,FAULT_HIT,FAULT_HIT);

	}
	return (client.write((const uint8_t*)&buff, sizeof(buff))>0)? true:false;
}

bool sendTimePacket(){
	sprintf(buff,"Timer\n%d\n%ld\0",DATA_TIME,timer);
	return (client.write((const uint8_t*)&buff, sizeof(buff))>0)? true:false;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
	Wire.setSpeed(CLOCK_SPEED_400KHZ);
	Wire.begin();
	
	WiFi.on();
	WiFi.connect();

	while(!WiFi.ready())
	{
		Spark.process();
		delay(100);                 
	}
	Spark.process();

	WiFi.ping(WiFi.gatewayIP()); 

	Serial.begin(115200);

	while (!Serial){
	Spark.process(); 
	delay(100);
	}
	
	Serial.println("connecting to wifi");
    
    //while(!WiFi.ready());
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.subnetMask());
    Serial.println(WiFi.gatewayIP());
    Serial.println(WiFi.SSID());

  Serial.println("connecting to server");

  if (client.connect(server, 2222))
  {
    Serial.println("server connected");
	
	//delay(5000);
  }
  else
  {
    Serial.println("connection failed");
	//delay(5000);
  }
  
	
    

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your  offsets here, scaled for sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for mpu6050 chip

    // make sure it worked (returns 0 if so)
	if (devStatus == 0) {
			// turn on the DMP, now that it's ready
			Serial.println(F("Enabling DMP..."));
			mpu.setDMPEnabled(true);
			attachInterrupt(D2, dmpDataReady, RISING);
			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			Serial.println(F("DMP ready! Waiting for first interrupt..."));
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();
			mpu.setDMPEnabled(false);
			
			
			
	} else {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
		}
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
// ================================================================
// === read incoming start ========================================
//	============== start timer ====================================
// ======================read IMU data ============================
// ============================when tip pressed====================
// =================================read pressure==================
// ====================================Y=======N===================
// =================================send======send=================
// ===========================time&pressure======fault=============
void loop() {
	start=true;
	if(start){
		digitalWrite(LED1pin,LOW);
		digitalWrite(LED2pin,LOW);
		startstart = receivedStart();
		if(startstart) Serial.println("start start true");
		else Serial.println("start start false");
		delay(3000);
		Serial.println("I am waiting for start signal");
		start=false;
		Serial.println("Starting Internal timer");
		myTimer.interrupt_SIT(INT_ENABLE);
		myTimer.begin(incrementTimer,2,hmSec);
		mpu.setDMPEnabled(true);
		mpu.resetFIFO();
	}
	while(startstart){
		 // if programming failed, don't try to do anything
		if (!dmpReady) {Serial.println("DMP NOT READY, breaking");break;}

		// wait for MPU interrupt or extra packet(s) available
		while (!mpuInterrupt && fifoCount < packetSize) {
			Spark.process();
		}

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if ((mpuIntStatus & 0x02) > 0) {
			// wait for correct available data length, should be a VERY short wait
			while (!mpuInterrupt && fifoCount < packetSize) {
					if(swordtipInterrupt){
						
						startstart = false;
						mpu.setDMPEnabled(false);
						//mpu.resetFIFO();
						swordtipInterrupt=false;
						myTimer.interrupt_SIT(INT_DISABLE);
						int i=0;
						if(readPressure()){	
							if(!sendPressurePacket(TRANS_HIT))Serial.println("Could not send pressure packet");;
								
						}else{
							if(!sendPressurePacket(TRANS_FAULT))Serial.println("Could not send pressure packet");		
						}
						if(!sendTimePacket()) Serial.println("Could not send time packet");

						break;
					}else break;	
				}

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			
			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;


			// display Euler angles in degrees
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			Serial.print("ypr\t");
			sendIMUPacket(1,ypr[0] * 180/M_PI,ypr[1] * 180/M_PI,ypr[2] * 180/M_PI,0);
			Serial.print(ypr[0] * 180/M_PI);
			Serial.print("\t");
			Serial.print(ypr[1] * 180/M_PI);
			Serial.print("\t");
			Serial.println(ypr[2] * 180/M_PI);
			
			//display activity
			blinkState = !blinkState;
			digitalWrite(LED_PIN, blinkState);
		}
	}
}
