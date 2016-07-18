/////////////////////////////////////////////////////////
//
// roboclaw robot. accommodates the following hardware:
//  Drive Motor Controller:
//    - roboclaw 15a motor driver I/O
//    D11=xmit S1
//    D10=recv S2
//
//  Switched on/off power (headlight, gripper open/close):
//    - L298N 2-channel DC motor driver output, via analog pins (used as digital)
//    D16=IN1
//    D17=IN2
//    D14=IN3
//    D15=IN4
//
//  RC radio receive 8 channels
//    - 27ms FrSky CPPM RC input
//   	D8=signal input 
//
/////////////////////////////////////////////////////////

// setup operation and debugging modes
// debugLevels:  0=true debugging, only use for special occasions (birthday parties, etc...)
//               1=no debugging, output values for another serial device to decode
//               2=only show setup and init messages
//               3=show motor directions
//               4=show motor set values
int debugLevel=1;
bool motorsActive=true;




/////////////////////////////////////////////////////////
// misc variable declaration
/////////////////////////////////////////////////////////
int i=0;
bool gripper_enabled=false;
unsigned long motor_stop_after_millis=0;
bool record_tracks=false;
#define record_max_tracks 20
int record_per_ms=500;
int record_left[record_max_tracks];
int record_right[record_max_tracks];
int record_count=0;
unsigned long record_next_millis;
int record_current_left=0;
int record_current_right=0;
int record_misc=0;
int record_playback_speed=75;
String record_left_direction="";
String record_right_direction="";
char buf[50];



/////////////////////////////////////////////////////////
// roboclaw includes
/////////////////////////////////////////////////////////
#include <SoftwareSerial.h>
#include "RoboClaw.h"
SoftwareSerial serial(10,11);
RoboClaw roboclaw(&serial,10000);
#define address 0x80 
// for packet serial mode, set roboclaw mode to 7
int this_motor_value;
String this_motor_direction;
int lastSpeed[3]={0,0,0};



/////////////////////////////////////////////////////////
// LN298N includes (not using speed pins)
/////////////////////////////////////////////////////////
int l298npins[4]={14,15,16,17};



/////////////////////////////////////////////////////////
// FrSky CPPM includes
/////////////////////////////////////////////////////////
#include <CPPM.h>
const uint8_t NUM_CHANNELS = 8;
int cppm_in[NUM_CHANNELS];
int cppm_last[NUM_CHANNELS];
int cppm_this;;
unsigned long cppm_loop_start;
// FrSky CPPM channel 0-7 definitions
int cppm_drive_l_chan  =0;  // drive motors up/down
int cppm_drive_r_chan  =1;  // drive motors left/right
int cppm_servo_chan    =2;  // gripper servos 1 & 2
                            // 3=
int cppm_gripper_chan  =4;  // L298N M1 gripper stop=0, close=255, open=127
int cppm_record_chan   =5;  // start recording tracks
int cppm_headlight_chan=6;  // L298N M2 headlight on=255, off=0
int cppm_nerf_chan     =7;  // nerf gun servo



/////////////////////////////////////////////////////////
// Nerf gun init
/////////////////////////////////////////////////////////
String nerf_mode="";
unsigned long nerf_next_millis=0;

int nerf_stop_speed=200;
int nerf_spinup_speed=320;
int nerf_fire_speed=450;

int nerf_spinup_time=3000;
int nerf_fire_time=1000;
int nerf_spindown_time=2000;



/////////////////////////////////////////////////////////
// i2c servo init
/////////////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int lastPWM[8]={0,0,0,0,0,0,0,0};
String gripper_mode="";
int gripper_servo_pin_0=0;
int gripper_servo_pin_1=1;
int nerf_servo_pin=2;
int this_servo_value=0;



void setup() {
	Serial.begin(115200);
	if (debugLevel>1) Serial.println("setup started");

	/////////////////////////////////////////////////////////
	// roboclaw setup
	/////////////////////////////////////////////////////////
	if (debugLevel>1) Serial.println("setup roboclaw");
	roboclaw.begin(38400);
	if (motorsActive){
		roboclaw.ForwardMixed(address, 0);
		roboclaw.TurnRightMixed(address, 0);
		roboclaw.ResetEncoders(address);
		Serial.println("Encoders reset to zeros");
		delay(50);
		// get initial position of encoders if recording is on
		if (record_tracks){
			record_left[record_count]=0;
			record_right[record_count]=0;
			record_next_millis=millis()+record_per_ms;
			Serial.print("recorded step ");
			Serial.println(record_count);
		}
	}



	/////////////////////////////////////////////////////////
	// FrSky CPPM setup
	/////////////////////////////////////////////////////////
	CPPM.begin(NUM_CHANNELS);
	for (i=0; i<NUM_CHANNELS; i++){
		cppm_last[i]=0;
		cppm_in[i]=0;
	}



	/////////////////////////////////////////////////////////
	// L298N setup
	/////////////////////////////////////////////////////////
	if (debugLevel>1) Serial.println("L298N setup");
	if (motorsActive){
		pinMode(l298npins[0], OUTPUT);
		pinMode(l298npins[1], OUTPUT);
		pinMode(l298npins[2], OUTPUT);
		pinMode(l298npins[3], OUTPUT);

		digitalWrite(l298npins[0], LOW);
		digitalWrite(l298npins[1], LOW);
		digitalWrite(l298npins[2], LOW);
		digitalWrite(l298npins[3], LOW);
	}



	/////////////////////////////////////////////////////////
	// servo setup, attach and set at initial position
	/////////////////////////////////////////////////////////
	if (debugLevel>1) Serial.println("servo setup");
	if (motorsActive){
		pwm.begin();
		pwm.setPWMFreq(50);  // Analog servos run at ~60 Hz updates

		// prepare nerf gun trigger location
		pwm.setPWM(nerf_servo_pin, 0, nerf_stop_speed);
		nerf_mode="ready";
		yield();
	}



	if (debugLevel>1) Serial.println("setup complete");

	flashHeadlight(2);
}

void loop() {
	cppm_loop_start=millis();
	int16_t channels[NUM_CHANNELS];
	if (CPPM.ok()) {
		// read CPPM values from receiver
		CPPM.read(channels);

		/////////////////////////////////////////////////////
		// get current values of all channels from RC receiver
		/////////////////////////////////////////////////////
		int i=0;
		for (i=0; i<NUM_CHANNELS; i++){
			// get for this channel
			cppm_this=channels[i];

			// make sure in proper range (0-255)
			if (cppm_this < 0) {
				cppm_this=0;
			} else if (cppm_this > 255) {
				cppm_this=255;
			} else if (cppm_this >=123 && cppm_this <=130) {
				cppm_this=127;
			}

			// save val to cppm_in
			cppm_in[i]=cppm_this;
		}
		// cppm_in is now populated with current values from receiver



		/////////////////////////////////////////////////////
		// drive motors
		/////////////////////////////////////////////////////
		if (cppm_in[cppm_drive_l_chan]!=cppm_last[cppm_drive_l_chan]) {
			if (cppm_in[cppm_drive_l_chan]>130){
				// driving forward
				this_motor_value=map(cppm_in[cppm_drive_l_chan], 130, 255, 0, 127);
				this_motor_direction="forward";
			} else if (cppm_in[cppm_drive_l_chan]<123) {
				// driving backward
				this_motor_value=map(cppm_in[cppm_drive_l_chan], 123, 0, 0, 127);
				this_motor_direction="backward";
			} else {
				// stop
				this_motor_value=0;
				this_motor_direction="forward";
			}
			if (this_motor_value > 126) this_motor_value=126;
			if (this_motor_direction=="forward"){
				if (motorsActive) roboclaw.ForwardM1(address, this_motor_value);
				if (debugLevel>2) Serial.println("motor left foward");
			} else {
				if (motorsActive) roboclaw.BackwardM1(address, this_motor_value);
				if (debugLevel>2) Serial.println("motor left backward");
			}
			cppm_last[cppm_drive_l_chan]=cppm_in[cppm_drive_l_chan];
		}
		if (cppm_in[cppm_drive_r_chan]!=cppm_last[cppm_drive_r_chan]) {
			if (cppm_in[cppm_drive_r_chan]>130){
				// driving forward
				this_motor_value=map(cppm_in[cppm_drive_r_chan], 130, 255, 0, 127);
				this_motor_direction="forward";
			} else if (cppm_in[cppm_drive_r_chan]<123) {
				// driving backward
				this_motor_value=map(cppm_in[cppm_drive_r_chan], 123, 0, 0, 127);
				this_motor_direction="backward";
			} else {
				// stop
				this_motor_value=0;
				this_motor_direction="forward";
			}
			if (this_motor_value > 126) this_motor_value=126;
			if (this_motor_direction=="forward"){
				if (motorsActive) roboclaw.ForwardM2(address, this_motor_value);
				if (debugLevel>2) Serial.println("motor right foward");
			} else {
				if (motorsActive) roboclaw.BackwardM2(address, this_motor_value);
				if (debugLevel>2) Serial.println("motor right backward");
			}
			cppm_last[cppm_drive_r_chan]=cppm_in[cppm_drive_r_chan];
		}



		/////////////////////////////////////////////////////
		// turn light on/off
		/////////////////////////////////////////////////////
		if (cppm_in[cppm_headlight_chan]!=cppm_last[cppm_headlight_chan]) {
			if (cppm_in[cppm_headlight_chan]<127) {
				// turn headlight off
				if (debugLevel>1) Serial.println("headlight off");
				digitalWrite(l298npins[0], LOW);
				digitalWrite(l298npins[1], LOW);
			} else {
				// turn headlight on
				if (debugLevel>1) Serial.println("headlight on");
				digitalWrite(l298npins[0], LOW);
				digitalWrite(l298npins[1], HIGH);
			}
			cppm_last[cppm_headlight_chan]=cppm_in[cppm_headlight_chan];
		}



		/////////////////////////////////////////////////////
		// open/close gripper
		/////////////////////////////////////////////////////
		if (gripper_enabled==false && cppm_in[cppm_gripper_chan]>20){
			// gripper 3-way switch will be up when initially booted, wait for it to be moved before even
			// thinking about open/close the gripper
			gripper_enabled=true;
		}
		if (gripper_enabled){
			if (cppm_in[cppm_gripper_chan]!=cppm_last[cppm_gripper_chan]) {
				if (cppm_in[cppm_gripper_chan]<20) {
					if (gripper_mode!="close"){
						// close gripper
						if (debugLevel>1) Serial.println("gripper close");
						digitalWrite(l298npins[3], LOW);
						digitalWrite(l298npins[2], HIGH);
						gripper_mode="close";
					}
				} else if (cppm_in[cppm_gripper_chan]>235) {
					if (gripper_mode!="open"){
						// open gripper
						if (debugLevel>1) Serial.println("gripper open");
						digitalWrite(l298npins[2], LOW);
						digitalWrite(l298npins[3], HIGH);
						gripper_mode="open";
					}
				} else {
					if (gripper_mode!="stop"){
						// stop gripper
						if (debugLevel>1) Serial.println("gripper stop");
						digitalWrite(l298npins[2], LOW);
						digitalWrite(l298npins[3], LOW);
						gripper_mode="stop";
					}
				}
				cppm_last[cppm_gripper_chan]=cppm_in[cppm_gripper_chan];
			}
		}



		/////////////////////////////////////////////////////
		// move gripper servos if more than +/-2
		/////////////////////////////////////////////////////
		if (cppm_in[cppm_servo_chan]>(cppm_last[cppm_servo_chan]+1) ||
				cppm_in[cppm_servo_chan]<(cppm_last[cppm_servo_chan]-1)){
			// set servos to new angle
			if (motorsActive) {
				// 277 and 448 seem to be a good min and max for the hitec HS322-HD
				this_servo_value=map(cppm_in[cppm_servo_chan], 0, 255, 277, 448);
				pwm.setPWM(gripper_servo_pin_0, 0, this_servo_value);

				// reverse value for the 2nd gripper servo
				this_servo_value=map(cppm_in[cppm_servo_chan], 255, 0, 277, 448);
				pwm.setPWM(gripper_servo_pin_1, 0, this_servo_value);
				if (debugLevel>2) {
					Serial.print("gripper servos in=");
					Serial.print(cppm_in[cppm_servo_chan]);
					Serial.print(", out= ");
					Serial.println(this_servo_value);
				}
			}
			cppm_last[cppm_servo_chan]=cppm_in[cppm_servo_chan];
		}



		/////////////////////////////////////////////////////
		// fire nerf gun
		/////////////////////////////////////////////////////
		if (cppm_in[cppm_nerf_chan]>200 && nerf_mode=="ready"){
			nerf_mode="spinning up";
			if (debugLevel>1) Serial.println("nerf fire spinning up");
			pwm.setPWM(nerf_servo_pin, 0, nerf_spinup_speed);
			cppm_last[cppm_nerf_chan]=cppm_in[cppm_nerf_chan];
			nerf_next_millis=millis()+nerf_spinup_time;
		}

		/////////////////////////////////////////////////////
		// trigger track recording
		/////////////////////////////////////////////////////
		if (cppm_in[cppm_record_chan]>200 && record_tracks==false){
			record_tracks=true;
			if (debugLevel>1) Serial.println("record tracks initiated");
			cppm_last[cppm_record_chan]=cppm_in[cppm_record_chan];
		}

		// get motor velocities
		int speed0=getSpeed(0);
		int speed1=getSpeed(1);
		int distance0=getDistance(0);
		int distance1=getDistance(1);

		/////////////////////////////////////////////////////
		// print values for use by serial devices, a raspberry pi for instance
		/////////////////////////////////////////////////////
		if (debugLevel==1){
			for (i=0; i<NUM_CHANNELS; i++){
				Serial.print("c");
				Serial.print(i);
				Serial.print("=");
				sprintf(buf, "%03d", cppm_in[i]);
				Serial.print(buf);
				Serial.print(",");
			}

			// print velocities
			Serial.print("s0=");
			sprintf(buf, "%03d", speed0);
			Serial.print(buf);
			Serial.print(",s1=");
			sprintf(buf, "%03d", speed1);
			Serial.print(buf);

			// print distances
			Serial.print(",d0=");
			sprintf(buf, "%012d", distance0);
			Serial.print(buf);
			Serial.print(",d1=");
			sprintf(buf, "%012d", distance1);
			Serial.print(buf);

			Serial.println("");
		}

		// reset motor stop
		motor_stop_after_millis=0;
	} else {
		/////////////////////////////////////////////////////
		// RC radio has disappeared perhaps
		/////////////////////////////////////////////////////
		if (debugLevel>1) Serial.println("no response from receiver");

		// stop the drive motors if this goes on for 1 second
		if (motor_stop_after_millis==0) motor_stop_after_millis=millis()+1000;

		if (millis()>motor_stop_after_millis){
			if (motorsActive) roboclaw.ForwardM1(address, 0);
			if (debugLevel>1) Serial.println("left right stop");

			if (motorsActive) roboclaw.ForwardM2(address, 0);
			if (debugLevel>1) Serial.println("motor right stop");
		}

		// stop the gripper
		if (debugLevel>1) Serial.println("gripper stop");
		digitalWrite(l298npins[2], LOW);
		digitalWrite(l298npins[3], LOW);
	}

	// perform nerf firing steps
	if (nerf_mode!="ready"){
		if (millis()>=nerf_next_millis){
			// time to go to the next step of the firing sequence, put in reverse order so else if doesn't get confused
			if (nerf_mode=="spinning down"){
				// nerf spin down complete, ready for another fire
				pwm.setPWM(nerf_servo_pin, 0, nerf_stop_speed);
				if (debugLevel>1) Serial.println("nerf fire ready");
				nerf_mode="ready";
			} else if (nerf_mode=="firing"){
				// nerf fire complete, now turn off
				pwm.setPWM(nerf_servo_pin, 0, nerf_stop_speed);
				nerf_mode="spinning down";
				if (debugLevel>1) Serial.println("nerf fire spinning down");
				nerf_next_millis=millis()+nerf_spindown_time;
			} else if (nerf_mode=="spinning up"){
				nerf_mode="firing";
				if (debugLevel>1) Serial.println("nerf fire firing");
				// nerf spinup complete, now fire
				pwm.setPWM(nerf_servo_pin, 0, nerf_fire_speed);
				nerf_next_millis=millis()+nerf_fire_time;
			}
		}
	}

	if (record_tracks){
		if (millis() >= record_next_millis){
			// time to make note of the encoder counts again
			record_count+=1;
			if (record_count>=record_max_tracks){
				// turn off recording
				record_tracks=false;
				Serial.println("Recording halted, max seconds reached.");
				roboclaw.ResetEncoders(address);
				Serial.println("Encoders reset to zeros");
				Serial.println("");
				Serial.println("step          left         right");
				// print out recorded tracks
				for (i=0; i<record_max_tracks; i++){
					sprintf(buf, "%04d", i);
					Serial.print(buf);
					Serial.print("  ");
					sprintf(buf, "%012d", record_left[i]);
					Serial.print(buf);
					Serial.print("  ");
					sprintf(buf, "%012d", record_right[i]);
					Serial.println(buf);
				}

				// stop motors and pause a second
				roboclaw.ForwardM1(address, 0);
				roboclaw.ForwardM2(address, 0);
				flashHeadlight(2);
				delay(1000);

				// reset encoders
				roboclaw.ResetEncoders(address);
				Serial.println("Encoders reset to zeros");

				// playback recorded steps
				Serial.println("Playing back tracks");
				for (i=0; i<record_max_tracks; i++){
					// get current encoder positions
					record_current_left=getDistance(0);
					record_current_right=getDistance(1);

					flashHeadlight(1);

					Serial.println("");
					Serial.println("step       curleft      curright          left         right  ldir  rdir");
					// print out starting location
					sprintf(buf, "%04d", i);
					Serial.print(buf);
					Serial.print("  ");
					sprintf(buf, "%012d", record_current_left);
					Serial.print(buf);
					Serial.print("  ");
					sprintf(buf, "%012d", record_current_right);
					Serial.print(buf);
					Serial.print("  ");
					sprintf(buf, "%012d", record_left[i]);
					Serial.print(buf);
					Serial.print("  ");
					sprintf(buf, "%012d", record_right[i]);
					Serial.println(buf);

					record_misc=0;
					if (record_current_left==record_left[i]) record_left_direction="";
					if (record_current_left<record_left[i]) record_left_direction="f";
					if (record_current_left>record_left[i]) record_left_direction="b";

					if (record_current_right==record_right[i]) record_right_direction="";
					if (record_current_right<record_right[i]) record_right_direction="f";
					if (record_current_right>record_right[i]) record_right_direction="b";

					while (record_left_direction!="" || record_right_direction!=""){
						record_current_left=getDistance(0);
						record_current_right=getDistance(1);

						// print out expected location
						sprintf(buf, "%04d", i);
						Serial.print(buf);
						Serial.print("  ");
						sprintf(buf, "%012d", record_current_left);
						Serial.print(buf);
						Serial.print("  ");
						sprintf(buf, "%012d", record_current_right);
						Serial.print(buf);
						Serial.print("  ");
						sprintf(buf, "%012d", record_left[i]);
						Serial.print(buf);
						Serial.print("  ");
						sprintf(buf, "%012d", record_right[i]);
						Serial.print(buf);
						Serial.print("  ");
						Serial.print(record_left_direction);
						Serial.print("     ");
						Serial.println(record_right_direction);

						record_misc+=1;
						if (record_left_direction=="f"){
							if (record_current_left<record_left[i]){
								roboclaw.ForwardM1(address, record_playback_speed);
							} else {
								// must have reached position
								roboclaw.ForwardM1(address, 0);
								record_left_direction="";
							}
						}
						if (record_left_direction=="b"){
							if (record_current_left>record_left[i]){
								roboclaw.BackwardM1(address, record_playback_speed);
							} else {
								// must have reached position
								roboclaw.ForwardM1(address, 0);
								record_left_direction="";
							}
						}

						if (record_right_direction=="f"){
							if (record_current_right<record_right[i]){
								roboclaw.ForwardM2(address, record_playback_speed);
							} else {
								// must have reached position
								roboclaw.ForwardM2(address, 0);
								record_right_direction="";
							}
						}
						if (record_right_direction=="b"){
							if (record_current_right>record_right[i]){
								roboclaw.BackwardM2(address, record_playback_speed);
							} else {
								// must have reached position
								roboclaw.ForwardM2(address, 0);
								record_right_direction="";
							}
						}

						if (record_misc>5000){
							// aw just quit
							Serial.println("quitting, this is crazy 1");
							break;
						}
						// pause a bit
						//delay(500);
					}

					if (record_misc>5000){
						// aw just quit
						Serial.println("quitting, this is crazy 2");
					}
				}

				record_tracks=false;
				flashHeadlight(3);
			} else {
				flashHeadlight(1);
				record_left[record_count]=getDistance(0);
				record_right[record_count]=getDistance(1);
				record_next_millis=millis()+record_per_ms;
				Serial.print("recorded step ");
				Serial.println(record_count);
			}
		}
	}

	// the proper timing of the CPPM reads needs around 50ms between reads
	cppm_loop_start=millis()-cppm_loop_start;
	if (cppm_loop_start<50) {
		delay(50-cppm_loop_start);
	} else {
		if (debugLevel>1) Serial.println("no delay");
	}
}

int getDistance (int encoderNumber) {
	// get most recent encoder count value from roboclaw
  uint8_t status;
  bool valid;
	int32_t distance=0;
 
	if (encoderNumber==0){
  	distance = -1*roboclaw.ReadEncM1(address, &status, &valid);
	} else if (encoderNumber==1){
  	distance = roboclaw.ReadEncM2(address, &status, &valid);
	} else {
		distance=0;
	}
	//distance=distance/100;

	if (valid) {
		return distance;
	} else {
		return 0;
	}
}
int getSpeed(int encoderNumber) {
	// get most recent speed value from roboclaw via ReadISpeedMx
  uint8_t status;
  bool valid;
	int32_t speed=0;
 
	if (encoderNumber==0){
  	speed = -1*roboclaw.ReadISpeedM1(address, &status, &valid);
	} else if (encoderNumber==1){
  	speed = roboclaw.ReadISpeedM2(address, &status, &valid);
	} else {
		speed=0;
	}
	speed=speed/100;

	lastSpeed[encoderNumber]=speed;

	if (valid) {
		return speed;
	} else {
		return 0;
	}
}

void flashHeadlight(int times){
	if (times > 0){
		for (int j=0; j<times; j++){
			// turn headlight on
			if (debugLevel>1) Serial.println("headlight on");
			digitalWrite(l298npins[0], LOW);
			digitalWrite(l298npins[1], HIGH);

			delay(200);

			// turn headlight off
			if (debugLevel>1) Serial.println("headlight off");
			digitalWrite(l298npins[0], LOW);
			digitalWrite(l298npins[1], LOW);

			delay(200);
		}
	}
}
