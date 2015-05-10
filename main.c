/*
 ===============================================================================
Name: Christopher Curti
 Breatheband firmware
 May 9, 2015
 libraries are from ucxpresso.new LCPXpress
 used ble_heartrate as basis, see getting started guide for ucxpresso
 https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&ved=0CB8QFjAA&url=http%3A%2F%2Fucxpresso.googlecode.com%2Fsvn%2Ftrunk%2Fnano11uxx%2FuCXpresso.BLE%2Fdoc%2Fgetting_started_with_ucxpresso.ble.pdf&ei=swFPVYX-N8HfoAS07ICQCQ&usg=AFQjCNFUvixFaB1OP4D3bjVYUMLnpY13Ew&bvm=bv.92885102,d.cGU
 ===============================================================================
 */

#include "uCXpresso.h"
#include "class/serial.h"
#include "class/usb_cdc.h"

#ifdef DEBUG
#include "debug.h"
#define DBG		dbg_printf
#else
#define DBG(...)
#endif

//
// TODO: insert other include files here
//
#include "class/bus.h"
#include "class/ble_serial.h"
#include "class/ble_heartrate.h"
#include "class/ble_battery.h"
#include "class/ble_ht.h"
#include "class/adc.h"
#include "class/timer.h"
#include <math.h>
#define HRV_PERCENTAGE 0.20	   //use to determine stress alert sensitivity for hrv
#define GSR_PERCENTAGE 0.50    //use to determine stress alert sensitivity for gsr
#define IBI_SIZE 200

int main(void) {
#ifdef DEBUG
	#if __USE_USB
		usbCDC ser;
		ser.connect();
	#else
		CSerial ser;
		ser.settings(115200);
	#endif
	CDebug dbg(ser);
	dbg.start();
#endif


	/*************************************************************************
	 *
	 *                         your setup code here
	 *
	**************************************************************************/
	//
	// BLE engine (serial stream)
	//
	bleSerial	ble("BreatheBand");
	ble.enable(96);

	//
	// Heart Beat Rate Service
	//
	bleHeartRate hrm(ble);
	hrm.supportContact(true);
	hrm.contactStatus(true);
	hrm.setSensorLocation(HRSL_WRIST);

	//
	// Battery Level & Health Thermometer Service (Use internal sensors)
	//
	bleBatteryLevel		 bat(ble);
	bleHealthThermometer ht(ble);

	//
	// Sensors
	//
	CAdc gsrSensor(AD0);	//gsr sensor ADC
	gsrSensor.begin();		//enable reading adc

	CAdc ppgSensor(AD1);    //ppg sensor ADC
	ppgSensor.enable();	    //enable reading adc

	CAdc accel_X(AD2);		//accelerometer X
	accel_X.begin();

	CAdc accel_Y(AD3);		//accelerometer Y ADC
	accel_Y.begin();

	CAdc accel_Z(AD4);		//acclerometer Z ADC
	accel_Z.begin();


	//Tactile Switches

	CPin ButtonOne(P17);   //Map B1 to pin 17
	CPin ButtonTwo(P18);   //Map B2 to pin 18
	ButtonOne.input();
	ButtonTwo.input();


	// RGB LED Pins
	CPin ledRed(P21);		//map to pin 21
	ledRed.output();		//set as output

	CPin ledGreen(P23);		//map to pin 23
	ledGreen.output();		//set as output

	CPin ledBlue(P22);      //map to pin 22
	ledBlue.output();       //set as output


	//Vibratory Motor

	CPin Vibrate(P26);   	//map to pin 26
	Vibrate.output();
	Vibrate.write(LOW);	    //vibrate off

	//
	//Bank for recording alert times
	int test[10];

	for(int i = 0 ; i<=9 ; i++){   //initialize array to zero
		test[i] = 0;
		}
	int storeCount = 0;            //counter for number of alerts stored , MAX is 10

	//

	//Boolean Flags
	bool alert = false ;   			//triggers alert to USER if true
	bool baseLineMode = true;    	//triggers baseline acquisition, in beginning need to gather baseline value so enable
	bool storeAlert = false;		//does the value for current alert need to be stored
	bool vibrateOn = false;			//is vibratory alert enabled
	bool needToSend = false;		//if values in alert array have not been sent, then true
	bool firstBeat = true;			//keep track which beat we are on
	bool secondBeat = false;		//keep track which beat we are on
	bool Pulse = false;
	bool hrvUpdated = false;


	//Other Variables
	float totalTimeSince = 0.0;     //keeps track of time
	int sampleCounter = 0;
	int lastBeatTime = 0;
	int N = 0;
	int T = 512; //trough
	int thresh = 512; //threshold
	int amp = 100;   //amplitude
	int IBI = 600;   //interval between beats
	int P = 512;
	int ppg;
	int gsr;
	int X_acc;
	int Y_acc;
	int Z_acc;
	int ibiCount = 0; //keep track of ibi values
	int rate[IBI_SIZE];
	int hrvBaseline = 0;
	int hrv = 0;
	int gsrBaseline = 512;
	int halfVoltage = 256;
	int totalAcceleration = 0;
	int maxAcceleration = 100000;
	//clock and battery values
	CTimeout t1, t2, t3;
	//float value;
	//uint8_t level;

	CTimer sampleTime = CTimer(TIMER1);
	sampleTime.second(0.002);
	sampleTime.enable();

	while(1) {

		sampleTime.wait();
		totalTimeSince += 0.002;   //update clock

		X_acc = accel_X.read();		//grab acceleration values
		Y_acc = accel_Y.read();
		Z_acc = accel_Z.read();
		gsr = gsrSensor.read();		//grab gsr value
		ppg = ppgSensor.read();		//grab ppg value

		X_acc -=halfVoltage;  //find change from 0 acc
				Y_acc -=halfVoltage;
				Z_acc -=halfVoltage;
				X_acc = (X_acc) * (X_acc); //square difference
				Y_acc = (Y_acc) * (Y_acc);
				Z_acc = (Z_acc) * (Z_acc);
				totalAcceleration = X_acc + Y_acc + Z_acc;


				//
		//
		// Modes and colors
		//

		if (baseLineMode){            //if in baseline mode also this is the start
					ledBlue.write(LOW);
					ledRed.write(HIGH);		//Light is whitishBlue
					ledGreen.write(LOW);
				}

		if(!alert && vibrateOn && !baseLineMode){  // if not in baselinemode and not alert
					ledBlue.write(LOW);
					ledRed.write(HIGH);		//Light is blue
					ledGreen.write(HIGH);
				}

		if(!alert && !vibrateOn && !baseLineMode){  //no vibrate mode
							ledBlue.write(HIGH);
							ledRed.write(HIGH);		//Light is green
							ledGreen.write(LOW);
						}

		if ((alert) && !baseLineMode){						// if alert triggered and not in baseline mode
					if(storeAlert){							// if current alert needs to be stored
						if(storeCount <= 9){				// if array is not full

							test[storeCount] = (int)(totalTimeSince / 60);		//store value as relative minute since device started
							storeCount ++;										//update number of elements in array
						}
						else{
							for (int i = 1; i <= 9 ; i++){						//if alert array is full
								test[i-1] = test[i];							//drop first element and shif all other down
							}
							test[9] = (int)(totalTimeSince/60);					//store value as relative minute since device started

						}
						storeAlert = false;										//reset flag to show value is stored
					}
					ledBlue.write(LOW);
					ledRed.write(LOW);		//Light is red
					ledGreen.write(HIGH);
					if (vibrateOn){			//if vibration enables
						Vibrate.write(HIGH); //start vibration
							}
		}

		if ((ButtonTwo) == LOW){
					//dbg.println("Button Two Pressed");
					Vibrate.write(LOW);
					alert = true;
					storeAlert = true;
					sleep(1000);
					totalTimeSince += 1;
					if (ButtonTwo == LOW && !baseLineMode){  //baseline mode selected
						//dbg.println("start baseline selected");
						ibiCount = 0;           //start new baseline selection
						baseLineMode = true;
						}
					else if(ButtonTwo == LOW && baseLineMode){
						//dbg.println("BaseLineMode Canceled");
						baseLineMode = false;
					}



				}
		while (ButtonOne == LOW){
					dbg.println("ButtonOne Pressed");
					alert = false;
					Vibrate.write(LOW);
					sleep(1000);
					totalTimeSince += 1;

					if (ButtonOne == LOW){
						Vibrate.write(HIGH);
						sleep(200);
						totalTimeSince += .1;
						Vibrate.write(LOW);
						//dbg.println("for two seconds");
						if (vibrateOn){
							ledBlue.write(HIGH);
							ledRed.write(HIGH);		//Light is Green
							ledGreen.write(LOW);
							vibrateOn = false;


							}
						else{
							vibrateOn = true;
							}
						}

					}

		if (ble.isConnected()){                 //if device connected
					for (int i = 0; i<=storeCount ; i++){         //send alert array

						if (test[i] != 0) {

						test[i] = ((int)totalTimeSince/60) - test[i];   //store time in minutes since alert
						}
						sleep(100);
						hrm.sendMeasure((uint16_t)test[i]);    //send time in minutes since last alert
					}
					needToSend = false;					//values are sent can remove array values after connection
				}

		if (!ble.isConnected()){			//if device
			if(!needToSend){				//can get rid of array values

				for (int i = 0; i<=9 ; i++){
					test[i] = 0;
					}
				needToSend = true;    //alert values have been cleared and will need to be sent
				totalTimeSince = 0.0;
				storeCount = 0;
					}
				}


		//HRV Measurement
		sampleCounter += 2;
		N = sampleCounter - lastBeatTime;

		if (ppg < thresh && N > (IBI/5) * 3) { //avoid dicrotic noise
			if (ppg <T) {
				T= gsr;
			}
		}
		if (ppg > thresh && ppg > P) { // thresh condition helps avoid noise
					P = ppg;                             // P is the peak
					}  				// keep track of highest point in pulse wave
		if (N > 250) {                         // avoid high frequency noise
					if ((ppg > thresh) && (Pulse == false) && (N > (IBI / 5) * 3)) {
									Pulse = true; // set the Pulse flag when we think there is a pulse

									IBI = sampleCounter - lastBeatTime; // measure time between beats in mS
									lastBeatTime = sampleCounter; // keep track of time for next pulse

									if (secondBeat) { // if this is the second beat, if secondBeat == TRUE
										secondBeat = false;             // clear secondBeat flag

											rate[ibiCount] = IBI; //get interval between beats
											ibiCount ++;

									}
									if (firstBeat) { // if it's the first time we found a beat, if firstBeat == TRUE
											firstBeat = false;               // clear firstBeat flag
											secondBeat = true;           // set the second beat flag
											continue;         // IBI value is unreliable so discard it
									}
								}
				}
		if (ppg < thresh && Pulse == true) { //when values are going down, beat is over
					Pulse = false;
					amp = P - T;
					thresh = (amp / 2) +T;   //set threshold at 50 % of the Amplitude
					P = thresh;
					T= thresh;
				}
		if(N > 2500){ //if 2.5 seconds pass without beat then reset
					thresh = 512;
					P = 512;
					T = 512;
					lastBeatTime = sampleCounter;
					firstBeat = true;
					secondBeat = false;
				}
		if (ibiCount == IBI_SIZE-1){ // if we collected 300 ibi values
			sampleTime.disable();
			float mean = 0.0;
			float deviation = 0.0;
			for (int i = 0; i<IBI_SIZE ; i++){
				mean += rate[i];
			}
			mean = mean/IBI_SIZE;

			for (int i = 0; i<IBI_SIZE ; i++){
				deviation += (rate[i] - mean)*(rate[i]-mean);
			}

			if (baseLineMode){
				hrvBaseline = deviation;   //run out of room with more complicagted stdev, for comparison sake this is alright
				baseLineMode = false;
				gsrBaseline = gsr;
			}
			else{
				hrv = deviation;
				hrvUpdated = true;
			}
			sampleTime.enable();
		}
		if (!baseLineMode){ //if not in baseline mode then check values for hrv and gsr

			if (hrvUpdated){ //if hrv recently updated
						if (hrvBaseline < hrv){ //then new baseline
							hrvBaseline = hrv;

						}

						hrvUpdated = false;

					if(gsrBaseline < gsr){ //then new baseline
						gsrBaseline = gsr;
					}

					if ((gsr < gsrBaseline*GSR_PERCENTAGE)) {
						alert = true;   //alert user
						dbg.println("GSRALERT");
					}

					if ((hrv < hrvBaseline*HRV_PERCENTAGE)) {
						alert = true;
					}


				}
		}





	}
    return 0 ;
}

//
// default memory pool
//
static uint8_t mem_pool[DEFAULT_POOL_SIZE];

//
// setup before the system startup
//
extern "C" void sys_setup(void) {
	pool_memadd((uint32_t)mem_pool, sizeof(mem_pool));
#if __USE_USB==0
	pool_memadd(USB_MEM_BASE, USB_MEM_SIZE);
#endif
}
