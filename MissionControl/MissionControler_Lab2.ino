﻿//Mission Controller


#include <SPI.h>
#include <RH_RF95.h>

#include <LiquidCrystal.h>
	     	   //RS EN BD4 BD5 BD6 BD7
LiquidCrystal lcd(13, 12, 11, 10, 9, 6);



// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 430.0

#define RFM95_CS		5
#define RFM95_RST		6
#define ERRORLED		9
#define RFM95_INT		10

#define THROTTLE		14
#define KILL_ENGINE		15
#define SELF_DESTRUCT	16

//Mission Control caller ID
const uint8_t MC_ID = 9;

//Flight Computer caller ID
const uint8_t FC_ID = 8;

//two types of commands: set state & set throttle
uint8_t CMD_SET_STATE = 0x01; // B 0000 0001;
uint8_t CMD_SET_THROTTLE = 0x02; // B 0000 0010;

//the command I will be sending: nr.1-command type, nr.2-value and nr.3-error check.
const int COMMAND_size = 3;
uint8_t COMMAND[COMMAND_size];
uint8_t CMD_TYPE;
uint8_t CMD_VALUE;
uint8_t CMD_CHECK;

//the data I will be recieving from the Flight Computer
uint8_t DATA_TYPE;
uint8_t DATA_VALUE;
uint8_t DATA_ERROR;


//Values
uint8_t BATTERY;


bool KILL_SWITCH = false;
bool FAIL_SAFE = false;

enum STATE_NAMES { START, TEST, READY, IGNITION, ASCENT, COASTING, APOGEE, DESCENT, LAND, DISABLE, RESET, SELFDESTRUCT };

//Things to print on lcd:
char currentState[8];

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// packet counter, we increment per transmission
int16_t packetnum = 0;



void setup()
{
	pinMode(13, OUTPUT);
	pinMode(12, OUTPUT);
	pinMode(11, OUTPUT);
	pinMode(10, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(6, OUTPUT);
	
	lcd.begin(16, 2);
	lcd.print("hello, world!");



	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);
	pinMode(ERRORLED, OUTPUT);
	digitalWrite(ERRORLED, LOW);
	pinMode(THROTTLE, INPUT);
	pinMode(KILL_ENGINE, INPUT_PULLUP);
	pinMode(SELF_DESTRUCT, INPUT_PULLUP);


	//Serial.begin(115200);
	//while (!Serial) {
	//  delay(1);
	//}

	delay(100);

	Serial.println("Initializing");

	// manual reset
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);

	while (!rf95.init()) {
		Serial.println("Connection failed!");
		while (1);
	}
	Serial.println("Connection achieved!");

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95.setFrequency(RF95_FREQ)) {
		Serial.println("setFrequency failed");
		while (1);
	}
	Serial.print("Frequency set to: "); Serial.println(RF95_FREQ);

	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
	// you can set transmitter powers from 5 to 23 dBm:
	rf95.setTxPower(23, false);



	testRun();

}

uint32_t readThrottle()
{
	//The entire voltage range is read with 32 bits and converted to 8 bits (1 byte) for transmission..
	float SCALE = 0.25;
	uint32_t temp_VALUE = analogRead(THROTTLE);
	return SCALE * temp_VALUE;
	
}

void setState(uint8_t state)
{
	switch (state)
	{
	case 0:
		CMD_VALUE = START;
		break;
	case 1:
		CMD_VALUE = TEST;
		break;
	case 2:
		CMD_VALUE = READY;
		break;
	case 3:
		CMD_VALUE = IGNITION;
		break;
	case 4:
		CMD_VALUE = ASCENT;
		break;
	case 5:
		CMD_VALUE = COASTING;
		break;
	case 6:
		CMD_VALUE = APOGEE;
		break;
	case 7:
		CMD_VALUE = DESCENT;
		break;
	case 8:
		CMD_VALUE = LAND;
		break;
	case 9:
		CMD_VALUE = DISABLE;
		break;
	case 10:
		CMD_VALUE = RESET;
		break;
	case 11:
		CMD_VALUE = SELFDESTRUCT;
		break;
	default:
		break;
	}
}


void sendCOMMAND(uint8_t CMD_TYPE, uint8_t CMD_VALUE)
{
	//check is the XOR of TYPE and VALUE
	CMD_CHECK = CMD_TYPE ^ CMD_VALUE;
	COMMAND[0] = CMD_TYPE;
	COMMAND[1] = CMD_VALUE;
	COMMAND[2] = CMD_CHECK;
	checkButtons();
	rf95.setHeaderId(MC_ID);
	//Serial.print("Transmitting on: ");
	//Serial.println(MC_ID);
	rf95.send((uint8_t *)COMMAND, COMMAND_size);
	delay(10);
	rf95.waitPacketSent();
	Serial.print("Command Sent: ");
	Serial.print(CMD_TYPE, HEX);
	Serial.print(" Data: ");
	Serial.println(CMD_VALUE, DEC);

	recieveDATA();
}

void recieveDATA()
{
	uint8_t buf[COMMAND_size];
	uint8_t len = sizeof(buf);
	checkButtons();
	//Serial.println("Waiting for reply...");
	//MAX wait for DATA is set 1 sec.
	if (rf95.waitAvailableTimeout(1000))
	{
		if (rf95.recv(buf, &len))
		{
			if (rf95.headerId() == FC_ID) {
				if (buf[2] == (buf[0] ^ buf[1]))
				{
					DATA_TYPE = buf[0];
					DATA_VALUE = buf[1];
					DATA_ERROR = buf[2];

					switch (DATA_TYPE)
					{
					case 0x01:
						Serial.print("#ACK COMMAND: ");
						Serial.println(buf[1]);
						digitalWrite(ERRORLED, LOW);
						break;
						
					case 0x02:
						Serial.print("#BATT VOLTAGE: ");
						BATTERY = buf[1];
						Serial.println(buf[1]);
						digitalWrite(ERRORLED, LOW);
						break;
					case 0x03:
						Serial.println("#Command FAIL");
						digitalWrite(ERRORLED, HIGH);
						break;
					default:
						Serial.println("#FAILED");
						digitalWrite(ERRORLED, HIGH);
						break;
					}
				}
				else
				{
					Serial.println("CheackSum ERROR");
					digitalWrite(ERRORLED, HIGH);

				}
				//Serial.println((char*)buf);
				//Serial.print("RSSI: ");
				//Serial.println(rf95.lastRssi(), DEC);
			}
			else {
				Serial.println("Not my message, ID:");
				Serial.println(rf95.headerId());
			}

		}
		else
		{
			Serial.println("Receive failed");
			digitalWrite(ERRORLED, HIGH);							//DATA_TYPE: 1-data, 2-ack, 3-error
			
		}
	}
	else
	{
		Serial.println("ACK not received");
		digitalWrite(ERRORLED, HIGH);
	}

}

void checkButtons()
{
	if (!digitalRead(KILL_ENGINE))
	{
		KILL_SWITCH = true;
	}
	if (!digitalRead(SELF_DESTRUCT))
	{
		FAIL_SAFE = true;
	}
}


void testRun() {

	Serial.println("Starting signal test... ");
	while (digitalRead(ERRORLED) == LOW) {
		Serial.print("Trying... ");
		sendCOMMAND(CMD_SET_STATE, 1);
		delay(1000);

	}
	Serial.println();
	Serial.print("Done, Signal:");
	Serial.println(rf95.lastRssi());

	Serial.println("Starting fuel pump test... ");
	while (digitalRead(ERRORLED) == LOW) {
		Serial.print("Set state... ");
		sendCOMMAND(CMD_SET_STATE, 7);
		delay(1000);

	}
	delay(1000);
	while (digitalRead(ERRORLED) == LOW) {
		Serial.print("Reseting... ");
		sendCOMMAND(CMD_SET_STATE,1);
		delay(1000);

	}
	Serial.println();
	Serial.print("Done.");

	Serial.println("Starting parachute test... ");
	while (digitalRead(ERRORLED) == LOW) {
		Serial.print("Set state... ");
		sendCOMMAND(CMD_SET_STATE, 8);
		delay(1000);

	}
	delay(1000);
	while (digitalRead(ERRORLED) == LOW) {
		Serial.print("Reseting... ");
		sendCOMMAND(CMD_SET_STATE, 1);
		delay(1000);

	}
	Serial.println();
	Serial.print("Done.");

	Serial.println("All test done.");
	delay(2000);


}

void loop()
{
/*
	checkButtons();
	if (KILL_SWITCH)
	{

		setCommandType(2);
		setState(9);
		sendCOMMAND();
		delay(1500);
		KILL_SWITCH = false;
	}
	if (FAIL_SAFE)
	{
	
		setCommandType(1);
		setState(11);
		sendCOMMAND();
		delay(1500);
		FAIL_SAFE = false;
	}
	*/
	//if time passed (skoða excel check listann fyrir states) skipta um state og eitthvað svoleiðis...
	//annars alltaf senda throttle stillingar.
	lcd.clear();
	//setCommandType(1);
	readThrottle();
	sendCOMMAND(CMD_SET_STATE,1);
	delay(100);

	sendCOMMAND(CMD_SET_THROTTLE,readThrottle());
	readThrottle();
	delay(100);

	lcd.print("hello");
	Serial.println("____________________");

	//ég sendi stærð sem er *strengur 
	//og byrjar á radiopacket í minninu, og ég ætla að senda 20 sæti.

	//Þetta er það sem ég ætla að senda:
	//ég ætla að hafa 4 byte, fyrsta er ID, annað er message type, þriðja
	//er value. Fyrstu þrjú byte-in eru XOR-uð saman og það sett í fjórða byte-ið.
	//að XOR-a saman er ^ merkið. þegar pakkinn er móttekinn eru 3 fyrstu bætin
	//aftur XOR-uð saman og það borið saman við fjórða byte-ið úr sendingunni.
	//Ef scramble == 4th byte recieved -> allt er í lagi. Halda áfram.



}