/*
 Name:		FlightController.ino
 Created:	09/16/2018 11:24:47
 Author:	Arnór
*/



#include <SPI.h>
#include <RH_RF95.h>


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

#define RFM95_CS		5
#define RFM95_RST		6
#define ERRORLED		9
#define RFM95_INT		10

#define THROTTLE		14
#define KILL_ENGINE		15
#define SELF_DESTRUCT	16

//Mission Control caller ID
const uint8_t MC_ID = 0x80;

//Flight Computer caller ID
const uint8_t FC_ID = 0x81;

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


//Data
uint8_t DATA_BATTERY = 0x05;

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

void setMessageType(uint8_t temp)
{
	switch (temp)
	{
	case 0x01:
		CMD_TYPE = CMD_SET_STATE;
	case 0x02:
		CMD_TYPE = CMD_SET_THROTTLE;
	default:
		break;
	}
}

void sendBack()
{
	//check is the XOR of TYPE and VALUE
	CMD_CHECK = DATA_BATTERY ^ CMD_VALUE;
	COMMAND[0] = CMD_TYPE;
	COMMAND[1] = DATA_BATTERY;
	COMMAND[2] = CMD_CHECK;
	digitalWrite(ERRORLED, LOW);
	rf95.setHeaderId(FC_ID);
	Serial.println("Transmitting...");
	rf95.send((uint8_t *)COMMAND, COMMAND_size);
	delay(10);
	rf95.waitPacketSent();
	Serial.println("Command Sent");
	recieveDATA();
}

void recieveDATA()
{
	uint8_t buf[COMMAND_size];
	uint8_t len = sizeof(buf);
	
	Serial.println("Waiting for reply...");
	//MAX wait for DATA is set 1 sec.
	if (rf95.waitAvailableTimeout(1000))
	{
		if (rf95.recv(buf, &len))
		{
			if (rf95.headerId() == MC_ID) 
			{
				if (buf[2] == (buf[0] ^ buf[1]))
				{
					DATA_TYPE = buf[0];
					DATA_VALUE = buf[1];
					DATA_ERROR = buf[2];

					switch (DATA_TYPE)
					{
					case 0x01:
						Serial.print("Set Stage: ");
						Serial.println(buf[1]);
						setMessageType(DATA_TYPE);
						sendBack();
						break;
					case 0x02:
						Serial.println("Set Throttle");
						Serial.println(buf[1]);
						setMessageType(DATA_TYPE);
						sendBack();
						break;
					}
				}
				else
				{
					Serial.println("Recieve FAIL");
					digitalWrite(ERRORLED, HIGH);

				}
			}
			else { Serial.println("Not my message."); }
			//Serial.println((char*)buf);
			//Serial.print("RSSI: ");
			//Serial.println(rf95.lastRssi(), DEC);
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


void loop()
{



	recieveDATA();
	delay(900);



	//ég sendi stærð sem er *strengur 
	//og byrjar á radiopacket í minninu, og ég ætla að senda 20 sæti.

	//Þetta er það sem ég ætla að senda:
	//ég ætla að hafa 4 byte, fyrsta er ID, annað er message type, þriðja
	//er value. Fyrstu þrjú byte-in eru XOR-uð saman og það sett í fjórða byte-ið.
	//að XOR-a saman er ^ merkið. þegar pakkinn er móttekinn eru 3 fyrstu bætin
	//aftur XOR-uð saman og það borið saman við fjórða byte-ið úr sendingunni.
	//Ef scramble == 4th byte recieved -> allt er í lagi. Halda áfram.



}