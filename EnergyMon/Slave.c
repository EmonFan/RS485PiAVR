
/* ----------------------- AVR includes -------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/crc16.h>

/* ----------------------- 1 Wire interface includes ------------------------*/
#include "1wire/polled/OWIPolled.h"
#include "1wire/polled/OWIHighLevelFunctions.h"
#include "1wire/polled/OWIBitFunctions.h"
#include "1wire/common_files/OWIcrc.h"

/* ----------------------- Interrupt UART includes --------------------------*/
#include "./uart/uart.h"

/* ----------------------- Other includes -----------------------------------*/
#include "./robin/robin.h"

/* ----------------------- Defines ------------------------------------------*/
#define USART_BAUDRATE  115200
#define BAUD_PRESCALE   (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

/* ----------------------- PIN Defines --------------------------------------*/
#define RS485_REPIN     PORTB0       //Receive enable pin LOW to receive
#define AO_PIN          PORTB1       //Address A0
#define A1_PIN          PORTB2       //Address A1
#define A2_PIN          PORTB3       //Address A2
#define CT3_PIN         PORTB4       //Is the CT in use?
#define CT4_PIN         PORTB5       //Is the CT in use?
#define OWIB            PORTB6       //In use by the 1 wire bus
#define CT6_PIN         PORTB7       //Is the CT in use?

#define CT6_SIZE        PORTC0       //Is the CT a low or high range device
#define CT3_SIZE        PORTC1       //Is the CT a low or high range device

#define CT5_PIN         PORTD3       //Is the CT in use?
#define CT1_PIN         PORTD6       //Is the CT in use?
#define CT2_PIN         PORTD5       //Is the CT in use?

#define ADDRESS_BASE        0xA0  //Base address, the address pins will be added to this value
#define DEVICE_TYPE_TEMP    0xC0  //The device is a temperature sensor
#define DEVICE_TYPE_POWER   0xC1  //The device is a power sensor

/* ----------------------- Slave Commands -----------------------------------*/
#define ENUMERATE_COUNT   0xD0
#define ENUMERATE_ITEM    0xD1
#define TRANSMIT_ITEM     0xD2

/* ----------------------- 1 Wire bus defines -------------------------------*/
// 1-wire stuff. This is copied from the example in ../lib/1wire/polled.
#define DS1820_FAMILY_ID			  0x10
#define DS18B20_FAMILY_ID			  0x28
#define DS1820_START_CONVERSION	0x44
#define DS1820_READ_SCRATCHPAD	0xbe
#define DS1820_ERROR				    -1000   // Return code. Outside temperature range.

#define SEARCH_SUCCESSFUL		    0x00
#define SEARCH_CRC_ERROR			  0x01

#define FALSE	                  0
#define TRUE		                1

#define MAX_DEVICES	            32          //!< Max number of devices to search for.
#define BUSES		                (OWI_PIN_5 | OWI_PIN_6 |OWI_PIN_7) //!< Buses to search. (PB5, PB6, PB7)

/* ----------------------- DS1820 Wiring ------------------------------------*/
/*
  ORANGE      -> +5V
  BLUE        -> SIGNAL
  BLUE/WHITE  -> GND

  BROWN       -> +5V
  GREEN       -> SIGNAL
  GREEN/WHITE -> GND
*/
/* ----------------------- Global Debugging ---------------------------------*/
#define DEBUG
/* ----------------------- Global Debugging ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/
static uint8_t    pkt_buf_Rcv[PKT_MAX_PLEN+2];
static uint8_t    pkt_bufSend[PKT_MAX_PLEN+2];
static uint8_t    dataByte = 0;
static uint8_t    packet_sequence = 0;
static uint8_t    devicesFound = 0;
//static uint16_t   ADCValue;
static uint8_t    timerIrqCount = 0;

/* ----------------------- Function prototypes ------------------------------*/
uint8_t processPacket(unsigned int c);
uint8_t pkt_compute_cksum(uint8_t * p);
void processCommand(void);
void sendResponse(void);
void ADCsetup(uint8_t channel);

/* ----------------------- RS485 Macros -------------------------------------*/
#define TRANSMIT PORTB |= _BV(RS485_REPIN); //Bit HIGH
#define RECEIVE PORTB &= ~_BV(RS485_REPIN); //Bit LOW

/* ----------------------- 1 Wire interface ---------------------------------*/
/**
 *  The OWI_device data type holds information about what bus each device
 *  is connected to, and its 64 bit identifier.
 */
typedef struct OWI_device
{
	unsigned char bus;      //!< A bitmask of the bus the device is connected to.
	unsigned char id[8];    //!< The 64 bit identifier.
	double temperature;     //!< The measured temperature
  uint8_t deviceType;
} OWI_device;

typedef struct SlaveConfig
{
  unsigned char address;
  uint8_t totalCTs;
  uint8_t CT1 :1; //Bit storage
  uint8_t CT2 :1;
  uint8_t CT3 :1;
  uint8_t CT4 :1;
  uint8_t CT5 :1;
  uint8_t CT6 :1;
  uint8_t CT3_SMALL :1;
  uint8_t CT6_SMALL :1;
} SlaveConfig;

unsigned char SearchBuses(OWI_device * devices, unsigned char len, unsigned char buses);
signed int DS1820_ReadTemperatureNow(unsigned char bus, unsigned char * id);
signed int DS1820_StartConversion(unsigned char bus, unsigned char * id);
signed int DS1820_ReadTemperature(unsigned char bus, unsigned char * id);
void OWI_Inventory(void);

OWI_device devices[MAX_DEVICES];
SlaveConfig myConfig;

/*! \brief  Perform a 1-Wire search
 *
 *  This function shows how the OWI_SearchRom function can be used to
 *  discover all slaves on the bus. It will also CRC check the 64 bit
 *  identifiers.
 *
 *  \param  devices Pointer to an array of type OWI_device. The discovered
 *                  devices will be placed from the beginning of this array.
 *
 *  \param  len     The length of the device array. (Max. number of elements).
 *
 *  \param  buses   Bitmask of the buses to perform search on.
 *
 *  \retval SEARCH_SUCCESSFUL   Search completed successfully.
 *  \retval SEARCH_CRC_ERROR    A CRC error occured. Probably because of noise
 *                              during transmission.
 */
unsigned char SearchBuses(OWI_device * devices, unsigned char len, unsigned char buses) {
	unsigned char i, j;
	unsigned char presence;
	unsigned char * newID;
	unsigned char * currentID;
	unsigned char currentBus;
	unsigned char lastDeviation;
	unsigned char numDevices;

	// Initialize all addresses as zero, on bus 0 (does not exist).
	// Do a search on the bus to discover all addresses.
	for (i = 0; i < len; i++) {
		devices[i].bus = 0x00;
		for (j = 0; j < 8; j++) {
			devices[i].id[j] = 0x00;
		}
	}

	// Find the buses with slave devices.
	presence = OWI_DetectPresence(BUSES);

	numDevices = 0;
	newID = devices[0].id;

	// Go through all buses with slave devices.
	for (currentBus = 0x01; currentBus; currentBus <<= 1) {
		lastDeviation = 0;
		currentID = newID;
		if (currentBus & presence) { // Devices available on this bus.
			// Do slave search on each bus, and place identifiers and corresponding
			// bus "addresses" in the array.
			do {
				memcpy(newID, currentID, 8);
				OWI_DetectPresence(currentBus);
				lastDeviation = OWI_SearchRom(newID, lastDeviation, currentBus);
				currentID = newID;
				devices[numDevices].bus = currentBus;
				numDevices++;
				newID=devices[numDevices].id;
			}  while (lastDeviation != OWI_ROM_SEARCH_FINISHED);
		}
	}

	// Go through all the devices and do CRC check.
	for (i = 0; i < numDevices; i++) {
		// If any id has a crc error, return error.
		if(OWI_CheckRomCRC(devices[i].id) != OWI_CRC_OK) {
			return SEARCH_CRC_ERROR;
		}
	}
	// Else, return Successful.
	return SEARCH_SUCCESSFUL;
}

/*! \brief  Read the temperature from a DS1820 temperature sensor.
 *
 *  This function will start a conversion and read back the temperature
 *  from a DS1820 temperature sensor.
 *
 *  \param  bus A bitmask of the bus where the DS1820 is located.
 *
 *  \param  id  The 64 bit identifier of the DS1820.
 *
 *  \return The 16 bit signed temperature read from the DS1820.
 * The DS1820 must have Vdd pin connected for this code to work.
 */
signed int DS1820_ReadTemperatureNow(unsigned char bus, unsigned char * id) {
	signed int temperature;

	// Reset, presence.
	if (!OWI_DetectPresence(bus)) {
		return DS1820_ERROR; // Error
	}
	// Match the id found earlier.
	OWI_MatchRom(id, bus);
	// Send start conversion command.
	OWI_SendByte(DS1820_START_CONVERSION, bus);
	// Wait until conversion is finished.
	// Bus line is held low until conversion is finished.
	while (!OWI_ReadBit(bus))
		;

	// Reset, presence.
	if (!OWI_DetectPresence(bus)) {
		return DS1820_ERROR; // Error
	}
	// Match id again.
	OWI_MatchRom(id, bus);
	// Send READ SCRATCHPAD command.
	OWI_SendByte(DS1820_READ_SCRATCHPAD, bus);
	// Read only two first bytes (temperature low, temperature high)
	// and place them in the 16 bit temperature variable.
	temperature = OWI_ReceiveByte(bus);
	temperature |= (OWI_ReceiveByte(bus) << 8);

	return temperature;
}

// A conversion will take .75 seconds ! We're not sitting around for this !!!
signed int DS1820_StartConversion(unsigned char bus, unsigned char * id) {
	// Reset, presence.
	if (!OWI_DetectPresence(bus)) {
		return DS1820_ERROR; // Error
	}
	// Match the id found earlier.
	OWI_MatchRom(id, bus);
	// Send start conversion command.
	OWI_SendByte(DS1820_START_CONVERSION, bus);

  return 0; //All good
}

signed int DS1820_ReadTemperature(unsigned char bus, unsigned char * id) {
	signed int temperature;
	// Reset, presence.
	if (!OWI_DetectPresence(bus)) {
		return DS1820_ERROR; // Error
	}

  //Verify that our sensor is still alive
  unsigned char tmp_rom[8];
  memcpy(tmp_rom, id, 8);   //Make a copy of the address we are looking for
	OWI_SearchRom(tmp_rom, 64, bus);  //The search will cause a copy of the found address into the supplied buffer
  if (memcmp(id, tmp_rom, 8))       //If they don't match, the sensor is no longer there.
    return DS1820_ERROR;

	// Send READ SCRATCHPAD command.
	OWI_SendByte(DS1820_READ_SCRATCHPAD, bus);
	// Read only two first bytes (temperature low, temperature high)
	// and place them in the 16 bit temperature variable.
	temperature = OWI_ReceiveByte(bus);
	temperature |= (OWI_ReceiveByte(bus) << 8);

	return temperature;
}


void OWI_Inventory(void) {
  devicesFound = 0; //Re-initialize our sensor counter
  memset(&devices, 0, sizeof(OWI_device)*MAX_DEVICES);
  //Let's scan the 1-wire bus and inventory any and all devices (up to MAX_DEVICES)
#ifdef DEBUG
  uart0_puts_P("Sensors Inventoried.\n\r");
#endif

	while(SearchBuses(devices, MAX_DEVICES, BUSES) != SEARCH_SUCCESSFUL)
  {
    // Do the bus search until all ids are read without crc error. devices array will be populated
  }

	unsigned char i = 0;
	OWI_device * ds1820;

  //Our temperature sensors can be either DS18b20 or DS1820. Family codes 28 and 10 respectively
	// Search through the array and count how many sensors were discovered
  ds1820 = devices; //Grab a pointer to the device list
	while (i++ < MAX_DEVICES) {
		// Return the pointer if there is a family id match.
		if ( ((*ds1820).id[0] == DS1820_FAMILY_ID) || ((*ds1820).id[0] == DS18B20_FAMILY_ID) ) {
      ds1820++;       //Increment the device pointer
      devicesFound++; //Increment the device counter
		}
	}
  //We're done with the temperature sensors. Let's stuff our CT's in here too
  //CT addresses will be made up of the SLAVE ADDRESS + zero padding + CT number
  //Example A5000001, A5000002, etc.
  if (myConfig.totalCTs > 0)
  {
    //Sloppy but I'm tired!
    if (devicesFound >= MAX_DEVICES)
      return;
    if (myConfig.CT1)
    {
      (*ds1820).id[0] = myConfig.address;
      (*ds1820).id[7] = 1;
      (*ds1820).deviceType = DEVICE_TYPE_POWER;
      ds1820++;       //Increment the device pointer
      devicesFound++; //Increment the device counter
    }
    if (devicesFound >= MAX_DEVICES)
      return;
    if (myConfig.CT2)
    {
      (*ds1820).id[0] = myConfig.address;
      (*ds1820).id[7] = 2;
      (*ds1820).deviceType = DEVICE_TYPE_POWER;
      ds1820++;       //Increment the device pointer
      devicesFound++; //Increment the device counter
    }
    if (devicesFound >= MAX_DEVICES)
      return;
    if (myConfig.CT3)
    {
      (*ds1820).id[0] = myConfig.address;
      (*ds1820).id[7] = 3;
      (*ds1820).deviceType = DEVICE_TYPE_POWER;
      ds1820++;       //Increment the device pointer
      devicesFound++; //Increment the device counter
    }
    if (devicesFound >= MAX_DEVICES)
      return;
    if (myConfig.CT4)
    {
      (*ds1820).id[0] = myConfig.address;
      (*ds1820).id[7] = 4;
      (*ds1820).deviceType = DEVICE_TYPE_POWER;
      ds1820++;       //Increment the device pointer
      devicesFound++; //Increment the device counter
    }
    if (devicesFound >= MAX_DEVICES)
      return;
    if (myConfig.CT5)
    {
      (*ds1820).id[0] = myConfig.address;
      (*ds1820).id[7] = 5;
      (*ds1820).deviceType = DEVICE_TYPE_POWER;
      ds1820++;       //Increment the device pointer
      devicesFound++; //Increment the device counter
    }
    if (devicesFound >= MAX_DEVICES)
      return;
    if (myConfig.CT6)
    {
      (*ds1820).id[0] = myConfig.address;
      (*ds1820).id[7] = 6;
      (*ds1820).deviceType = DEVICE_TYPE_POWER;
      ds1820++;       //Increment the device pointer
      devicesFound++; //Increment the device counter
    }
  }
}

void IO_Config(void) {
/*Current Sensor analog channels
  A0 = CHAN1
  A1 = CHAN2
  A2 = CHAN3 Hi/Lo
  A3 = CHAN4
  A4 = CHAN5        JUMPER TO ->A6
  A5 = CHAN6 Hi/Lo  JUMPER TO ->A7
*/
/*
  Slave address configuration:
  No jumper is a 1
  A0 -> D5
  A1 -> D6
  A2 -> D7

  Current Transformer presence:
  No jumper is a 1
  CT1     -> D8
  CT2     -> D9
  CT3     -> D10
  CT4     -> D11
  1-WIRE  -> D12
  CT5     -> D3/TX1
  CT6     -> D13

  Current Transformer size
  No jumper is a 1 wich is small by default
  CT3 SMALL -> A4/SDA
  CT6 SMALL -> A5/SCL
*/
  //Let's configure PORTB
  // *** PB6 is used by the 1-wire bus ***
  //Enable internal pull-up resistors
  PORTB |= (1<<AO_PIN);
  PORTB |= (1<<A1_PIN);
  PORTB |= (1<<A2_PIN);
  PORTB |= (1<<CT3_PIN);
  PORTB |= (1<<CT4_PIN);
  PORTB |= (1<<CT6_PIN);

  //We're using a RS485 driver chip with RS485_REPIN as the control line
  //Let's configure the pin for output
  DDRB |= (1<<RS485_REPIN);

  //Let's configure the pins for input
  //These are used for configuration info
  DDRB &= ~(1<<AO_PIN);
  DDRB &= ~(1<<A1_PIN);
  DDRB &= ~(1<<A2_PIN);
  DDRB &= ~(1<<CT3_PIN);
  DDRB &= ~(1<<CT4_PIN);
  DDRB &= ~(1<<CT6_PIN);

  //Let's configure PORTC
  //Enable internal pull-up resistors
  PORTC |= (1<<CT3_SIZE);
  PORTC |= (1<<CT6_SIZE);

  //Let's configure the pins for input
  //These are used for configuration info
  DDRC &= ~(1<<CT3_SIZE);
  DDRC &= ~(1<<CT6_SIZE);

  //Let's configure PORTD
  //Enable internal pull-up resistors
#ifndef DEBUG
  PORTD |= (1<<CT5_PIN);  //USED WITH RS485 ON UART2 FOR DEBUGGIN!!! Disable during debug
#endif
  PORTD |= (1<<CT1_PIN);
  PORTD |= (1<<CT2_PIN);

  //Let's configure the pins for input
  //These are used for configuration info
#ifndef DEBUG
  DDRD &= ~(1<<CT5_PIN);  //USED WITH RS485 ON UART2 FOR DEBUGGIN!!! Disable during debug
#endif
  DDRD &= ~(1<<CT1_PIN);
  DDRD &= ~(1<<CT2_PIN);

  if (PIND & (1<<CT1_PIN))
    myConfig.CT1 = 1;

  if (PIND & (1<<CT2_PIN))
    myConfig.CT2 = 1;

  if (PINB & (1<<CT3_PIN))
    myConfig.CT3 = 1;

  if (PINB & (1<<CT4_PIN))
    myConfig.CT4 = 1;

#ifndef DEBUG
  if (PIND & (1<<CT5_PIN))
    myConfig.CT5 = 1;
#endif

  if (PINB & (1<<CT6_PIN))
    myConfig.CT6 = 1;

  if (PINC & (1<<CT3_SIZE))
    myConfig.CT3_SMALL = 1;

  if (PINC & (1<<CT6_SIZE))
    myConfig.CT6_SMALL = 1;

  myConfig.totalCTs = myConfig.CT1 + myConfig.CT2 + myConfig.CT3 + myConfig.CT4 + myConfig.CT5 + myConfig.CT6;

  myConfig.address = PINB & 0x0E;
  myConfig.address >>= 1;          //Shift into position because we're using PIN0 AKA D4 for RS485 direction control
  myConfig.address |= ADDRESS_BASE;
}
void ADCsetup(uint8_t channel)
{
  //Let's reset to default conditions before we start. Ensures safe channel changes
  ADCSRA = 0;
  ADMUX = 0;

  //Channel mapping is reversed on the Microduino 0 is 7, 1 is 6, etc.
  channel ^= 0b0111; //Reverse the bits

  // Set REFS0 in ADMUX to change reference voltage to AVCC and set the channel
  ADMUX |= (1<<REFS0) | channel;

  // Set the Prescaler to 128 (16000KHz/128 = 125KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

  // Set ADEN in ADCSRA to enable the ADC & start the first conversion
  ADCSRA |= (1<<ADEN) | (1<<ADSC);
}

int main (void)
{
  unsigned int c;
	OWI_device * ds1820;

  /*
   *  Initialize UART library, pass baudrate and AVR cpu clock
   *  with the macro
   *  UART_BAUD_SELECT() (normal speed mode )
   *  or
   *  UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
   */
  uart1_init(UART_BAUD_SELECT(USART_BAUDRATE, F_CPU) );
//  uart1_init(UART_BAUD_SELECT_DOUBLE_SPEED(USART_BAUDRATE, F_CPU) );

#ifdef DEBUG
  //Our debug console
  uart0_init(UART_BAUD_SELECT(USART_BAUDRATE, F_CPU) );
	char buffer[32];
#endif

  //Let's configure a 2Hz timer to take temperature measurements
  TCCR1B |= (1 << WGM12 ); // Configure timer 1 for CTC mode
  TIMSK1 |= (1 << OCIE1A ) ; // Enable CTC interrupt
  //2Hz 31249 3Hz = 46874, 4Hz = 62499
  OCR1A = 31249; // Set CTC compare value to 2Hz at 16 MHz AVR clock , with a prescaler of 1024
  TCCR1B |= ((1 << CS10 ) | (1 << CS12)); // Start timer at Fcpu /256

  /*
   * now enable interrupts, since UART library is interrupt controlled
   */
  sei();

  //Let's determine our configuration
  //Address, CT's installed and their sizes
  IO_Config();
  //Inventory the 1-wire bus
  OWI_Inventory(); //Make sure IO_Config is called first

#ifdef DEBUG
  itoa(myConfig.address, buffer, 10);   // convert interger into string (decimal format)
  uart0_puts("Address: \n\r");
  uart0_puts(buffer);        // and transmit string to UART
  uart0_puts("\n\r");        // and transmit string to UART

  if (myConfig.CT1)
    uart0_puts("CT1 installed\n\r");
  else
    uart0_puts("CT1 NOT installed\n\r");

  if (myConfig.CT2)
    uart0_puts("CT2 installed\n\r");
  else
    uart0_puts("CT2 NOT installed\n\r");

  if (myConfig.CT3)
    uart0_puts("CT3 installed\n\r");
  else
    uart0_puts("CT3 NOT installed\n\r");

  if (myConfig.CT4)
    uart0_puts("CT4 installed\n\r");
  else
    uart0_puts("CT4 NOT installed\n\r");

  if (myConfig.CT5)
    uart0_puts("CT5 installed\n\r");
  else
    uart0_puts("CT5 NOT installed\n\r");

  if (myConfig.CT6)
    uart0_puts("CT6 installed\n\r");
  else
    uart0_puts("CT6 NOT installed\n\r");

  if (myConfig.CT3_SMALL)
    uart0_puts("CT3 is SMALL\n\r");
  else
    uart0_puts("CT3 is LARGE\n\r");

  if (myConfig.CT6_SMALL)
    uart0_puts("CT6 is SMALL\n\r");
  else
    uart0_puts("CT6 is LARGE\n\r");
#endif

#ifdef DEBUG
/*
  for (uint8_t channel=0; channel<8; channel++)
  {
    uart0_puts("ADC Channel: ");
    itoa(channel, buffer, 10);
    uart0_puts(buffer);
    uart0_puts("\n\r");

    ADCsetup(channel);
    for (int i=0; i<10; i++)
    {
      while (ADCSRA & (1<<ADSC)); //Wait for conversion to finish
      ADCValue = ADCW;            //Get the value
      uart0_puts("ADC: ");
      itoa(ADCValue, buffer, 10); // convert interger into string (decimal format)
      uart0_puts(buffer);         // and transmit string to UART
      uart0_puts("\n\r");
      ADCSRA |= (1<<ADSC);        //Start another conversion
      _delay_ms(200);
    }
}
*/
#endif

#ifdef DEBUG
  uart0_puts("Sensors found: ");
  itoa(devicesFound, buffer, 10);
  uart0_puts(buffer);
  uart0_puts("\n\r");

  for (int j=0; j<devicesFound; j++)
  {
    ds1820 = &devices[j];

    if (ds1820 != NULL)
    {
      if ((*ds1820).deviceType == DEVICE_TYPE_POWER)
        uart0_puts("CT Sensor with ID: ");
      else
        uart0_puts("Temperature Sensor with ID: ");

      for (int i=0; i<8; i++)
      {
        itoa( (*ds1820).id[i], buffer, 16);
        uart0_puts(buffer);
        if (i < 7)
          uart0_puts(":");
      }
      uart0_puts("\n\r");

/*
      double temperature = (double) DS1820_ReadTemperatureNow((*ds1820).bus, (*ds1820).id) / 2.0;
      (*ds1820).temperature = temperature;
      sprintf(buffer, "Temp: %4.1f\001C\r", temperature);
      uart0_puts(buffer);
      uart0_puts("\n\r");
*/
    }
  }

  uart0_puts("Ready to work now!\n\r");
#endif

  //Let's put ourselves in receive mode
  RECEIVE;
  for(;;)
  {
    /*
     * Get received character from ringbuffer
     * uart_getc() returns in the lower byte the received character and
     * in the higher byte (bitmask) the last receive error
     * UART_NO_DATA is returned when no data is available.
     *
     */
    c = uart1_getc();
    if ( c & UART_NO_DATA )
    {
        /*
         * no data available from UART
         */
    }
    else
    {
        /*
         * new data available from UART
         * check for Frame or Overrun error
         */
        if ( c & UART_FRAME_ERROR )
        {
            /* Framing Error detected, i.e no stop bit detected */
#ifdef DEBUG
            uart0_puts_P("UART Frame Error: ");
#endif
        }
        if ( c & UART_OVERRUN_ERROR )
        {
            /*
             * Overrun, a character already present in the UART UDR register was
             * not read by the interrupt handler before the next character arrived,
             * one or more received characters have been dropped
             */
#ifdef DEBUG
            uart0_puts_P("UART Overrun Error: ");
#endif
        }
        if ( c & UART_BUFFER_OVERFLOW )
        {
            /*
             * We are not reading the receive buffer fast enough,
             * one or more received character have been dropped
             */
#ifdef DEBUG
            uart0_puts_P("Buffer overflow error: ");
#endif
        }

        switch (processPacket(c))
        {
            case CRC_ERROR:
                //CRC error
#ifdef DEBUG
                uart0_puts("CheckSum Error\n\r");
#endif
                //Get ready for the next command
                dataByte = 0;
                packet_sequence = 0;
            break;

            case PROCESSING:

                //All good, still processing
            break;

            case DONE:
                //We're done. Let's do some work
                //Let's sleep for a millisecond to allow the pi to be ready to receive
                _delay_ms(1);
                processCommand();
                //Get ready for the next command
                dataByte = 0;
                packet_sequence = 0;
            break;

            case ABORTED:
                //This packet was not addressed to us
                //Get ready for the next command
                dataByte = 0;
                packet_sequence = 0;
#ifdef DEBUG
//                uart0_puts("Wrong Address.\n\r");
#endif
            break;
        }
    }
  }
}

void sendResponse(void)
{
    int pos = 0;
    pkt_bufSend[pos++] = PKT_LEADIN_BYTE;
    pkt_bufSend[pos++] = PKT_START_BYTE;
    pkt_bufSend[pos++] = myConfig.address;
    pkt_bufSend[pos++] = 0x0B;
    pkt_bufSend[pos++] = PKTF_NACK;

    uint8_t buffer[40];
    double temp = -23.75;
    sprintf((char *)&buffer, "Water Temp: %f2", temp);

    //Data length
    uint8_t len = strlen((char *)&buffer);
    pkt_bufSend[pos++] = len;

    //add the data
    sprintf((char *)&pkt_bufSend[pos], (char *)&buffer);

    //calculate the checksum
    uint8_t cs = pkt_compute_cksum(pkt_buf_Rcv);

    //add the checksum
    pkt_bufSend[pos+len] = cs;

    //Wait for the master to be ready
//    _delay_ms(20);
    //Enable transmit mode
    TRANSMIT;
    for (int i=0; i < pos+len+1; i++)
    {
      uart1_putc(pkt_bufSend[i]);
    }
//    _delay_ms(10);
    RECEIVE;
}

void processCommand(void)
{
    uint8_t flags = pkt_buf_Rcv[PKT_FLAG];
    uint8_t chksum, command, deviceIndex;
    unsigned char deviceID[8];    //!< The 64 bit identifier.
//    uint8_t sensorIsAlive = 0;

    OWI_device * ds1820;

    char buffer[12];  //For data conversion to text

    int pos = 0;  //Keep track of the current buffer position
    int len = 0;  //Payload length

    pkt_bufSend[pos++] = PKT_LEADIN_BYTE;
    pkt_bufSend[pos++] = PKT_START_BYTE;
    pkt_bufSend[pos++] = pkt_buf_Rcv[PKT_SOURCE];
    pkt_bufSend[pos++] = myConfig.address;

    switch (flags)
    {
        case PKTF_NACK:
            pkt_bufSend[pos++] = PKTF_NACK;
        break;

        case PKTF_ACK:
            pkt_bufSend[pos++] = PKTF_ACK;
        break;

        case PKTF_REQACK:
            pkt_bufSend[pos++] = PKTF_ACK;
        break;

        case PKTF_REQID:
            //We have been ping'ed !
            pkt_bufSend[pos++] = PKTF_NACK;
            len = 1;
            pkt_bufSend[pos++] = len;     //Length
            pkt_bufSend[pos] = myConfig.address; //Payload
        break;

        case PKTF_COMMAND:
            //Hey, we've been asked to do some work. The requested command will be in the payload
            command = pkt_buf_Rcv[PKT_DATA];

#ifdef DEBUG
//            uart0_puts("Command!\n\r");
#endif
            switch (command)
            {
              case ENUMERATE_COUNT: //How many sensors do we have?
//              OWI_Inventory();  //Let's re-inventory
#ifdef DEBUG
//                uart0_puts("Item Count.\n\r");
#endif
                pkt_bufSend[pos++] = PKTF_NACK;
                len = 1;
                pkt_bufSend[pos++] = len;         //Length
                pkt_bufSend[pos] = devicesFound;  //Payload
              break;
              case ENUMERATE_ITEM:  //Get sensor details
                deviceIndex = pkt_buf_Rcv[PKT_DATA+1];  //The requested sensor index should be stuffed in here
#ifdef DEBUG
//                uart0_puts("Item Name.\n\r");
#endif
                len = 8;  //Our ID is 8 bytes long
                ds1820 = &devices[deviceIndex];
                if (ds1820 != NULL)
                {
                  pkt_bufSend[pos++] = PKTF_NACK;
                  pkt_bufSend[pos++] = len;   //Length
                  for (int i=0; i<len; i++)
                  {
                    pkt_bufSend[pos+i]   = (*ds1820).id[i];       //Payload
                  }
                }
              break;
              case TRANSMIT_ITEM: //Send the sensor data
                memcpy(&deviceID, &pkt_buf_Rcv[PKT_DATA+1], 8);  //Get the requested sensor ID
#ifdef DEBUG
/**
 *                uart0_puts("Requested: ");
 *                for (int i=0; i<8; i++)
 *                {
 *                  itoa( deviceID[i], buffer, 16);
 *                  uart0_puts(buffer);
 *                  if (i < 7)
 *                    uart0_puts(":");
 *                }
 *                uart0_puts("\n\r");
 */
#endif
                ds1820 = NULL;
                //Find the sensor in question in our list
//                sensorIsAlive = 0; //Assume the sensor is dead!
                for (int i=0; i<devicesFound; i++)
                {
                  ds1820 = &devices[i];
                  if ( !memcmp( (*ds1820).id, deviceID, 8 ) )  //In our list?
                    break;  //We found it. Leave now
                }

                if (ds1820 != NULL)
                {
#ifdef DEBUG
/**
 *                uart0_puts("Sending: ");
 *                for (int i=0; i<8; i++)
 *                {
 *                  itoa( (*ds1820).id[i], buffer, 16);
 *                  uart0_puts(buffer);
 *                  if (i < 7)
 *                    uart0_puts(":");
 *                }
 *                uart0_puts("\n\r");
 */
#endif
                  if ((*ds1820).temperature < -100)
                    pkt_bufSend[pos++] = PKTF_SENSOR_BAD;
                  else
                    pkt_bufSend[pos++] = PKTF_NACK;

                  sprintf(buffer, "%4.1f", (*ds1820).temperature);
                  len = strlen(buffer);
#ifdef DEBUG
                  uart0_puts("Temperature: ");
                  uart0_puts(buffer);
                  uart0_puts("\n\r");
#endif
                  pkt_bufSend[pos++] = len;   //Length
                  for (int i=0; i<len; i++)
                  {
                    pkt_bufSend[pos+i]   = buffer[i];       //Payload
                  }
                }
                break;
              }
        break;

        case PKTF_RESERVED:
        case PKTF_SENSOR_BAD:
        case PKTF_USER2:
        break;
    }

    chksum = pkt_compute_cksum(pkt_bufSend);
    pkt_bufSend[pos+len] = chksum;

    //Let's send the packet
    //Wait for the master to be ready
    _delay_ms(20);
    //Enable transmit mode
    TRANSMIT;
    for (int i=0; i < pos+len+1; i++)
    {
      uart1_putc(pkt_bufSend[i]);
#ifdef DEBUG
//      itoa( pkt_bufSend[i], buffer, 10);   // convert interger into string (decimal format)
//      uart0_puts(buffer);
#endif
    }
#ifdef DEBUG
//      uart0_puts("\n\r");
#endif
    _delay_ms(10);
    RECEIVE;
}

uint8_t processPacket(unsigned int c)
{
    uint8_t result = PROCESSING;

#ifdef DEBUG
    char buffer[7];
//    itoa( c, buffer, 16);
//    uart0_puts("ProcessPacket: ");
//    uart0_puts(buffer);
//    uart0_puts("\n\r");
    _delay_ms(1); //Seems to be required to enable comms. WTF?

#endif
    if (packet_sequence == 0) //If we're in the middle of a packet skip these tests
    {
      switch (c)
      {
          case PKT_LEADIN_BYTE: //Is this the start of a packet sequence?
              packet_sequence = PKT_LEADIN;
  #ifdef DEBUG
  //            uart0_puts("Lead in\n\r");
  #endif
          break;
          case PKT_START_BYTE:
              if (packet_sequence == PKT_LEADIN) //Have we really started ?
              {
                  packet_sequence = PKT_START;  //Yes, we have
                  dataByte = 0;
  #ifdef DEBUG
  //                uart0_puts("Start \n\r");
  #endif
              }
              else
                  packet_sequence = 0; //We have not
          break;
      }
  }
    switch (packet_sequence)
    {
        case PKT_LEADIN:
            break;
        case PKT_START:
            if (c == PKT_START_BYTE)
            {
                break;  // False start
            }
            pkt_buf_Rcv[PKT_DEST] = c;
            packet_sequence = PKT_DEST;

            if (c != myConfig.address) //Our address ?
                result = ABORTED;
#ifdef DEBUG
//            uart0_puts("Destination? \n\r");
#endif
            break;
        case PKT_DEST:
            pkt_buf_Rcv[PKT_SOURCE]= c;
            packet_sequence = PKT_SOURCE;
#ifdef DEBUG
//            uart0_puts("Source? \n\r");
#endif
            break;
        case PKT_SOURCE:
            pkt_buf_Rcv[PKT_FLAG] = c;
            packet_sequence = PKT_FLAG;
#ifdef DEBUG
//            uart0_puts("Flag? \n\r");
#endif
            break;
        case PKT_FLAG:
            pkt_buf_Rcv[PKT_LENGTH] = c;
            packet_sequence = PKT_LENGTH;
            dataByte = 1;
#ifdef DEBUG
//            uart0_puts("Length? \n\r");
//            itoa( c, buffer, 16);
//            uart0_puts(buffer);
//            uart0_puts("\n\r");
#endif
            break;
        case PKT_LENGTH:
        case PKT_DATA:
            pkt_buf_Rcv[PKT_LENGTH + dataByte++] = c;
            packet_sequence = PKT_DATA;
#ifdef DEBUG
//            uart0_puts("Data? \n\r");
#endif
            if (dataByte > pkt_buf_Rcv[PKT_LENGTH])
                packet_sequence = PKT_CHECKSUM;

            break;
        case PKT_CHECKSUM:
#ifdef DEBUG
//            uart0_puts("Checksum? \n\r");
#endif
            pkt_buf_Rcv[PKT_LENGTH + dataByte] = c;
            if (c != pkt_compute_cksum(pkt_buf_Rcv))
            {
//                result = CRC_ERROR;
                result = DONE;
#ifdef DEBUG
                uart0_puts("Checksum BAD \n\r");
                uart0_puts(itoa(c, buffer, 8));
                uart0_puts("\n\r");
#endif
            }
            else
            {
                result = DONE;
#ifdef DEBUG
//                uart0_puts("Checksum GOOD \n\r");
#endif
            }
            return result;
        break;
    }
    return result;
}

uint8_t pkt_compute_cksum(uint8_t * p)
{
    uint8_t ck, plen, i;

    //Include the Source, destination, flag, and length
    plen = p[PKT_LENGTH] + PKT_N_LEADIN;
    ck = p[PKT_DEST];
    for (i=PKT_DEST+1; i<plen; i++)
        ck += p[i];

    return ck;
}

//Timer ISR routine
ISR ( TIMER1_COMPA_vect )
{
 	double temperature = 0.0;
	OWI_device * ds1820;

  //Temperature conversion requires .75 seconds per device
  //We will command the slaves to begin a conversion every 2 interrupts
  //We will read the results on the next interrupt

  switch (timerIrqCount)
  {
    case 0:
      //Batter up! Start conversions
      //Let's re-scan the 1-wire bus and inventory any and all devices (up to MAX_DEVICES)
      for (int i=0; i<devicesFound; i++)
      {
        ds1820 = &devices[i];
        if ((*ds1820).deviceType != DEVICE_TYPE_POWER)
          DS1820_StartConversion((*ds1820).bus, (*ds1820).id);
      }
      timerIrqCount++;
#ifdef DEBUG
//      uart0_puts("Initiate Conversions.\n\r");
#endif
    break;
    case 1:
      //We're done, or should be by now. Read the values
      //Let's re-scan the 1-wire bus and inventory any and all devices (up to MAX_DEVICES)
      for (int i=0; i<devicesFound; i++)
      {
        ds1820 = &devices[i];
        if ((*ds1820).deviceType != DEVICE_TYPE_POWER)
        {
          temperature = (double) DS1820_ReadTemperature((*ds1820).bus, (*ds1820).id) / 2.0;
          (*ds1820).temperature = temperature;
        }
      }
      timerIrqCount = 0; //Reset
#ifdef DEBUG
//      uart0_puts("Read Values.\n\r");
#endif
    break;
    default:  //Loop zero
      timerIrqCount++;
#ifdef DEBUG
//      uart0_puts("Just looping.\n\r");
#endif
  }
}
