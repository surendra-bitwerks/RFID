/*
 * RFID.c
 *
 * Created: 4/8/2020 1:39:12 PM
 * Author : Alex
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif /* F_CPU */

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

#include "Serial.h"
#include "spi.h"


#define PORT_CS						PORTB
#define CS_PIN						PORTB2


#define LED_ON()					PORTB |= (1 << PORTB5)
#define LED_OFF()					PORTB &= ~(1 << PORTB5)


#define CS_LOW()					PORT_CS &=~ (1 << CS_PIN)
#define CS_HIGH()					PORT_CS |= (1 << CS_PIN)

#define nullptr ((void*)0)

typedef enum {
	STATUS_OK				,	// Success
	STATUS_ERROR			,	// Error in communication
	STATUS_COLLISION		,	// Collission detected
	STATUS_TIMEOUT			,	// Timeout in communication.
	STATUS_NO_ROOM			,	// A buffer is not big enough.
	STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
	STATUS_INVALID			,	// Invalid argument.
	STATUS_CRC_WRONG		,	// The CRC_A does not match
	STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
	}StatusCode;
	
typedef enum {
	// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
	PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
	PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
	PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
	PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
	PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
	// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
	PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
	PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
	PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
	PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
	PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
	// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
	PICC_CMD_UL_WRITE		= 0xA2		// Writes one 4 byte page to the PICC.
}PICC_Command;


typedef enum  {
	// Page 0: Command and status
	//						  0x00			// reserved for future use
	CommandReg				= 0x01 << 1,	// starts and stops command execution
	ComIEnReg				= 0x02 << 1,	// enable and disable interrupt request control bits
	DivIEnReg				= 0x03 << 1,	// enable and disable interrupt request control bits
	ComIrqReg				= 0x04 << 1,	// interrupt request bits
	DivIrqReg				= 0x05 << 1,	// interrupt request bits
	ErrorReg				= 0x06 << 1,	// error bits showing the error status of the last command executed
	Status1Reg				= 0x07 << 1,	// communication status bits
	Status2Reg				= 0x08 << 1,	// receiver and transmitter status bits
	FIFODataReg				= 0x09 << 1,	// input and output of 64 byte FIFO buffer
	FIFOLevelReg			= 0x0A << 1,	// number of bytes stored in the FIFO buffer
	WaterLevelReg			= 0x0B << 1,	// level for FIFO underflow and overflow warning
	ControlReg				= 0x0C << 1,	// miscellaneous control registers
	BitFramingReg			= 0x0D << 1,	// adjustments for bit-oriented frames
	CollReg					= 0x0E << 1,	// bit position of the first bit-collision detected on the RF interface
	//						  0x0F			// reserved for future use
	
	// Page 1: Command
	// 						  0x10			// reserved for future use
	ModeReg					= 0x11 << 1,	// defines general modes for transmitting and receiving
	TxModeReg				= 0x12 << 1,	// defines transmission data rate and framing
	RxModeReg				= 0x13 << 1,	// defines reception data rate and framing
	TxControlReg			= 0x14 << 1,	// controls the logical behavior of the antenna driver pins TX1 and TX2
	TxASKReg				= 0x15 << 1,	// controls the setting of the transmission modulation
	TxSelReg				= 0x16 << 1,	// selects the internal sources for the antenna driver
	RxSelReg				= 0x17 << 1,	// selects internal receiver settings
	RxThresholdReg			= 0x18 << 1,	// selects thresholds for the bit decoder
	DemodReg				= 0x19 << 1,	// defines demodulator settings
	// 						  0x1A			// reserved for future use
	// 						  0x1B			// reserved for future use
	MfTxReg					= 0x1C << 1,	// controls some MIFARE communication transmit parameters
	MfRxReg					= 0x1D << 1,	// controls some MIFARE communication receive parameters
	// 						  0x1E			// reserved for future use
	SerialSpeedReg			= 0x1F << 1,	// selects the speed of the serial UART interface
	
	// Page 2: Configuration
	// 						  0x20			// reserved for future use
	CRCResultRegH			= 0x21 << 1,	// shows the MSB and LSB values of the CRC calculation
	CRCResultRegL			= 0x22 << 1,
	// 						  0x23			// reserved for future use
	ModWidthReg				= 0x24 << 1,	// controls the ModWidth setting?
	// 						  0x25			// reserved for future use
	RFCfgReg				= 0x26 << 1,	// configures the receiver gain
	GsNReg					= 0x27 << 1,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation
	CWGsPReg				= 0x28 << 1,	// defines the conductance of the p-driver output during periods of no modulation
	ModGsPReg				= 0x29 << 1,	// defines the conductance of the p-driver output during periods of modulation
	TModeReg				= 0x2A << 1,	// defines settings for the internal timer
	TPrescalerReg			= 0x2B << 1,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
	TReloadRegH				= 0x2C << 1,	// defines the 16-bit timer reload value
	TReloadRegL				= 0x2D << 1,
	TCounterValueRegH		= 0x2E << 1,	// shows the 16-bit timer value
	TCounterValueRegL		= 0x2F << 1,
	
	// Page 3: Test Registers
	// 						  0x30			// reserved for future use
	TestSel1Reg				= 0x31 << 1,	// general test signal configuration
	TestSel2Reg				= 0x32 << 1,	// general test signal configuration
	TestPinEnReg			= 0x33 << 1,	// enables pin output driver on pins D1 to D7
	TestPinValueReg			= 0x34 << 1,	// defines the values for D1 to D7 when it is used as an I/O bus
	TestBusReg				= 0x35 << 1,	// shows the status of the internal test bus
	AutoTestReg				= 0x36 << 1,	// controls the digital self-test
	VersionReg				= 0x37 << 1,	// shows the software version
	AnalogTestReg			= 0x38 << 1,	// controls the pins AUX1 and AUX2
	TestDAC1Reg				= 0x39 << 1,	// defines the test value for TestDAC1
	TestDAC2Reg				= 0x3A << 1,	// defines the test value for TestDAC2
	TestADCReg				= 0x3B << 1		// shows the value of ADC I and Q channels
	// 						  0x3C			// reserved for production tests
	// 						  0x3D			// reserved for production tests
	// 						  0x3E			// reserved for production tests
	// 						  0x3F			// reserved for production tests
	}PCD_Register;
	
// MFRC522 commands. Described in chapter 10 of the datasheet.
typedef enum {
	PCD_Idle				= 0x00,		// no action, cancels current command execution
	PCD_Mem					= 0x01,		// stores 25 bytes into the internal buffer
	PCD_GenerateRandomID	= 0x02,		// generates a 10-byte random ID number
	PCD_CalcCRC				= 0x03,		// activates the CRC coprocessor or performs a self-test
	PCD_Transmit			= 0x04,		// transmits data from the FIFO buffer
	PCD_NoCmdChange			= 0x07,		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
	PCD_Receive				= 0x08,		// activates the receiver circuits
	PCD_Transceive 			= 0x0C,		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
	PCD_MFAuthent 			= 0x0E,		// performs the MIFARE standard authentication as a reader
	PCD_SoftReset			= 0x0F		// resets the MFRC522
}PCD_Command;

void PCD_WriteRegister(	PCD_Register reg,	uint8_t value);
void PCD_WriteRegister_N(PCD_Register reg, uint8_t count, uint8_t *values);
void PCD_init(void);
uint8_t PCD_ReadRegister( PCD_Register reg);
void PCD_ReadRegister_N(PCD_Register reg, uint8_t count, uint8_t *values, uint8_t rxAlign);
void PCD_AntennaOn(void);
void PCD_ClearRegisterBitMask(PCD_Register reg, uint8_t mask);
void PCD_SetRegisterBitMask(PCD_Register reg, uint8_t mask);
bool PICC_is_new_card_present(void);


StatusCode PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize);
StatusCode PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize);
StatusCode PCD_TransceiveData(uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits /* = nullptr*/, uint8_t rxAlign/* = 0*/, bool checkCRC/* = false*/);
StatusCode PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData /*= nullptr*/, uint8_t *backLen /*= nullptr*/, uint8_t *validBits /*= nullptr*/, uint8_t rxAlign /*= 0*/, bool checkCRC /*= false*/);

int 
main(void)
{
	uart_init();
	spi_init();
	
	uart_transmit_string ("Initializing PCD\r\n");
	PCD_init ();
	
	_delay_ms (4);
	uart_transmit_string ("Scan PICC to see UID, SAK, type and data blocks... \r\n");
	
	
	
    while (1) 
    {
		if( ! PICC_is_new_card_present ())
		{
			uart_transmit_string("Card_not_present\r\n");
			continue;
		}
		
		uart_transmit_string("MAIN\r\n");
    }
}

void
PCD_WriteRegister(	PCD_Register reg,	uint8_t value)
{
	CS_LOW();
	spi_send_byte(reg);
	spi_send_byte(value);
	CS_LOW();
	
}

void PCD_WriteRegister_N(	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
							uint8_t count,			///< The number of bytes to write to the register
							uint8_t *values		///< The values to write. Byte array.
						)
{
	CS_LOW();
	spi_send_byte (reg);
	for (uint8_t index = 0; index < count; index++)
	{
		spi_send_byte (values[index]);
	}
	CS_HIGH();
}

uint8_t
PCD_ReadRegister( PCD_Register reg)
{
	uint8_t value;
	
	CS_LOW();
	spi_send_byte(0x80 | reg);
	value = spi_receive_byte();
	CS_HIGH();
	//uart_transmit_string("value received\r\n");	
	return value;
}

void PCD_ReadRegister_N(PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
						uint8_t count,			///< The number of bytes to read
						uint8_t *values,		///< Byte array to store the values in.
						uint8_t rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
					   ) 
{
	if (count == 0)
	{
		return;
	}
	
	uint8_t address = 0x80 | reg;
	uint8_t index = 0;
	
	CS_LOW();
	count--;
	spi_send_byte (address);				// Tell MFRC522 which address we want to read
	
	if (rxAlign)				// Only update bit positions rxAlign..7 in values[0]
	{
		// Create bit mask for bit positions rxAlign..7
		uint8_t mask = (0xFF << rxAlign) & 0xFF;
		
		uint8_t value = spi_receive_byte ();
		
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
		spi_send_byte (address);
	}
	
	while (index < count)
	{
		values[index] = spi_receive_byte ();
		index++;
		spi_send_byte (address);
	}
	
	values[index] = spi_receive_byte ();
	
	CS_HIGH();	
}

void 
PCD_AntennaOn()
{
	uint8_t value = PCD_ReadRegister(TxControlReg);
	if((value & 0x03) != 0x03)
	{
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
}

void 
PCD_init(void)
{
	DDRB |= (1 << CS_PIN);		//Set CS_PIN as an output.
	
	uart_transmit_string("TXmodeREg ");
	PCD_WriteRegister(TxModeReg, 0x00);
	uart_transmit_string("completed\r\n");
	
	uart_transmit_string("RxmodeReg ");
	PCD_WriteRegister(RxModeReg, 0x00);
	uart_transmit_string("completed\r\n");
	
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25?s.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();
}

void PCD_SetRegisterBitMask(PCD_Register reg, uint8_t mask)
{
	uint8_t temp;
	temp = PCD_ReadRegister(reg);
	PCD_WriteRegister( reg, temp | mask);
}


void PCD_ClearRegisterBitMask(PCD_Register reg, uint8_t mask)
{
	uint8_t temp;
	temp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, temp & (~mask));
}

bool PICC_is_new_card_present(void)
{
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	
	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);
	
	StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
	return( (result == STATUS_OK) || (result == STATUS_COLLISION) );	
}


StatusCode PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize)
{
	return PICC_REQA_or_WUPA (PICC_CMD_REQA, bufferATQA, bufferSize);
}

StatusCode PICC_REQA_or_WUPA (uint8_t command, uint8_t *bufferATQA, uint8_t *bufferSize)
{
	uint8_t validBits;
	StatusCode status;
	
	if (bufferATQA == nullptr || *bufferSize < 2)			// The ATQA response is 2 bytes long.
	{
		return STATUS_NO_ROOM;
	}
	
	PCD_ClearRegisterBitMask (CollReg, 0x80);				// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;											// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData (&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK)
	{
		return status;
	}
	
	if (*bufferSize != 2 || validBits != 0)
	{
		return STATUS_ERROR;
	}
	return	STATUS_OK;
	
}



StatusCode PCD_TransceiveData(uint8_t *sendData, 
							  uint8_t sendLen, 
							  uint8_t *backData, 
							  uint8_t *backLen, 
							  uint8_t *validBits /*= nullptr*/, 
							  uint8_t rxAlign /*= 0*/, 
							  bool checkCRC/* = false*/)
{
	uint8_t waitIRq = 0x30;
	return PCD_CommunicateWithPICC (PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}


StatusCode 
PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, 
								   uint8_t sendLen, uint8_t *backData /*= nullptr*/, uint8_t *backLen /*= nullptr*/, 
								   uint8_t *validBits /*= nullptr*/, uint8_t rxAlign /*= 0*/,bool checkCRC /*= false*/)
{
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;			// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister (CommandReg, PCD_Idle);					// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);						// Clear all seven interrupt request bits
	PCD_WriteRegister(FIFOLevelReg, 0x80);					// FlushBuffer = 1, FIFO initialization
	
	PCD_WriteRegister_N (FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);			// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	
	if (command == PCD_Transceive)
	{
		PCD_SetRegisterBitMask (BitFramingReg, 0x80);
	}
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86?s.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	
	uint16_t i;
	for (i = 2000; i > 0; i--)
	{
		uint8_t n = PCD_ReadRegister (ComIrqReg);
		if (n & waitIRq)
		{
			break;
		}
		if (n & 0x01)
		{
			return STATUS_TIMEOUT;
		}
	}
	
	if (i == 0)
	{
		return STATUS_TIMEOUT;
	}
	
	uint8_t errorRegValue = PCD_ReadRegister (ErrorReg);
	if (errorRegValue & 0x13)
	{
		return STATUS_ERROR;
	}
	
	uint8_t _validBits = 0;
	
	if (backData && backLen)
	{
		uint8_t n = PCD_ReadRegister (FIFOLevelReg);
		if (n > *backLen)
		{
			return STATUS_NO_ROOM;
		}
		*backLen = n;
		
		PCD_ReadRegister_N (FIFODataReg, n, backData, rxAlign);
		_validBits = PCD_ReadRegister (ControlReg) & 0x07;
		if (validBits)
		{
			*validBits = _validBits;
		}
		
		//Tell about collisions
		
		if (errorRegValue & 0x08)
		{
			return STATUS_COLLISION;
		}
		
		//Perform CRC_A validation if requested.
		if (backData && backLen && checkCRC)
		{
			// In this case a MIFARE Classic NAK is not OK.
			if (*backLen == 1 && _validBits == 4)
			{
				return STATUS_MIFARE_NACK;
			}
			// We need at least the CRC_A value and all 8 bits of the last byte must be received.
			if (*backLen < 2 || _validBits != 0) {
				return STATUS_CRC_WRONG;
			}
			// Verify CRC_A - do our own calculation and store the control in controlBuffer.
			//uint8_t controlBuffer[2];
			
			/*StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
			if (status != STATUS_OK) {
				return status;
			}
			if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
				return STATUS_CRC_WRONG;
			}*/
		}
		
	}
	return STATUS_OK;
}



