/*			Dual boot for Arduino Due
	Copyright (C) Xulio Coira, 2014
	Copyright (C) Alfredo Prado ,2014

	xulioc [at] gmail [dot] com
	radikalbytes [at] gmail [dot] com

	Firmware based on Arduino Due bootloader (Arduino Ltd.)
	Firmware based on MIDI shield from openpipe.cc
	Writen using LUFA library 100807
*/
/*
             LUFA Library
     Copyright (C) Dean Camera, 2010.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com

  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this 
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in 
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting 
  documentation, and that the name of the author not be used in 
  advertising or publicity pertaining to distribution of the 
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/


/** \file
 *
 *  Main source file for the Arduino-usbserial project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "dual_boot_Due.h"

#define MIDI_BAUDRATE 31250
 //#define MIDI_BAUDRATE 115200
 //#define MIDI_BAUDRATE 500000
 //#define MIDI_BAUDRATE 1000000

uint8_t tmp;
int16_t received;
uint8_t command, channel, data1, data2, data3;
uint8_t status;

MIDI_EventPacket_t ReceivedMIDIEvent;

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
RingBuff_t USBtoUSART_Buffer;

/** Circular buffer to hold data from the serial port before it is sent to the host. */
RingBuff_t USARTtoUSB_Buffer;

/** Pulse generation counters to keep track of the number of milliseconds remaining for each pulse type */
volatile struct
{
	uint8_t TxLEDPulse; /**< Milliseconds remaining for data Tx LED pulse */
	uint8_t RxLEDPulse; /**< Milliseconds remaining for data Rx LED pulse */
	uint8_t PingPongLEDPulse; /**< Milliseconds remaining for enumeration Tx/Rx ping-pong LED pulse */
} PulseMSRemaining;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config = 
			{
				.ControlInterfaceNumber         = 0,

				.DataINEndpointNumber           = CDC_TX_EPNUM,
				.DataINEndpointSize             = CDC_TXRX_EPSIZE,
				.DataINEndpointDoubleBank       = false,

				.DataOUTEndpointNumber          = CDC_RX_EPNUM,
				.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
				.DataOUTEndpointDoubleBank      = false,

				.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
				.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
				.NotificationEndpointDoubleBank = false,
			},
	};

/** LUFA MIDI Class driver interface configuration and state information. This structure is
 *  passed to all MIDI Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface =
	{
		.Config =
			{
				.StreamingInterfaceNumber = 1,

				.DataINEndpointNumber      = MIDI_STREAM_IN_EPNUM,
				.DataINEndpointSize        = MIDI_STREAM_EPSIZE,
				.DataINEndpointDoubleBank  = false,

				.DataOUTEndpointNumber     = MIDI_STREAM_OUT_EPNUM,
				.DataOUTEndpointSize       = MIDI_STREAM_EPSIZE,
				.DataOUTEndpointDoubleBank = false,
			},
	};

int ResetTimer = 0;
int tries = 20;
bool CurrentDTRState = false;
bool PreviousDTRState = false;
bool CurrentRTSState = false;
bool PreviousRTSState = false;
bool Selected1200BPS = false;

/* Mode definitions. This global varaible holds the running mode, either MIDI or SERIAL.
*/

#define MODE_MIDI 0
#define MODE_SERIAL 1
#define MODE_SELECTION_DELAY_SECONDS 1

int mode;

/* Force hard reset to re-enumerate USB device */
void forceHardReset() 
{ 
  cli(); // disable interrupts 
  wdt_enable(WDTO_15MS); // enable watchdog 
  while(1); // wait for watchdog to reset processor 
}

void setResetPin(bool v) {
	/* Target /RESET line  */
	if (v) {
		/* ACTIVE   => OUTPUT LOW (0v on target /RESET) */
		AVR_RESET_LINE_DDR  |= AVR_RESET_LINE_MASK;
		AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
	} else {
	 	/* INACTIVE => set as INPUT (internal pullup on target /RESET keep it at 3.3v) */
		AVR_RESET_LINE_DDR  &= ~AVR_RESET_LINE_MASK;
		AVR_RESET_LINE_PORT &= ~AVR_RESET_LINE_MASK;
	}
}

void setErasePin(bool v) {
	if (v) {
		AVR_ERASE_LINE_PORT &= ~AVR_ERASE_LINE_MASK;
	} else {
		AVR_ERASE_LINE_PORT |= AVR_ERASE_LINE_MASK;
	}
}


/* Send a MIDI command by USB */
void USB_MIDI_Send_Command(unsigned char channel, unsigned char command, unsigned char data2, unsigned char data3){

	USB_MIDI_EventPacket_t MIDIEvent = (USB_MIDI_EventPacket_t)
		{
			.CableNumber = 0, 
			.Command 	 = (command >> 4),
			.Data1       = command | channel,
			.Data2       = data2,
			.Data3       = data3,
		};

	MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent);
	MIDI_Device_Flush(&Keyboard_MIDI_Interface);

}


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	int rs;
	SetupHardware();
	
	RingBuffer_InitBuffer(&USBtoUSART_Buffer);
	RingBuffer_InitBuffer(&USARTtoUSB_Buffer);

	sei();


	for (;;)
	{
		if (mode==MODE_MIDI){
			/// MIDI MODE ///
			//Check hot change to Serial Mode
			if (PINB & (1<<4)) forceHardReset();
			wdt_reset();
			if (Serial_IsCharReceived()){
				tmp = Serial_RxByte();
				if (tmp<0x80){
				//RUNNING STATUS
				// http://www.blitter.com/~russtopia/MIDI/~jglatt/tech/midispec/run.htm
				received=status;
				rs=1;

				}else{
				received=tmp;
				status=received;
				rs=0;
				}
				command = (uint8_t)received & 0xF0;
				channel = (uint8_t)received & 0x0F;

				switch(command){

				// 1 BYTE COMMANDS
				case 0xC0: //ProgramChange
				case 0xD0: //AfterTouchChannel
				if (rs){
					data1=tmp;
				}else{
					while( !Serial_IsCharReceived() );
					data1 = (uint8_t)Serial_RxByte();
				}
				USB_MIDI_Send_Command(channel, command, data1, 0);
				break;

				// 2 BYTE COMMANDS
				case 0x80: //NoteOff
				case 0x90: //NoteOn
				case 0xA0: //AfterTouchPoly
				case 0xB0: //ControlChange
				case 0xE0: //PitchBend
				if (rs){
					data1=tmp;
				}else{
					while( !Serial_IsCharReceived() );
					data1 = (uint8_t)Serial_RxByte();
				}
				while( !Serial_IsCharReceived() );
				data2 = (uint8_t)Serial_RxByte();
				USB_MIDI_Send_Command(channel, command, data1, data2);
				break;

				//SYSTEM
				case 0xF0:
				if ( channel==0 ){
					//Sysex
					//TODO
				}else if ( channel >= 6 ){
					//Tune Request
					//End of Exclusive
					//System Real Time
					USB_MIDI_Send_Command(channel, command, 0, 0);
				}else if ( channel==1 || channel==3 ){
					//MIDI Time Code Quarter Frame
					//Song Select
					while( !Serial_IsCharReceived() );
					data1 = (uint8_t)Serial_RxByte();
					USB_MIDI_Send_Command(channel, command, data1, 0);
				}else if ( channel==2 ){
					//Song Position Pointer
					while( !Serial_IsCharReceived() );
					data1 = (uint8_t)Serial_RxByte();
					while( !Serial_IsCharReceived() );
					data2 = (uint8_t)Serial_RxByte();
					USB_MIDI_Send_Command(channel, command, data1, data2);
				}
				break;
				default:
					USB_MIDI_Send_Command(0, 0x80, received, 0);
					USB_MIDI_Send_Command(0, 0x80, data1, 0);
					USB_MIDI_Send_Command(0, 0x80, data2, 0);
				}
			}
				if (MIDI_Device_ReceiveEventPacket(&Keyboard_MIDI_Interface, &ReceivedMIDIEvent))
				{

					command = (uint8_t)ReceivedMIDIEvent.Data1 & 0xF0;
					channel = (uint8_t)ReceivedMIDIEvent.Data1 & 0x0F;
					switch(command){
						// 1 BYTE COMMANDS
						case 0xC0: //ProgramChange
						case 0xD0: //AfterTouchChannel
							Serial_TxByte	(ReceivedMIDIEvent.Data1);
							Serial_TxByte	(ReceivedMIDIEvent.Data2);
							break;
						// 2 BYTE COMMANDS
						case 0x80: //NoteOff
						case 0x90: //NoteOn
						case 0xA0: //AfterTouchPoly
						case 0xB0: //ControlChange
						case 0xE0: //PitchBend
							Serial_TxByte	(ReceivedMIDIEvent.Data1);
							Serial_TxByte	(ReceivedMIDIEvent.Data2);
							Serial_TxByte	(ReceivedMIDIEvent.Data3);
							break;
						//SYSTEM
						case 0xF0:
							if ( channel==0 ){
								//Sysex
								//TODO
							}else if ( channel >= 6 ){
								//Tune Request
								//End of Exclusive
								//System Real Time
								Serial_TxByte	(ReceivedMIDIEvent.Data1);
							}else if ( channel==1 || channel==3 ){
								//MIDI Time Code Quarter Frame
								//Song Select
								while( !Serial_IsCharReceived() );
								Serial_TxByte	(ReceivedMIDIEvent.Data1);
								Serial_TxByte	(ReceivedMIDIEvent.Data2);
							}else if ( channel==2 ){
								//Song Position Pointer
								Serial_TxByte	(ReceivedMIDIEvent.Data1);
								Serial_TxByte	(ReceivedMIDIEvent.Data2);
								Serial_TxByte	(ReceivedMIDIEvent.Data3);
							}
							break;
					}
				}
			MIDI_Device_USBTask(&Keyboard_MIDI_Interface);

			/// END MIDI MODE ///
		}else{
			//Check hot change to MIDI mode
			if ((PINB & (1<<4))==0) forceHardReset(); 
			/// SERIAL MODE ///
			// Only try to read in bytes from the CDC interface if the transmit buffer is not full
			if (!(RingBuffer_IsFull(&USBtoUSART_Buffer)))
			{
				int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

				// Read bytes from the USB OUT endpoint into the USART transmit buffer
				if (!(ReceivedByte < 0))
			  		RingBuffer_Insert(&USBtoUSART_Buffer, ReceivedByte);
			}
		
			// Check if the UART receive buffer flush timer has expired or the buffer is nearly full
			RingBuff_Count_t BufferCount = RingBuffer_GetCount(&USARTtoUSB_Buffer);
			if ((TIFR0 & (1 << TOV0)) || (BufferCount > BUFFER_NEARLY_FULL))
			{
				TIFR0 |= (1 << TOV0);

				if (USARTtoUSB_Buffer.Count) {
					LEDs_TurnOnLEDs(LEDMASK_TX);
					PulseMSRemaining.TxLEDPulse = TX_RX_LED_PULSE_MS;
				}

				// Read bytes from the USART receive buffer into the USB IN endpoint
				while (BufferCount--)
			  		CDC_Device_SendByte(&VirtualSerial_CDC_Interface, RingBuffer_Remove(&USARTtoUSB_Buffer));
			  
				// Turn off TX LED(s) once the TX pulse period has elapsed
				if (PulseMSRemaining.TxLEDPulse && !(--PulseMSRemaining.TxLEDPulse))
			  		LEDs_TurnOffLEDs(LEDMASK_TX);

				// Turn off RX LED(s) once the RX pulse period has elapsed
				if (PulseMSRemaining.RxLEDPulse && !(--PulseMSRemaining.RxLEDPulse))
			  		LEDs_TurnOffLEDs(LEDMASK_RX);

				if (ResetTimer > 0)
				{
					// SAM3X RESET/ERASE Sequence
					// --------------------------
					// Between 60 and 120: do erase
					if (ResetTimer >= 60 && ResetTimer <= 120) {
						setErasePin(true);
					} else {
						setErasePin(false);
					}

					// Between 1 and 50: do reset
					if (ResetTimer >= 1 && ResetTimer <= 50) {
						setResetPin(true);
					} else {
						setResetPin(false);
					}
					ResetTimer--;
				} else {
					setErasePin(false);
					setResetPin(false);
				}
			}
		
			// Load the next byte from the USART transmit buffer into the USART
			if (!(RingBuffer_IsEmpty(&USBtoUSART_Buffer))) {
				Serial_TxByte(RingBuffer_Remove(&USBtoUSART_Buffer));
				LEDs_TurnOnLEDs(LEDMASK_RX);
				PulseMSRemaining.RxLEDPulse = TX_RX_LED_PULSE_MS;
			}
		
			CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
			//End serial mode
		}
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	setResetPin(false);

	/* input used to togle USB mode */
	DDRB &= ~(1<<4);   //PB4 Input mode
	PORTB |= (1<<4);   //pull-up active in PB4

	/* Target /ERASE line is active HIGH: there is a mosfet that inverts logic */
	AVR_ERASE_LINE_PORT |= AVR_ERASE_LINE_MASK;
	AVR_ERASE_LINE_DDR  |= AVR_ERASE_LINE_MASK;	

	// SELECT MODE
	 _delay_ms(MODE_SELECTION_DELAY_SECONDS*200);
	 if (PINB & (1<<4)){
	 	/// SERIAL MODE

		mode=MODE_SERIAL;
	 	setResetPin(false);
		/* Hardware Initialization */
		Serial_Init(9600, false);
		LEDs_Init();
		USB_Init();

	 }else{
	 	/// MIDI MODE

	 	mode=MODE_MIDI;
	 	AVR_ERASE_LINE_DDR  |= AVR_ERASE_LINE_MASK;	
	 	setErasePin(false);
		setResetPin(false);

	 	// Initialize the serial USART driver before first use
    	Serial_Init(MIDI_BAUDRATE, true);
    	USB_Init();
	}
	/* Start the flush timer so that overflows occur rapidly to push received bytes to the USB interface */
	TCCR0B = (1 << CS02);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
	MIDI_Device_ConfigureEndpoints(&Keyboard_MIDI_Interface);
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
	MIDI_Device_ProcessControlRequest(&Keyboard_MIDI_Interface);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	uint8_t ConfigMask = 0;

	switch (CDCInterfaceInfo->State.LineEncoding.ParityType)
	{
		case CDC_PARITY_Odd:
			ConfigMask = ((1 << UPM11) | (1 << UPM10));		
			break;
		case CDC_PARITY_Even:
			ConfigMask = (1 << UPM11);		
			break;
	}

	if (CDCInterfaceInfo->State.LineEncoding.CharFormat == CDC_LINEENCODING_TwoStopBits)
	  ConfigMask |= (1 << USBS1);

	switch (CDCInterfaceInfo->State.LineEncoding.DataBits)
	{
		case 6:
			ConfigMask |= (1 << UCSZ10);
			break;
		case 7:
			ConfigMask |= (1 << UCSZ11);
			break;
		case 8:
			ConfigMask |= ((1 << UCSZ11) | (1 << UCSZ10));
			break;
	}

	/* Must turn off USART before reconfiguring it, otherwise incorrect operation may occur */
	UCSR1B = 0;
	UCSR1A = 0;
	UCSR1C = 0;

	long bps = CDCInterfaceInfo->State.LineEncoding.BaudRateBPS;
	Selected1200BPS = (bps == 1200);

	UBRR1  = SERIAL_2X_UBBRVAL(bps);
	UCSR1C = ConfigMask;
	UCSR1A = (1 << U2X1);
	UCSR1B = (1 << RXCIE1) | (1 << TXEN1) | (1 << RXEN1);
}

/** ISR to manage the reception of data from the serial port, placing received bytes into a circular buffer
 *  for later transmission to the host.
 */
ISR(USART1_RX_vect, ISR_BLOCK)
{
	uint8_t ReceivedByte = UDR1;

	if (USB_DeviceState == DEVICE_STATE_Configured)
	  RingBuffer_Insert(&USARTtoUSB_Buffer, ReceivedByte);
}

/** Event handler for the CDC Class driver Host-to-Device Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	PreviousDTRState = CurrentDTRState;
	PreviousRTSState = CurrentRTSState;
	CurrentDTRState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR);
	CurrentRTSState = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_RTS);

	if (Selected1200BPS) {
		/* Start Erase / Reset procedure when receiving the magic "1200" baudrate */
		ResetTimer = 120;
	} else if (!PreviousDTRState && CurrentDTRState) {
		/* Reset on rising edge of DTR */
		ResetTimer = 50;
	}
}

