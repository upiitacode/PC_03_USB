//*****************************************************************************
//
// usb_dev_bulk.c - Main routines for the generic bulk device example.
//
// Copyright (c) 2012-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "inc/hw_ints.h"
//#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "usblib/usblib.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdbulk.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usb_bulk_structs.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB Generic Bulk Device (usb_dev_bulk)</h1>
//!
//! This example provides a generic USB device offering simple bulk data
//! transfer to and from the host.  The device uses a vendor-specific class ID
//! and supports a single bulk IN endpoint and a single bulk OUT endpoint.
//! Data received from the host is assumed to be ASCII text and it is
//! echoed back with the case of all alphabetic characters swapped.
//!
//! A Windows INF file for the device is provided on the installation CD and
//! in the C:/ti/TivaWare-for-C-Series/windows_drivers directory of TivaWare C
//! series releases.  This INF contains information required to install the
//! WinUSB subsystem on Windowi16XP and Vista PCs.  WinUSB is a Windows
//! subsystem allowing user mode applications to access the USB device without
//! the need for a vendor-specific kernel mode driver.
//!
//! A sample Windows command-line application, usb_bulk_example, illustrating
//! how to connect to and communicate with the bulk device is also provided.
//! The application binary is installed as part of the ''Windows-side examples
//! for USB kits'' package (SW-USB-win) on the installation CD or via download
//! from http://www.ti.com/tivaware .  Project files are included to allow
//! the examples to be built using Microsoft VisualStudio 2008.  Source code
//! for this application can be found in directory
//! TivaWare-for-C-Series/tools/usb_bulk_example.
//
//*****************************************************************************

//*****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     100
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// The global system tick counter.
//
//*****************************************************************************
volatile uint32_t g_ui32SysTickCount = 0;

//*****************************************************************************
//
// Variables tracking transmit and receive counts.
//
//*****************************************************************************
volatile uint32_t g_ui32TxCount = 0;
volatile uint32_t g_ui32RxCount = 0;
#ifdef DEBUG
uint32_t g_ui32UARTRxErrors = 0;
#endif

//*****************************************************************************
//
// Debug-related definitions and declarations.
//
// Debug output is available via UART0 if DEBUG is defined during build.
//
//*****************************************************************************
#ifdef DEBUG
//*****************************************************************************
//
// Map all debug print calls to UARTprintf in debug builds.
//
//*****************************************************************************
#define DEBUG_PRINT UARTprintf

#else

//*****************************************************************************
//
// Compile out all debug print calls in release builds.
//
//*****************************************************************************
#define DEBUG_PRINT while(0) ((int (*)(char *, ...))0)
#endif

//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;

//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
static volatile bool g_bUSBConfigured = false;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    UARTprintf("Error at line %d of %s\n", ui32Line, pcFilename);
    while(1)
    {
    }
}
#endif

//*****************************************************************************
//
// Interrupt handler for the system tick counter.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Update our system tick counter.
    //
    g_ui32SysTickCount++;
}

int txReady=1;

//*****************************************************************************
//
// Handles bulk driver notifications related to the transmit channel (data to
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if(ui32Event == USB_EVENT_TX_COMPLETE)
    {
        g_ui32TxCount += ui32MsgValue;
		txReady=0;
    }

    //
    // Dump a debug message.
    //
    DEBUG_PRINT("TX complete %d\n", ui32MsgValue);

    return(0);
}

void * myBulk;
unsigned char myInBuffer[256];
int rxReady=0;
int readBytes=0;
//*****************************************************************************
//
// Handles bulk driver notifications related to the receive channel (data from
// the USB host).
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the bulk driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            UARTprintf("Host connected.\n");

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            UARTprintf("Host disconnected.\n");
            break;
        }

        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
            //
            // Get a pointer to our instance data from the callback data
            // parameter.
            //
			
			//UARTprintf("pvCBData %p\n",pvCBData);
			//UARTprintf("message value %d\n",ui32MsgValue);
		
			memcpy(myInBuffer,pvMsgData,ui32MsgValue);
			readBytes=ui32MsgValue;
			rxReady=1;
			//
			// Read the new packet and echo it back to the host.
			//
			return ui32MsgValue;
			
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        //
        // Ignore all other events and return 0.
        //
        default:
        {
            break;
        }
    }

    return(0);
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

// DMA ADC variable and fucntion declarations

void config_ADC(void);
void start_ADC(void);
void stop_ADC(void);


short ADC_samples[1024];
char trasferData[64] __attribute__((aligned(2)));
int DMA_CH14_Transfers=0;
int dmaTransferComplete=1;
int uartTrasferComplete=1;


void config_DMA(void);
void reloadDMA_DMA_Chanel14(void);


typedef struct {
  void * sourceEndPointer;
	void * destinationEndPointer;
  unsigned int controlWord;
	unsigned int unused;
} CHControlStruct;

typedef struct {
  CHControlStruct primary[32];
	CHControlStruct alternate[32];
} DMAControlStruct;

DMAControlStruct myControlStruct __attribute__((aligned(1024)));
 
void startUART_Transfer(void);


//*****************************************************************************
//
// This is the main application entry function.
//
//*****************************************************************************

void config_sal(void);
int main(void)
{
	  volatile uint32_t ui32Loop;
    uint32_t ui32TxCount;
    uint32_t ui32RxCount;
		int myCounter =0;
		void * myBulk;
	 
	
		

	
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run from the PLL at 50MHz
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2 & PF3).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIOF_BASE, GPIO_PIN_3 | GPIO_PIN_2);

    //
    // Open UART0 and show the application name on the UART.
    //
    ConfigureUART();
		
	
    UARTprintf("\033[2JTiva C Series USB bulk device example\n");
    UARTprintf("---------------------------------\n\n");
		
		
	
    //
    // Not configured initially.
    //
    g_bUSBConfigured = false;

    //
    // Enable the GPIO peripheral used for USB, and configure the USB
    // pins.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIOD_BASE, GPIO_PIN_4 | GPIO_PIN_5);


    //
    // Tell the user what we are up to.
    //
		UARTprintf("Configuring USB\n");

    //
    // Initialize the transmit and receive buffers.
    //
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    //
    // Set the USB stack mode to Device mode with VBUS monitoring.
    //
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    myBulk=USBDBulkInit(0, &g_sBulkDevice);

    //
    // Wait for initial configuration to complete.
    //
    UARTprintf("Waiting for host...\n");

    //
    // Clear our local byte counters.
    //
    ui32RxCount = 0;
    ui32TxCount = 0;

    //
    // Main application loop.
    //
		
	  config_ADC();
		config_DMA();
		config_sal();
	  NVIC_EnableIRQ(ADC0SS0_IRQn);//the DMA asserts this interrupt when transfer complete
		
		
		 
		stop_ADC(); 
		int itemsToPacket=32;
    while(1)
    {
			if((usbReady!=0)){	
				reloadDMA_DMA_Chanel14();
				dmaTransferComplete=0;
				start_ADC(); 
				while(!dmaTransferComplete);
				
				UARTprintf("Transfer start\n");
				GPIOB->DATA|=(0x1<<0);
				for(int index=0;index<32;index++){
					for(int j=0;j<itemsToPacket;j++){
						trasferComplete=0;
					  ((short*)&trasferData)[j]=ADC_samples[(index*itemsToPacket)+j];
					}
						USBDBulkPacketWrite(myBulk,(unsigned char *)trasferData,itemsToPacket*2,true);
						while(!trasferComplete);
				}
				UARTprintf("Transfer complete %d\n",DMA_CH14_Transfers);
				GPIOB->DATA&=~(0x1<<0);// saca un  0 en PC5
				stop_ADC();
			}
    }
}

void startUART_Transfer(void){
	int i;
	char tempCharBuffer[80];
	for(i=0;i<1024;i++){
		UARTprintf("DATA[%d]=%d\n",i,ADC_samples[i]);
		SysCtlDelay(10000);
	}
	UARTprintf("Transfer complete\n");
	UARTprintf("Total DMA trasfers: %d\n",DMA_CH14_Transfers);
	UARTprintf("Press enter to continue\n");
	UARTgets(tempCharBuffer,80);
	uartTrasferComplete=1;
}


void config_DMA(){
	//Module Initialization
	SYSCTL->RCGCDMA|=0x1;//enable DMA peripherial
	UDMA->CFG=0x0;//Disables DMA controller
	myControlStruct.primary[14].sourceEndPointer=(void *)0x40038048;
	myControlStruct.primary[14].destinationEndPointer=(void *)&ADC_samples[1023];
	myControlStruct.primary[14].controlWord=(0x1<<30)|//Destination increment:Half-word
																					(0x1<<28)|//Destination Data Size:Half-Word
																					(0x3<<26)|//Source address increment:	No increment						
																					(0x1<<24)|//Source Data Size:Half-Word
																					(0x2<<14)|//Arbitration Size 4 Trasfers
																					(1023<<4)|//Trasfers Size 1024 //reg =(n-1)
																					(0x1<<0);//Transfer Mode Basic
	UDMA->CTLBASE=(int)&myControlStruct;//pointer to base of DMA Control Structure
	//Configure Channel 14  Map Select 0 = ADC0 SS0
	UDMA->ALTCLR|=(0x1<<14);//use primary control structure
	UDMA->USEBURSTSET|=(0x1<<14);//channel 14 responds onnly to burst requests
	UDMA->REQMASKCLR|=(0x1<<14);
	UDMA->CHMAP1|=(0x0<<24);
	
	UDMA->ENASET|=(0x1<<14);//Enable Chanel
	UDMA->CFG=0x1;//Enables DMA controller
}

void reloadDMA_DMA_Chanel14(){
	//Module Initialization
	UDMA->ENACLR|=(0x1<<14);
	myControlStruct.primary[14].controlWord=(0x1<<30)|//Destination increment:Half-word
																					(0x1<<28)|//Destination Data Size:Half-Word
																					(0x3<<26)|//Source address increment:	No increment						
																					(0x1<<24)|//Source Data Size:Half-Word
																					(0x2<<14)|//Arbitration Size 4 Trasfers
																					(1023<<4)|//Trasfers Size 1024 //reg =(n-1)
																					(0x1<<0);//Transfer Mode Basic
	//UDMA->USEBURSTSET|=(0x1<<14);//channel 14 responds onnly to burst requests
	//UDMA->REQMASKCLR|=(0x1<<14);
	//UDMA->CHMAP1|=(0x0<<24);
	UDMA->ENASET|=(0x1<<14);//Enable Chanel
}

void start_ADC(void){
	ADC0->ACTSS|=(0x01);
	ADC0->PSSI=(0x1<<0);//inicia secuenciador 0
	//ADC_sampleEnd=0;
}

void stop_ADC(void){
	int dummie;
	while(!(ADC0->SSFSTAT0&(0x1<<8))) dummie=ADC0->SSFIFO0;
	ADC0->ACTSS&=~((int)0x01);//disable sequencer
	ADC0->DCISC=0x1;//clear interrupts

}

void config_ADC(){
	/*Configuracion de Pines*/
	
	SYSCTL->RCGCGPIO|=(0x1<<4)|(0x1<<1)|(0x1<<3);
	SYSCTL->RCGCADC|=0x01;//habilitamos ADC0
	// Puertos Analogicos en E4 E5 B4 B5
	GPIOE->DEN&=~((0x1<<4)|(0x1<<5));
	GPIOB->DEN&=~((0x1<<4)|(0x1<<5));
	GPIOE->AMSEL|=(0x1<<4)|(0x1<<5);
	GPIOB->AMSEL|=(0x1<<4)|(0x1<<5);
	
	
	//Configuracion ADC principal
	
	ADC0->ACTSS&=~(0xF);//desactivamos secuenciadores durantes la configuracion
	ADC0->EMUX=0xF; //para trigger por sofware always(continuos sample) 
	//ADC0->CTL|=(0x1<<6);//dither mode enable
	//ADC0->SAC|=(0x3);// x8 averaging
	//Configuracion del sequenciador
	ADC0->SSMUX0=(0x8)|(0x8<<4)|(0x8<<8)|(0x8<<12)|(0x8<<16)|(0x8<<20)|(0x8<<24)|(0x8<<28);//(0x8)|(0x9<<4)|(0xA<<8)|(0xB<<12)|(0x8<<16)|(0x9<<20)|(0xA<<24)|(0xB<<28);
	ADC0->SSCTL0=(0x1<<30)|(0x1<<29)|(0x1<<14);//Final de secuencia en muestra 3, y genera interrupcion
	//Desenmacaramos interrupcion por secuenciador 0
	//ADC0->IM|=(0x1<<0);
	ADC0->ACTSS|=(0x1<<0);//activamos secuenciador 0
}

void ADC0SS0_Handler(){
	
		UDMA->CHIS|=(0x1<<14);//clear interrupts caused by channel 14
		//reloadDMA_DMA_Chanel14();
	  dmaTransferComplete=1;
		DMA_CH14_Transfers++;
	  stop_ADC(); 
}

//extern void SysTickIntHandler(void);

extern void USB0DeviceIntHandler(void);
void USB0_Handler(void){
	USB0DeviceIntHandler();
}

extern void UARTStdioIntHandler(void);
void UART0_Handler(void){
	UARTStdioIntHandler();
}

void config_sal(void){
	SYSCTL->RCGCGPIO|=(0x1<<1)|(0x1<<5);// primero ver  los puertos a utilizar PUERTOC 
	GPIOB->DEN|=(0x1<<0)|(0x1<<1)|(0x1<<2)|(0x1<<3);//habilitammos los puertos entreda o salida o funcion espesifica PC4 y PC5 PC6 y PC7 
	GPIOB->DIR|=(0x1<<0)|(0x1<<1)|(0x1<<2)|(0x1<<3);//PC4,PC5,PC6,PC7" como salida ; ya salen cosas hay
	//GPIOB->PUR|=(0x1<<0)|(0x1<<1)|(0x1<<2)|(0x1<<3);// habilitamos como pull up 
	//GPIOF->DEN|=(0x1<<2)|(0x1<<3);//habilitammos los puertos entreda o salida o funcion espesifica PF2 y  PF3
	//GPIOF->DIR|=(0x1<<2)|(0x1<<3);//PF2, PF3" como salida ; ya salen cosas hay 
}
