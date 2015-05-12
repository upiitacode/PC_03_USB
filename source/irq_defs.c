extern void SysTickIntHandler(void);
extern void UARTStdioIntHandler(void);
extern void USB0DeviceIntHandler(void);

void SysTick_Handler (void){
	SysTickIntHandler();
}

void UART0_Handler(void){
	UARTStdioIntHandler();
}

void USB0_Handler(void){
	USB0DeviceIntHandler();
}