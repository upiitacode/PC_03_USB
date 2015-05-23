#include "adc_dma.h"
#include "TM4C123.h"                    // Device header


void config_ADC(void);
void start_ADC(void);
void stop_ADC(void);

void config_DMA(void);
void reloadDMA_DMA_Chanel14(void);

#define N_SAMPLES 8

short ADC_samples[N_SAMPLES];
char trasferData[64] __attribute__((aligned(2)));
int DMA_CH14_Transfers = 0;
int dmaTransferComplete = 1;

DMAControlStruct myControlStruct __attribute__((aligned(1024)));

void config_DMA(void) {
	//Module Initialization
	SYSCTL->RCGCDMA |= 0x1; //enable DMA peripherial
	UDMA->CFG = 0x0; //Disables DMA controller
	myControlStruct.primary[14].sourceEndPointer = (void *) 0x40038048;
	myControlStruct.primary[14].destinationEndPointer = (void *) &ADC_samples[N_SAMPLES-1];
	myControlStruct.primary[14].controlWord = ((0x1 << 30) | //Destination increment:Half-word
			(0x1 << 28) | //Destination Data Size:Half-Word
			(0x3 << 26) | //Source address increment:	No increment						
			(0x1 << 24) | //Source Data Size:Half-Word
			(0x2 << 14) | //Arbitration Size 4 Trasfers
			((N_SAMPLES-1) << 4) | //Trasfers Size 1024 //reg =(n-1)
			(0x1 << 0)); //Transfer Mode Basic
	UDMA->CTLBASE = (int) &myControlStruct; //pointer to base of DMA Control Structure
	//Configure Channel 14  Map Select 0 = ADC0 SS0
	UDMA->ALTCLR |= (0x1 << 14); //use primary control structure
	UDMA->USEBURSTSET |= (0x1 << 14); //channel 14 responds onnly to burst requests
	UDMA->REQMASKCLR |= (0x1 << 14);
	UDMA->CHMAP1 |= (0x0 << 24);
	
	UDMA->ENASET |= (0x1 << 14); //Enable Chanel
	UDMA->CFG = 0x1; //Enables DMA controller
}

void reloadDMA_DMA_Chanel14(void) {
	//Module Initialization
	UDMA->ENACLR |= (0x1 << 14);
	myControlStruct.primary[14].controlWord = (0x1 << 30) | //Destination increment:Half-word
			(0x1 << 28) | //Destination Data Size:Half-Word
			(0x3 << 26) | //Source address increment:	No increment						
			(0x1 << 24) | //Source Data Size:Half-Word
			(0x2 << 14) | //Arbitration Size 4 Trasfers
			((N_SAMPLES-1) << 4) | //Trasfers Size 1024 //reg =(n-1)
			(0x1 << 0); //Transfer Mode Basic
	//UDMA->USEBURSTSET|=(0x1<<14);//channel 14 responds onnly to burst requests
	//UDMA->REQMASKCLR|=(0x1<<14);
	//UDMA->CHMAP1|=(0x0<<24);
	UDMA->ENASET |= (0x1 << 14); //Enable Chanel
}

void start_ADC(void) {
	ADC0->ACTSS |= (0x01);
	ADC0->PSSI = (0x1 << 0); //inicia secuenciador 0
	//ADC_sampleEnd=0;
}

void stop_ADC(void) {
	int dummie;
	while (!(ADC0->SSFSTAT0 & (0x1 << 8))) dummie = ADC0->SSFIFO0;//read fifo until empty
	ADC0->ACTSS &= ~((int) 0x01); //disable sequencer
	ADC0->DCISC = 0x1; //clear interrupts

}

void config_ADC() {
	/*Configuracion de Pines*/
	SYSCTL->RCGCGPIO |= (0x1 << 4) | (0x1 << 1) | (0x1 << 3);
	SYSCTL->RCGCADC |= 0x01; //habilitamos ADC0
	
	// Puertos Analogicos en E4 E5 B4 B5
	GPIOE->DEN &= ~((0x1 << 4) | (0x1 << 5));
	GPIOB->DEN &= ~((0x1 << 4) | (0x1 << 5));
	GPIOE->AMSEL |= (0x1 << 4) | (0x1 << 5);
	GPIOB->AMSEL |= (0x1 << 4) | (0x1 << 5);
	
	//Configuracion ADC principal
	ADC0->ACTSS &= ~(0xF); //desactivamos secuenciadores durantes la configuracion
	ADC0->EMUX = 0xF; //para trigger por sofware always(continuos sample) 
	//ADC0->CTL|=(0x1<<6);//dither mode enable
	//ADC0->SAC|=(0x3);// x8 averaging
	//Configuracion del sequenciador
	ADC0->SSMUX0 = (0x8) | (0x8 << 4) | (0x8 << 8) | (0x8 << 12) | (0x8 << 16) | (0x8 << 20) | (0x8 << 24) | (0x8 << 28); //(0x8)|(0x9<<4)|(0xA<<8)|(0xB<<12)|(0x8<<16)|(0x9<<20)|(0xA<<24)|(0xB<<28);
	ADC0->SSCTL0 = (0x1 << 30) | (0x1 << 29) | (0x1 << 14); //Final de secuencia en muestra 3, y genera interrupcion
	//Desenmacaramos interrupcion por secuenciador 0
	//ADC0->IM|=(0x1<<0);
	ADC0->ACTSS |= (0x1 << 0); //activamos secuenciador 0
}

void ADC0SS0_Handler(){
	UDMA->CHIS |= (0x1 << 14); //clear interrupts caused by channel 14
	//reloadDMA_DMA_Chanel14();
	dmaTransferComplete = 1;
	DMA_CH14_Transfers++;
	stop_ADC();
}

void adc_capture(void){
	dmaTransferComplete=0;
	reloadDMA_DMA_Chanel14();
	start_ADC();
	while(dmaTransferComplete==0);
	
}

short adc_getData(void){
	return ADC_samples[0];
}

void adc_cofig(){
	config_ADC();
	NVIC_EnableIRQ(ADC0SS0_IRQn);
	config_DMA();
}