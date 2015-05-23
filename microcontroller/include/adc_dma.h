#ifndef _ADC_DMA_H
#define _ADC_DMA_H

void adc_capture(void);
short adc_getData(void);
void adc_cofig(void);

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

#endif
