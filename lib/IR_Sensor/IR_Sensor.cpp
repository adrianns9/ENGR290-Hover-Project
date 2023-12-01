#include "IR_Sensor.h"
#include <avr/io.h>

void setupADC() {
	    ADMUX = (1 << ADLAR) | (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADIF) << (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) ;
    DIDR0 = (1 << ADC0D);
    ADCSRA |= (1 << ADSC);
}

uint8_t readADCH() {
	return ADCH;
}