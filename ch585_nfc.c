#include "ch32fun.h"
#include "NFCA_LIB/CH58x_NFCA_LIB.h"
#include <stdio.h>

#define LED PA8


void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, FUN_LOW ); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite( LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(33);
	}
}

int main()
{
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx

	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );

	printf("~ ch585 NFC ~\n");
	blink(5);

	while(1);
}
