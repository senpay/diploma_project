/*
 * Copyright (c) 2014 Alexander Pushkarev.
* All rights reserved.

* Redistribution and use in source and binary forms are permitted
* provided that the above copyright notice and this paragraph are
* duplicated in all such forms and that any documentation,
* advertising materials, and other materials related to such
* distribution and use acknowledge that the software was developed
* by the Alexander Pushkarev.  The name of the
* Alexander Pushkarev may not be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include  "msp430g2452.h"
#include <stdio.h>

#define TRUE 1
#define FALSE 0

#define TEMP_LOW 100
#define TEMP_HIGH 210

#define LCM_DIR P2DIR
#define LCM_OUT P2OUT
#define RS  BIT0
#define EN  BIT3
#define D4  BIT4
#define D5  BIT5
#define D6  BIT6
#define D7  BIT7
#define LCD_MASK    (RS + EN + D4 + D5 + D6 + D7)

#define RELAY_DIR P2DIR
#define RELAY_OUT P2OUT
#define R1 BIT1
#define R2 BIT2
#define RELAY_MASK    (R1 + R2)

#define P1_MASK (BIT0 + BIT1 +BIT2 + BIT4 + BIT5 + BIT6 + BIT7)

//amplification factor
#define AMPL_FACTOR 100


// temp/volts ratio
#define VOLTS {6.95, 9.93, 11.46, 13.03, 14.66, 16.3, 17.95, 21.25, 22.91}
#define RATIO {14.38, 14.1, 13.95, 13.81, 13.64, 13.49, 13.37, 13.15, 13.09}
#define VOLTS_NUMBER 9

#define EMPTY_STRING "                "

void initADC(void);
void initOutput(void);
void initLCD(void);
unsigned short getAverageTemp(unsigned short);
unsigned short getTemp(void);
void turnRelayOn(unsigned char);
void turnRelayOff(unsigned char);
void initMCU(void);
void printToFirstLine(char[]);
void printToSecondLine(char[]);
void clearFirstLine(void);
void clearSecondLine(void);
void clearLCD(void);
void printRelayStatus(int, int);
unsigned short toogleTemperature();
void moveCursor(char, char);
void sendByte(char, char);
void pulse(void);
void printStr(char*);


/*
 * main.c
 */
void main(void) {
	// Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

	unsigned short temperature = 0;

    initMCU();
    initLCD();
    initADC();
    initOutput();
    printRelayStatus(0, 0);

    while(TRUE) {
    	while(temperature < TEMP_LOW) {
    		temperature = toogleTemperature();
    	}

    	turnRelayOn(1);
    	printRelayStatus(1, 0);

    	//Waits for five seconds (approximately), updating temperature
    	unsigned short i;
    	for (i = 10; i > 0; i--) {
    		toogleTemperature();
    	}

    	turnRelayOn(2);
    	printRelayStatus(1, 1);

    	while(temperature < TEMP_HIGH) {
    		temperature = toogleTemperature();
    	}

    	turnRelayOff(2);
    	printRelayStatus(1, 0);

    	while(temperature > TEMP_LOW) {
    		temperature = toogleTemperature();
    	}
    }

}


/*
 * Inits ADC10, P1.X ports.
 */
void initADC(void) {
	//Inits ADC
	/* ADC10CTL0 - ADC10, Control Register 0
	* ADC10SHT_0 - sample and hold time 4 x ADC10CLKs
	* ADC10ON - ADC10 on
	* MSC - Mode of multiple sampling and conversion
	* REF2_5V - Reference-generator voltage (2.5V). REFON must also be set
	* REFON - Turns reference-generator on
	* ENC - Allows ADC10 conversion (should be set last, as it blocks all other
	* changes)
	*/
	ADC10CTL0 = ADC10SHT_1 + ADC10ON + MSC + REF2_5V + REFON;
	//Enabling analog input on port P1.3
	ADC10AE0 = BIT3;

	/* ADC10CTL0 - ADC10, Control Register 1
	* INCH_3 - Use channel 3 for conversion
	* ADC10DIV_0 - No clock division needed
	* SHS - Use ADC10SC for starting conversion
	* CONSEQ_0 - single channel single conversions
	*  (will restart conversion manually using ADC10SC of ADC10CTL0)
	* ADC10SSEL_2 - Turns reference-generator on
	* ADC10SSEL_0 - Use ADC10SC as conversion clocks
	*/
	ADC10CTL1 = INCH_3 + ADC10DIV_0 + SHS_0 + CONSEQ_0 + ADC10SSEL_0;

	//configure all other P1 pins as inputs, pull-down enabled
	P1DIR &= ~P1_MASK;
	P1REN &= ~P1_MASK;
	P1OUT &= ~P1_MASK;

}


/*
 * Inits output for relays
 */
void initOutput(void) {
	RELAY_DIR |= RELAY_MASK;
	RELAY_OUT &= ~RELAY_MASK;
}


/*
 * Inits LCD
 */
void initLCD(void) {
	//Initing LCD in 4-bit mode
	LCM_DIR |= LCD_MASK;
	LCM_OUT &= ~LCD_MASK;

	__delay_cycles(200000);

	LCM_OUT = 0x20;
	pulse();

	sendByte(0x28, FALSE);
	sendByte(0x0C, FALSE);
	sendByte(0x06, FALSE);

	Write hello message
	printToFirstLine("     BSUIR      ");
	printToSecondLine("      ETT       ");
	__delay_cycles(2500000);
	clearLCD();
	printToFirstLine("(c) A. Pushkarev");
	__delay_cycles(2500000);
	clearLCD();
}


/*
 * Gets average temperature measured specified times with timeout of 1ms
 */
unsigned short getAverageTemp(unsigned short number) {
	float temperature = 0;

	unsigned short i;
	for (i = 0; i < number; i++) {
		temperature += ((float) getTemp())/number;
		__delay_cycles(1000);
	}

	return (unsigned short) temperature;

}


/*
 * Gets temperature from ADC
 */
unsigned short getTemp(void) {
	unsigned short temperature = 0;
	float measured_volt = 0;
	float real_volt = 0;
	const float volts[VOLTS_NUMBER] = VOLTS;
	const float ratio[VOLTS_NUMBER] = RATIO;
	float current_ratio = ratio[0];

	//Enabling new ADC conversion
	ADC10CTL0 |= ENC;

	//Starting new ADC conversion
	ADC10CTL0 |= ADC10SC;

	//Wait for conversion to end
	while (ADC10CTL1 & ADC10BUSY) {
	}

	//Getting raw conversion results
	// 0.002444 ~ conversion rate (2.5 / 1023)
	measured_volt = ADC10MEM * 0.002444;

	//Calculating real volt (in millivolts)
	real_volt = measured_volt / AMPL_FACTOR * 1000;

	//get closest ratio
	unsigned int i;
	for (i = 1; i < VOLTS_NUMBER; i++) {
		//if specified volt range is close enough - use specified ratio
		if (real_volt < volts[i]) {
			break;
		}
		current_ratio = ratio[i];
	}

	//Calculating final temperature
	temperature = real_volt * current_ratio;

	//Disabling new ADC conversion
	ADC10CTL0 &= ~ENC;

	return temperature;
}


/*
 * Turns on Relay.1 or Relay.2 depending on values passed
 * (1 - turns Relay.1, 2 - Relay.2. Invalid values ignored)
 */
void turnRelayOn(unsigned char relay) {
	if (relay == 1) {
		RELAY_OUT |= R1;
	} else if (relay == 2) {
		RELAY_OUT |= R2;
	}
}


/*
 * Turns off Relay.1 or Relay.2 depending on values passed
 * (1 - turns Relay.1, 2 - Relay.2. Invalid values ignored)
 */
void turnRelayOff(unsigned char relay) {
	if (relay == 1) {
		RELAY_OUT &= ~R1;
	} else if (relay == 2) {
		RELAY_OUT &= ~R2;
	}
}


/*
 * Inits MCU's CPU in proper speed (1MHz)
 */
void initMCU(void) {
	//Selecting P2.6 and P2.7 for digital I/O
	P2SEL &= ~BIT6;
	P2SEL &= ~BIT7;

    //Set 1MHZ CPU frequency
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;
}


/*
 * Prints message to first line of LCD
 */
void printToFirstLine(char message[]) {
	moveCursor(0, 0);
	printStr(message);
}


/*
 * Prints message to second line of LCD
 */
void printToSecondLine(char message[]) {
	moveCursor(1, 0);
	printStr(message);
}

/*
* Clears first line on LCD.
* As different displays might have different clear codes,
* it writes two empty lines to clear display visually.
*/
void clearFirstLine() {
	printToFirstLine(EMPTY_STRING);
}

/*
* Clears second line on LCD.
* As different displays might have different clear codes,
* it writes two empty lines to clear display visually.
*/
void clearSecondLine() {
	printToSecondLine(EMPTY_STRING);
}

/*
 * Clears screen on LCD.
 * As different displays might have different clear codes,
 * it writes two empty lines to clear display visually.
 */
void clearLCD() {
	clearFirstLine();
	clearSecondLine();
}


void printRelayStatus(int ka1, int ka2) {
	char message[16];

	sprintf(message, "KA1: %d, KA2: %d", ka1, ka2);
	printToSecondLine(message);
}


/*
 * Gets measured temperature and writes it to LCD. Waits ~ 500ms.
 * Returns measured temperature.
 */
unsigned short toogleTemperature() {
	unsigned short temperature;
	char message[16];

	temperature = getAverageTemp(20);

	if(temperature >= 300) {
		sprintf(message, "T = >300 C ");
	} else {
		sprintf(message, "T = %d C ", temperature);
	}

	printToFirstLine(message);

	//almost second - less then a half second to fix overhead
	//given by ADC measures
	__delay_cycles(479000);

	return temperature;
}


/*
 * Moves cursor of LCD to specified coordinates
 */
void moveCursor(char Row, char Col) {
	char address;

	if (Row == 0) {
		address = 0;
	} else {
		address = 0x40;
	}

	address |= Col;
	sendByte(0x80 | address, FALSE);
}

/*
 * Sends byte to LCD using 4-bit interface
 */
void sendByte(char byte, char is_data) {
	LCM_OUT &= ~LCD_MASK;
	LCM_OUT |= (byte & 0xF0);

	if (is_data == TRUE) {
		LCM_OUT |= RS;
	} else {
		LCM_OUT &= ~RS;
	}
	pulse();

	LCM_OUT &= ~LCD_MASK;
	LCM_OUT |= ((byte & 0x0F) << 4);

	if (is_data == TRUE) {
		LCM_OUT |= RS;
	} else {
		LCM_OUT &= ~RS;
	}
	pulse();
}


/*
 * Sends control signal to LCD
 */
void pulse(void) {
	LCM_OUT |= EN;
	__delay_cycles(400);

	LCM_OUT &= ~EN;
	__delay_cycles(400);
}


/*
 * Prints send text on LCD. Coordinates must be selected before print
 */
void printStr(char *str) {
	char *c;
	c = str;
	while ((c != 0) && (*c != 0)) {
		sendByte(*c, TRUE);
		c++;
	}
}
