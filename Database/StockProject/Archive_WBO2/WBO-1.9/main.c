
/////////////---------------------------------///////////////////
// 
// www.waltech.com
//
/////////////---------------------------------/////////////////
/*
 * TO DO:  
 * 	make led do something more?
 */

//Includes here:

#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "pid.h"
#include "stdint.h"
#include "initilize_hardware.h"
#include "dataout.h"
//#define F_CPU 4000000UL
//(in make file)
#define BAUD 9600UL
#define UBRRVAL (F_CPU/(BAUD*16)-1)

#define LOWER_NB 95 //lower value for start of narrow band curve.  
#define MAXPUMP_I 196 // maximum current value for pump (leanest). 
#define MINPUMP_I 50 // minimum current value for pump.  
//NOTE (based on manual pump power settings)
// Above max, virt gnd is pushed up. 
// Nothing happens below min.
// 0 current is at 116
#define TARGET_NERNST 724 // lambda=1 value for nurnst, target for pump pid 
#define TARGET_TEMP 237 
#define P_temp    450
#define I_temp    5
#define D_temp    1

#define P_pump    15
#define I_pump    74
#define D_pump    2

//in pid.h, changed scalevalue to 1 since parampeters are now integers: K * 128

uint8_t heat_power;//global value fed to timer 0 for heater pwm
uint16_t ADC_data;//read the ADC into this

uint16_t nurnst = 0;// value read from adc2 for the nernst cell w/o DC
uint16_t pump = 0;//measured voltage at pump
uint16_t DC_val = 0;//value read from adc2 for the nernst cell w/DC

int16_t IpumpVolts=0;//difference from above proportional to current flowing to pump.
uint16_t measured_temperature;//measured temperature value. also known as Ri
uint32_t zero_to_5_WB;//value applier to timer2 to make DAC output

uint8_t ramp_flag=0;// flag gets set once the startup temperature ramp is done
uint8_t its_off;//flag to keep track of the heater pwm state
volatile uint8_t ADC_flag;//keeps track in the ADC interrupt 
volatile uint8_t charspot=0;//keeps track of the position of the numbers going into the string
volatile uint8_t rampcounter=0;//test couter

////////////////////////////////////////////////////////////////////////
//for PID:
uint8_t pidCounter; //True when PID control loop should run one time
struct PID_DATA pidData_temp; //termperature PID structure of vars
struct PID_DATA pidData_pump; //pump PID structure of vars
////////////////////////////////////////////////////////////////////////



/////function prototypes/////
uint16_t readadc(void);

void do_things(void);
void two(void);
void three(void);
void four(void);
void six_1(void);
void six_2(void);
void six_3(void);
void seven(void);
void eight(void);

void PID_pump(void);
void PID_heater(void);



////Interrupt Service Routines
ISR(ADC_vect)
{
ADC_data = readadc();

if (ADC_flag == 7)//
{
	four();
}	
else if (ADC_flag == 2)
{
	six_1();
}
else if (ADC_flag == 8)
{
	six_2();
}
else if (ADC_flag == 9)
{
	six_3();
}	

else if (ADC_flag == 3)//nurnst
{
	seven();
}
else if (ADC_flag == 4)//nurnst+DC and calculations
{
	eight();
}
	else
	
	{
		ADC_data = readadc();//make sure adc is read to clear 
	}
}
ISR(TIMER0_OVF_vect)
{		
	if (its_off==1)//pulse is off
	 {
		 PORTD |= _BV(5);//turn pin on
		 its_off = 0;
		 uint8_t newtimerval= (255-heat_power);
		 if (newtimerval < 128)
		 {
			do_things();
		 }
		 TCNT0 = newtimerval;	 
	 }
	 else//pulse is on
	 {
		 PORTD &=~ _BV(5);//turn pin off
		 its_off = 1; 
		 uint8_t newtimerval= (heat_power);
		 TCNT0 = newtimerval;//heat_power setting into TCNTO
		 if (newtimerval < 128)
		 {
			do_things();
		 }
		 TCNT0 = (newtimerval);//heat_power setting into TCNTO
	 }

}

//////////////////////////////vvvvvvvvvvvvvvv MAIN  vvvvvvvvvvvvvvvvvvvvv///////////////////////////
int main()
{
//set up all the pins as inputs and outputs
/* 
 * 	PC5 //outputs for R2R DAC/not used
 * 	PC4
 * 	PC3
 *  PB5  
 *  PB4
 * 	PB0
 * 
 *  PD4  LED
 * 
 * Nernst DC connection: PB2
 * nch mosfet for heater: PD5
 * 
 * PB1 = pump power OC1A timer out
 * PB3 = output voltage OC2 timer output
 *   //ADC:
 * nernst V: 	adc2
 * pump			adc0
 *
 */ 
DDRC |= _BV(5)| _BV(4) | _BV(3);//six bit dac
DDRB |= _BV(5)| _BV(4) | _BV(0);//six bit dac
DDRD |= _BV(4);//LED

DDRD |= ( _BV(5));// mosfet for heater
DDRB |= _BV(2);// dc for temperature measurment
////setup uart:////
cli();//  disable interrupts until things are set up
	//init uart
    /* set baud rate */
   	UBRRH = (UBRRVAL >> 8);
   	UBRRL = (UBRRVAL & 0xff);
    /* set frame format: 8 bit, no parity, 1 bit */
    UCSRC |=  (1 << URSEL | 1 << UCSZ1 | 1 << UCSZ0);
    UCSRB |= (1 << RXEN | 1 << TXEN | 1 << RXCIE);//enable receive, transmit, and receive complete interrupt

//disable uart input, avoid Rx buffer overrun:
UCSRB &= ~(1 << RXEN);
UCSRB &= ~(1 << RXCIE);

setup_timer1();// pump control current dac on OC1A  
setup_timer2();//output 0-5v on OC2  
	
pid_Init(P_temp, I_temp, D_temp, &pidData_temp);//set up PID structure for temperature
pid_Init(P_pump, I_pump, D_pump, &pidData_pump);//set up PID structure for nernst
sei();//enable interrupts
adc_init();
// ramp up heat:
heat_power = 130;//initial time
timer0init();
PORTD |= _BV(4);//LED on
uart_putst("ramp temp\n");
while (heat_power<200)
{
heat_power++;
_delay_ms(150);
uart_put16dec(heat_power);
uart_putch(',');
uart_putch(' ');
}
uart_putch('\n');
ramp_flag=1;
///////////////////////
while(1)
	{
//most stuff handled in timer0 interrupt
//Suggested: use a state machine, just read ADC data in the interrupt, and poll a flag to see if it is done? 
	}
return 0;
}
//// end of main
///////////////////////////////////////////////////////// 
//////vvvvvvvvvvvvvv functions vvvvvvvvvvvvvvvvv/////////
/////////////////////////////////////////////////////////

void do_things(void)//do first adc, throw away pump measurement
{  
	if (ramp_flag == 1)
	{
	_delay_us(50);//maybe let things settle?
	ADC_flag = 7;//sets to run function after conversion
	ADMUX =(192 + 0);//V refrence plus mux (pump)			
	ADCSRA |= _BV(ADSC);// starts  conversion
	}	
}
void four(void)//record aux adc 1, mux for pump current
{ 
	ADC_flag = 2;
	ADMUX =(192 + 0);//V refrence plus mux (pump)			
	ADCSRA |= _BV(ADSC);// starts  conversion
}	

void six_1(void)//measures pump current 1/3 sample
{	
	pump = ADC_data;
	ADC_flag = 8;	
	ADCSRA |= (1<<ADSC);// starts  next conversion		
}

void six_2(void)//measures pump current 2/3
{	
	pump = pump + ADC_data;
	ADC_flag = 9;	
	ADCSRA |= (1<<ADSC);// starts  next conversion		
}

void six_3(void)//measures pump current 3/3
{	
	pump = pump + ADC_data;
	pump = pump/3;
	charspot = put_in_string(pump,'\0',charspot);//puts data in big string and sends back new char spot
	ADC_flag = 3;	
	ADMUX =(192 + 2);//V refrence plus mux channel//use 192 for internal 2.5v ref//use 64 for avcc as vref
	ADCSRA |= (1<<ADSC);// starts  next conversion		
}

void seven(void)//measure nurnst 
{
	nurnst = ADC_data;
	charspot = put_in_string(nurnst,'\0',charspot);//puts data in big string
	ADC_flag = 4;
	ADMUX =(192 + 2);//V refrence plus mux channel//use 192 for internal 2.5v ref//use 64 for avcc as vref
	PORTB |= _BV(2);//DC on
	ADCSRA |= (1<<ADSC);// starts  conversion	
}

void eight(void)
{
	PORTB &=~ _BV(2);//back to lo
	DC_val = ADC_data;
//	charspot = put_in_string(DC_val,'d',charspot);//puts data in big string
	ADC_flag = 0;	
	////do calculations and PIDs
	PID_pump();//run PID on pump and update pump pwm.
	//done in PID_pump//charspot = put_in_string(measured_temperature,'f',charspot);
	measured_temperature = (DC_val - nurnst);
	if (measured_temperature <= 255)//make into (8-bit - 10 bit value) and prevent negatives
		{
		measured_temperature = (255 - measured_temperature);
		}
	else
		{
		measured_temperature =0;
		}
	if ( (measured_temperature> (TARGET_TEMP-5))&&(measured_temperature<(TARGET_TEMP+5)) )
		{
		PORTD &=~ _BV(4);//LED off
		}
	else
		{
		PORTD |= _BV(4);//LED on
		}
	charspot = put_in_string(measured_temperature,'\0',charspot);//puts data in big string
	//calculate lambda output from Look Up Table:
	struct two_col{
		uint16_t x;
		uint16_t y;
	}; 
	struct two_col lambda_curve[]={	//table columns: pump, lambda(pump current ADC value, lambda)
		{200,0},
		{260, 68}, 
		{600, 150},
		{621, 200},
		{642, 2500},
		{663, 5000},
		{684, 7500},
		{702, 11000},
		{726, 13500},
		{747, 16000},
		{760, 20200},
		{799, 23000},
		};
	uint8_t n = 12;//number of rows in table	
	uint32_t lambda=0;
	//out of range check:
	if (pump<lambda_curve[0].x)//smaller than the lowest value in LOT
	{lambda = lambda_curve[0].x;}
	else if (pump>lambda_curve[n-1].x)//larger than the highest value in the LOT
	{lambda = lambda_curve[n-1].x;}
	//lookup in table, interpolate
	for( uint8_t i = 0; i < n-1; i++ )//loop through table and find value
	{
		if ( lambda_curve[i].x <= pump && lambda_curve[i+1].x >= pump )
		{
			uint16_t diffx = pump - lambda_curve[i].x; //difference between the pump value and the x value in the LOT
			uint16_t diffn = lambda_curve[i+1].x - lambda_curve[i].x;//spacing between the values in the LOT
			lambda = lambda_curve[i].y + ( lambda_curve[i+1].y - lambda_curve[i].y ) * diffx / diffn; //output value is interpolated.
		}
	}
	
	charspot = put_in_string(lambda,'\0',charspot);//puts data in big string
	zero_to_5_WB = lambda*13/1000UL;
	if(zero_to_5_WB >254){zero_to_5_WB = 255;}//avoid overscale for dac counter
	OCR2 = zero_to_5_WB;// set DAC output
	charspot = put_in_string(zero_to_5_WB,'\0',charspot);//puts data in big string timer counts passed
	

	PID_heater();//run the pid on the temperature and update timer 0
//	charspot = put_in_string(heat_power,'h',charspot);//puts data in big string
	charspot=spitout(charspot);//send it all out the uart
}
	
uint16_t readadc(void)
{
	uint8_t adcDataL = ADCL;
    uint8_t adcDataH = ADCH;
    uint16_t adcData = 0;
    adcData = adcData | adcDataH;
    adcData = adcData << 8;//left shift 8 spots
    adcData = adcData | adcDataL;
    return adcData;
}

void PID_heater(void)
{
	int32_t calculated = pid_Controller(TARGET_TEMP, measured_temperature, &pidData_temp);  // for temp PWM

	if ((calculated) > 255 )
	{
		heat_power = 255 ;
	}
	else if ((calculated) < 0 )
	{
		heat_power = 0 ;
	}
	else
	{
		heat_power =(calculated);
	}

}

void PID_pump(void)
{
int32_t calculated =  ( pid_Controller(TARGET_NERNST, nurnst, &pidData_pump) );  // PID
calculated = (calculated/4) + 116;//116 is zero current. 
if ((calculated) > MAXPUMP_I)
	{
		OCR1A = MAXPUMP_I;
	}
else if ((calculated) < MINPUMP_I)
	{
		OCR1A = MINPUMP_I;
	}
	else
	{
		OCR1A =(calculated);
	}
	charspot = put_in_string(OCR1A,'\0',charspot);//puts data in big string	

}

		