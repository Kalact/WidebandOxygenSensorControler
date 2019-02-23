
/////////////---------------------------------///////////////////
// 
// www.waltech.com
//
/////////////---------------------------------/////////////////
/*
 * ramp disabled
 * set for vref of 1.5v
 * vref is monitored on adc1
 * ramp to stoic
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


////////calibration settings://///////////
//offset for lamda value, uses to calibrate (lambda*100) = 100  at stoic.  Checking against NB sensor
//POSOFFSET is added, NEGOFFSET is subtracted.
#define POSOFFSET 0
#define NEGOFFSET 0

/*Lambda*100 to DAC (0-255)
* 
*	formula for output voltage:
*	Vout = [(L-Lmin)/(Lmax-Lmin)]*5
*
* example to provide lambda range from 0.64 to 1.36:
*	 LAMBDA_MIN	64UL
*	 LAMBDA_MAX 136UL
* Set two values below: 
*/
#define LAMBDA_MIN	64UL// set to desired value for 0v output (Lambda*100)
#define LAMBDA_MAX 136UL//set to desired value for 5v output	(Lambda*100
////////end calibration settings/////////////

#define NUM_BOXES_NORM 2//for boxcar averaging.  Set to 1 for no averaging
#define NUM_BOXES_STOIC 25//for near stoic oscillation mode

#define MAX_BOXES 25 //max boxes stored.

#define DAC_FACTOR 2550000UL/(LAMBDA_MAX-LAMBDA_MIN)//don't change: just a calculation pre-compile

//#define MAXPUMP_I 196 // maximum current value for pump (leanest). 
#define MAXPUMP_I 329 // maximum current value for pump (leanest). 
//see initilize_hardware.h for PWMPUMP_FREQ, if timer OCR1A = PWMPUMP_FREQ, DAC is maxed (5v)
#define ZERO_CURRENT  97
//#define MINPUMP_I 50 // minimum current value for pump. 
 #define MINPUMP_I 0 // minimum current value for pump.
//NOTE (based on manual pump power settings)
// Above max, virt gnd is pushed up. 
// Nothing happens below min.
// 0 current is at 116// now 97 (r4=8.2k ZERO_CURRENT

//#define TARGET_NERNST 832 // lambda=1 value for nurnst, target for pump pid 
#define TARGET_NERNST 821 // lambda=1 value for nurnst, target for pump pid
#define TARGET_TEMP 237 
#define P_temp    350//was 450
#define I_temp    100//5
#define D_temp    0//1

//#define P_pump    15
//#define I_pump    74
//#define D_pump    2

#define P_pump    5//
#define I_pump    2
#define D_pump    0


//#define F_CPU 4000000UL
//(in makefile)
#define BAUD 9600UL
#define UBRRVAL (F_CPU/(BAUD*16)-1)


//test
//for manual ip ramp:
uint32_t IpCount = 0;
uint16_t ipramp = 0;

//for slope determination of nurnst:
#define NURNSTPOINTS 3 //number of points to track
uint16_t nurnst_data[NURNSTPOINTS];

volatile uint16_t box_data[MAX_BOXES];
//in pid.h, changed scalevalue to 1 since parampeters are now integers: K * 128

uint8_t heat_power;//global value fed to timer0 for heater pwm
uint16_t ADC_data;//read the ADC into this

volatile uint16_t nurnst = 0;// value read from adc2 for the nernst cell w/o DC
volatile uint16_t pump = 0;//measured voltage at pump
uint16_t DC_val = 0;//value read from adc2 for the nernst cell w/DC

//int16_t IpumpVolts=0;//difference from above proportional to current flowing to pump.
uint16_t measured_temperature;//measured temperature value. also known as Ri
uint32_t zero_to_5_WB;//value applied to timer2 to make DAC output

uint8_t ramp_flag=0;// flag gets set once the startup temperature ramp is done
uint8_t its_off;//flag to keep track of the heater pwm state
volatile uint8_t ADC_flag;//keeps track in the ADC interrupt 
volatile uint8_t charspot=0;//keeps track of the position of the numbers going into the string

volatile int8_t nspike = 0;//flag for nurnst spike detected
volatile uint8_t near_stoic_flag = 1; //assume near stoic first
volatile uint8_t cycle_counter = 0;
volatile uint8_t cycles_no_flip = 0;//to time speed of nurnst oscillations near stoic
volatile uint8_t flip = 0;//flag indicating transison of nurnst

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


void PID_heater(void);
void PID_pump(uint16_t nurnst_val);
int8_t nurnst_slope_tracker(uint16_t nurnstval);
uint16_t boxcaravg(uint16_t new, uint8_t history);



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

void do_things(void)//do first adc
{  
	if (ramp_flag == 1)// startup temp ramp is finished 
	{
		cycle_counter++;
		if (cycle_counter >3)
		{
			cycle_counter = 0;//reset
			_delay_us(50);//maybe let things settle?
			ADC_flag = 7;//sets to run function after conversion
			ADMUX =(192 + 1);//V refrence plus mux 
			//use 192 for internal 2.5v ref//use 64 for avcc as vref			
			ADCSRA |= _BV(ADSC);// starts  conversion
		}
	}	
}
void four(void)//record aux adc 1, mux for pump current
{ 
	charspot = put_in_string(ADC_data,'\0',charspot);//puts data in big string and sends back new char spot
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
	//charspot = put_in_string(pump,'\0',charspot);//puts data in big string and sends back new char spot
	ADC_flag = 3;	
	ADMUX =(192 + 2);//V refrence plus mux channel//use 192 for internal 2.5v ref//use 64 for avcc as vref
	ADCSRA |= (1<<ADSC);// starts  next conversion		
}

void seven(void)//measure nurnst 
{
	nurnst = ADC_data;
	charspot = put_in_string(nurnst,'\0',charspot);//puts data in big string
	//nspike = nurnst_slope_tracker(nurnst);
	//charspot = put_in_string(near_stoic_flag,'\0',charspot);//puts data in big string
	PID_pump(nurnst);//run PID on pump and update pump pwm.
	ADC_flag = 4;
	ADMUX =(192 + 2);//V refrence plus mux channel//use 192 for internal 2.5v ref//use 64 for avcc as vref
	DDRB |= _BV(2);// dc for temperature measurment
	_delay_us(20);
	PORTB |= _BV(2);//DC on
	ADCSRA |= (1<<ADSC);// starts  conversion	
}

void eight(void)
{
	PORTB &=~ _BV(2);//back to lo
	_delay_us(20);
	DDRB &=~ _BV(2);//hiZ
	DC_val = ADC_data;
//	charspot = put_in_string(DC_val,'d',charspot);//puts data in big string
	ADC_flag = 0;	
	////do calculations and PIDs
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
	
	//pump current filtering:
	charspot = put_in_string(pump,'\0',charspot);//puts data in big string and sends back new char spot
	if (near_stoic_flag == 1)
	{
		pump = boxcaravg(pump,100);//average pump current 100 points
	}
	else
	{
		pump = boxcaravg(pump,3);//average pump current 3 points
	}
	charspot = put_in_string(pump,'\0',charspot);//puts data in big string and sends back new char spot
	
	//calculate lambda output from Look Up Table:
	struct two_col{
		uint16_t x;
		uint16_t y;
	}; 
	struct two_col lambda_curve[]={	//table columns: pump, lambda(x=pump current ADC value, y=lambda)
		{1,0},
		{480,68},
		{570,80},
		{610,85},
		{650,90},
		{690,100},
		{700,110},
		{750,143},
		{795,170},
		{810,242},
		{845,20200},
		{1024,26000},
		};
	uint8_t n = 12;//number of rows in table	
	uint32_t lambda=0;
	//out of range check:
	if (pump<lambda_curve[0].x)//smaller than the lowest value in LOT
	{lambda = lambda_curve[0].y;}
	else if (pump>lambda_curve[n-1].x)//larger than the highest value in the LOT
	{lambda = lambda_curve[n-1].y;}
	//lookup in table, interpolate
	for( uint8_t i = 0; i < n-1; i++ )//loop through table and find value
	{
		if ( (lambda_curve[i].x <= pump )&& (lambda_curve[i+1].x >= pump) )
		{
			uint16_t diffx = pump - lambda_curve[i].x; //difference between the pump value and the x value in the LOT
			uint16_t diffn = lambda_curve[i+1].x - lambda_curve[i].x;//spacing between the values in the LOT
			uint16_t diffy = lambda_curve[i+1].y - lambda_curve[i].y;//spacing of y values in table
			lambda = lambda_curve[i].y + ((diffy * diffx )/diffn); //output value is interpolated.
		}
	}
	lambda = lambda + POSOFFSET;
	lambda = lambda - NEGOFFSET;
	
	charspot = put_in_string(lambda,'\0',charspot);//puts data in big string
	if((lambda>=LAMBDA_MIN) && (lambda<=LAMBDA_MAX))
	{
		uint32_t dacval = (lambda-LAMBDA_MIN)*DAC_FACTOR;
		zero_to_5_WB = dacval/10000UL;
	}
	else if (lambda>LAMBDA_MAX){zero_to_5_WB = 255;}
	else {zero_to_5_WB = 0;}
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

void PID_pump(uint16_t nurnst_val)
{
	int32_t pumpV =  ( pid_Controller(TARGET_NERNST, nurnst, &pidData_pump) );  // PID
	pumpV = pumpV + ZERO_CURRENT;// is zero current.
/*	
	///check if needs to enter near stoic region://///////
	#define HI_NEAR 160// settings for pump V
	#define LO_NEAR 88
	#define ADD_LIMIT 0
	#define MAX_FLIP_COUNTS 6
//	if( (pumpV < (HI_NEAR+ADD_LIMIT))  &&  (pumpV>(LO_NEAR-ADD_LIMIT))  &&  (near_stoic_flag ==2)&&(nspike>-1) )
//	{//pumpV is now below HI_NEAR+ADD_LIMIT, was lean, heading to rich fast.
	if( (pumpV < (HI_NEAR+ADD_LIMIT))  &&  (pumpV>(LO_NEAR-ADD_LIMIT))  )
	{//pumpV is now below HI_NEAR+ADD_LIMIT, was lean, 
		near_stoic_flag =1;//now in near stoic region
		pumpV = LO_NEAR;	//set pump to close to lower limit of near stoic
		cycles_no_flip = 0;//start counter
		flip = 0; // set to rich side
	}
//	if( (pumpV>(LO_NEAR-ADD_LIMIT))  &&  (pumpV<(HI_NEAR+ADD_LIMIT)) && (near_stoic_flag ==0)&&(nspike<1) )
//	{//pumpV is now above LOW_NEAR-ADD_LIMIT,      below hi limit       was rich, heading to lean fast
	if( (pumpV>(LO_NEAR-ADD_LIMIT))  &&  (pumpV<(HI_NEAR+ADD_LIMIT)) )
	{//pumpV is now above LOW_NEAR-ADD_LIMIT,      below hi limit   
		near_stoic_flag =1;//now in near stoic region
		pumpV = HI_NEAR;	//set pump to close to lower limit of near stoic
		cycles_no_flip = 0;//start counter
		flip = 1; // lean side
	}
	
	///if it is in near stoic region
	if (near_stoic_flag ==1)
	{
		//check for flip. If flipped: toggle pumpV and reset flip counter.
		if(  (pumpV ==LO_NEAR)  &&  (flip == 0) && (nurnst < TARGET_NERNST ) )
			 //pumping O2 in		 was rich	    now lean
		{
			pumpV = HI_NEAR;//change pump
			cycles_no_flip = 0;//restart counter
		}
		
		if(  (pumpV ==HI_NEAR)  &&  (flip == 1) && (nurnst > TARGET_NERNST) )
			 //pumping O2 out		 was lean	    now rich
		{
			pumpV = LO_NEAR;//change pump
			cycles_no_flip = 0;//restart counter
		}	
		else{cycles_no_flip++;}//no flip, increment no_flip counter
		
		if (cycles_no_flip >MAX_FLIP_COUNTS)//if flip counter too high, release from near stoic in proper direction
		{
			cycles_no_flip = 0;
			if(pumpV == HI_NEAR) 
			{ 
				pumpV = HI_NEAR+ADD_LIMIT;
				near_stoic_flag = 2; 
			}
			else //pumpV == LO_NEAR
			{ 
				pumpV = LO_NEAR-ADD_LIMIT;
				near_stoic_flag = 0;
			}
			//set near_stoic flag to 0 or 2 (0=rich, 2=lean)
		}
	}
*/	
//apply calculated to pump dac timer
if ((pumpV) > MAXPUMP_I)
	{
		OCR1A = MAXPUMP_I;
	}
else if ((pumpV) < MINPUMP_I)
	{
		OCR1A = MINPUMP_I;
	}
	else
	{
		OCR1A =(pumpV);
	}
	charspot = put_in_string(OCR1A,'\0',charspot);//puts data in big string	
}

int8_t nurnst_slope_tracker(uint16_t nurnstval)		
{
	//running array: shift points, in latest value
	for (uint8_t i = 0; i<(NURNSTPOINTS-1); i++)//shift points
	{
		nurnst_data[i]=nurnst_data[i+1];
	}
	nurnst_data[NURNSTPOINTS-1] = nurnstval;//put in latest value
	//calculate sums for least squares:
	int8_t SUMx = 0; 
	int16_t SUMy =0;
	int16_t SUMxy = 0; 
	int16_t SUMxx  = 0;
	for (uint8_t i = 0; i<(NURNSTPOINTS); i++)//SUMx
	{
		SUMx = SUMx+i;
		SUMy = SUMy+nurnst_data[i];
		SUMxy = SUMxy+(i*nurnst_data[i]);
		SUMxx = SUMxx+(i*i);
	}
	int16_t slope = ((SUMx*SUMy)- NURNSTPOINTS*SUMxy) / ( (SUMx*SUMx) - NURNSTPOINTS*SUMxx);
	return (slope);	
}

uint16_t boxcaravg(uint16_t new, uint8_t history)
{
	for (uint8_t i = 0; i<(MAX_BOXES-1); i++)//shift boxcars
		{box_data[i]=box_data[i+1];}
	box_data[MAX_BOXES-1] = new;//put in latest value
	//add up 'history' number of boxcars:
	uint32_t avrg_data_tot = 0; 
	for (uint8_t i = MAX_BOXES-history ; i<MAX_BOXES; i++)
	{
	avrg_data_tot = avrg_data_tot+box_data[i];
	}
	return(avrg_data_tot/history);//return averaged value
}