#include <avr/io.h>
#include <util/delay.h>
#include "os.h"
#include "utils.h"

/* CONSTANT DEFINITIONS */

// CONSTANT BYTE TO NUMBER CONVENTIONS
//servo1
const unsigned int servo1left = 0;
const unsigned int servo1right = 1;
const unsigned int servo1stopped = 2;

//servo2
const unsigned int servo2left = 16;
const unsigned int servo2right = 17;
const unsigned int servo2stopped = 18;

//Roomba
const unsigned int roomba2for = 32;
const unsigned int roomba2back = 33;
const unsigned int roomba2stop = 34;
const unsigned int roomba2forfast = 35;
const unsigned int roomba2backfast = 36;

const unsigned int roomba1for = 48;
const unsigned int roomba1back = 49;
const unsigned int roomba1stop = 50;
const unsigned int roomba1forfast = 51;
const unsigned int roomba1backfast = 52;

//laser
const unsigned int firelaser = 64;
const unsigned int offlaser = 65;

// GENERAL DIRECTION CONSTANTS
const int backward = -1;
const int forward = 1;
const int stopped = 0;
const int right = 2;
const int left = 3;
const int backward_fast = 4;
const int forward_fast = 5;
const int right_fast = 6;
const int left_fast = 7;

// CONSTANT PIN ASSIGNATIONS
#define laser_button_pin PF0   // analog pin 0
#define servo_1_pin PF1   // analog pin 1
#define servo_2_pin PF2   // analog pin 2
#define roomba_1_pin PF3   // analog pin 3 // pin # moves roomba
#define roomba_2_pin PF4   // analog pin 4 // pin # moves roomba
#define roomba_button_pin PF5   // analog pin 5 


// GLOBAL STATUS VARIABLES
volatile BOOL firing_laser = FALSE; // global variable for current state of laser
volatile int servo_1_dir = 0; // -1 (backward), 0 (static), 1 (forward)
volatile int servo_2_dir = 0; // -1 (backward), 0 (static), 1 (forward)
volatile int roomba_dir1 = 0; // -1 (backward), 0 (static), 1 (forward)
volatile int roomba_dir2 = 0; // -1 (backward), 0 (static), 1 (forward)

const unsigned int left_on = 80;
const unsigned int right_on = 81;
const unsigned int forward_on = 82;
const unsigned int backward_on = 83;
const unsigned int stopped_on = 84;

const unsigned int left_off = 112;
const unsigned int right_off = 113;
const unsigned int forward_off = 114;
const unsigned int backward_off = 115;
const unsigned int stopped_off = 116;

/* CONSTANT DEFINITIONS END */

void signal_fire_laser() {
	uart_putchar((char)firelaser);
}

void signal_turnoff_laser() {
	uart_putchar((char)offlaser);
}

// task function for joystick1 task
void joystick1_task()
{
	for (;;) {
	
		int sensorValue1 = read_ADC(servo_1_pin);
		sensorValue1 = map(sensorValue1, 0, 260, 0, 2); //input signal for controlling myServo
		int sensorValue2 = read_ADC(servo_2_pin);
		sensorValue2 = map(sensorValue2, 0, 260, 0, 2); //input signal for controllers myServo2

		// really maps from 7 to 0
		
		if (sensorValue1 < 3 && sensorValue1 >= 0 && servo_1_dir != backward) {
			servo_1_dir = backward;
			uart_putchar((char)servo1left);
		} else if (sensorValue1 < 8 && sensorValue1 > 4 && servo_1_dir != forward) {
			servo_1_dir = forward;
			uart_putchar((char)servo1right);
		} else if((sensorValue1 == 3 || sensorValue1 == 4) && servo_1_dir != stopped) {
			servo_1_dir = stopped;
			uart_putchar((char)servo1stopped);
		}
		
		if (sensorValue2 < 3 && sensorValue2 >= 0 && servo_2_dir != backward) {
			servo_2_dir = backward;
			uart_putchar((char)servo2left);
		} else if (sensorValue2 < 8 && sensorValue2 > 4 && servo_2_dir != forward) {
			servo_2_dir = forward;
			uart_putchar((char)servo2right);
		} else if((sensorValue2 == 3 || sensorValue2 == 4) && servo_2_dir != stopped) {
			servo_2_dir = stopped;
			uart_putchar((char)servo2stopped);
		}
		
		int laser_value = (read_ADC(laser_button_pin) < 10) ? 1 : 0 ;

		if (laser_value == 1 && !firing_laser) {
			firing_laser = TRUE;
			signal_fire_laser();
		} else if(laser_value == 0 && firing_laser){
			firing_laser = FALSE;
			signal_turnoff_laser();
		}

		if (firing_laser == TRUE) {
			PORTB |= (1<<PB3);
		} else {
			PORTB &= ~(1<<PB3);
		}
		Task_Next();
	}
}

// task function for joystick2 task
void joystick2_task()
{
	for (;;) {

		int sensorValue3 = read_ADC(roomba_1_pin);
		sensorValue3 = map(sensorValue3, 0, 1024, 0, 7); //input signal for controlling roomba1
		int sensorValue4 = read_ADC(roomba_2_pin);
		sensorValue4 = map(sensorValue4, 0, 1024, 0, 7); //input signal for controllers roomba2

		if (sensorValue3 < 3 && sensorValue3 >= 0 && roomba_dir1 != backward) {
			roomba_dir1 = backward;
			//uart_putchar((char)roomba1back);
		} else if (sensorValue3 < 8 && sensorValue3 > 4 && roomba_dir1 != forward) {
			roomba_dir1 = forward;
			//uart_putchar((char)roomba1for);
		} else if((sensorValue3 == 3 || sensorValue3 == 4) && roomba_dir1 != stopped) {
			roomba_dir1 = stopped;
			uart_putchar((char)roomba1stop);
		}
		
		if (sensorValue4 < 3 && sensorValue4 >= 0 && roomba_dir2 != backward) {
			roomba_dir2 = backward;
			uart_putchar((char)roomba2back);
		} else if (sensorValue4 < 8 && sensorValue4 > 4 && roomba_dir2 != forward) {
			roomba_dir2 = forward;
			uart_putchar((char)roomba2for);
		} else if((sensorValue4 == 3 || sensorValue4 == 4) && roomba_dir2 != stopped) {
			roomba_dir2 = stopped;
			uart_putchar((char)roomba2stop);
		}

		Task_Next();
	}
}


void a_main()
{
DDRB |= (1<<PB1);	//pin 52
DDRB |= (1<<PB2);	//pin 51
DDRB |= (1<<PB3);	//pin 50
PORTB |= (1<<PB3);
PORTB &= ~(1<<PB3);
PORTB |= (1<<PB3);
PORTB &= ~(1<<PB3);
//uart_putchar((char)35);
	// Initialize components

	uart_init();
	init_ADC();
	mode_PORTA_INPUT(laser_button_pin);
	mode_PORTA_INPUT(servo_1_pin);
	mode_PORTA_INPUT(servo_2_pin);
	mode_PORTA_INPUT(roomba_1_pin);
	mode_PORTA_INPUT(roomba_button_pin);
	DDRA = 0xFF;

	Task_Create_Period(joystick1_task, 0, 4, 3, 0);
	Task_Create_Period(joystick2_task, 0, 4, 3, 4);

	//Task_Create_System(joystick1_task, 0);
	//Task_Create_System(joystick2_task, 0);

	Task_Terminate();
}
