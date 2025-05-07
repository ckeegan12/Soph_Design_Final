#include <stdint.h>
#include <stdio.h>
#include <xil_printf.h>
#include <stdbool.h>

// Hardware Addresses
#define BUTTONS 	(* (unsigned volatile *) 0x40000000) // 4  pin In		btnU, btnL, btnR, btnD
#define JA 			(* (unsigned volatile *) 0x40001000) // 8  pin In/Out	JA[7:0]
#define JA_DDR 		(* (unsigned volatile *) 0x40001004) // 8  pin DDR,		1 means input, 0 means output
#define JB 			(* (unsigned volatile *) 0x40002000) // 8  pin In/Out	JB[7:0]
#define JB_DDR 		(* (unsigned volatile *) 0x40002004) // 8  pin DDR		1 means input, 0 means output
#define JC 			(* (unsigned volatile *) 0x40003000) // 8  pin In/Out	JC[7:0]
#define JC_DDR 		(* (unsigned volatile *) 0x40003004) // 8  pin DDR,		1 means input, 0 means output
#define JXADC 		(* (unsigned volatile *) 0x40004000) // 8  pin In/Out	JXADC[7:0]
#define JXADC_DDR 	(* (unsigned volatile *) 0x40004004) // 8  pin DDR		1 means input, 0 means output
#define LEDS 		(* (unsigned volatile *) 0x40005000) // 16 pin Out		led[15:0]
#define ANODES 		(* (unsigned volatile *) 0x40006000) // 4  pin Out		an[3:0]
#define SEVEN_SEG 	(* (unsigned volatile *) 0x40006008) // 8  pin Out		dp, seg[6:0]
#define SWITCHES 	(* (unsigned volatile *) 0x40007000) // 16 pin In		sw[15:0]
#define UART 		(* (unsigned volatile *) 0x40008000) // Out of Scope
#define TIMER_0     (* (unsigned volatile *) 0x40009000)
#define TIMER_1     (* (unsigned volatile *) 0x4000A000)
#define TIMER_2     (* (unsigned volatile *) 0x4000B000)
#define TIMER_3     (* (unsigned volatile *) 0x4000C000)

// Memory Access Offsets - Buttons
#define BTND_OFFSET 0 // BTN[0]
#define BTNR_OFFSET 1 // BTN[1]
#define BTNL_OFFSET 2 // BTN[2]
#define BTNU_OFFSET 3 // BTN[3]

// Memory Access Offsets - Motors
#define L_PWM_OFFSET  0  // JC[0]
#define LEFT1_OFFSET  1  // JC[1]
#define LEFT2_OFFSET  2  // JC[2]
#define R_PWM_OFFSET  3  // JC[3]
#define RIGHT2_OFFSET 4  // JC[4]
#define RIGHT1_OFFSET 5  // JC[5]

// Memory Access Offsets - Quad Encs
#define L1_QUAD_ENC_OFFSET 0 // JA[0]
#define R1_QUAD_ENC_OFFSET 1 // JA[1]

// Constants
#define INCREMENT 5
#define ITP (uint32_t *)

// Motor
#define L_PWM_OFFSET 0 // JC[0]
#define LEFT1_OFFSET 1 // JC[1]
#define LEFT2_OFFSET 2 // JC[2]
#define R_PWM_OFFSET 3 // JC[3]
#define RIGHT2_OFFSET 4 // JC[4]
#define RIGHT1_OFFSET 5 // JC[5]
#define DUTY_MOTION_START_RIGHT 150
#define DUTY_MOTION_START_LEFT 160
#define DUTY_MOTION_START 200
#define PWM_TOP 255

// Quad Encoder
#define L1_QUAD_ENC_OFFSET 0 // JA[0]
#define R1_QUAD_ENC_OFFSET 1 // JA[1]
#define QUAD_ENC_TOP 10000

// Sensor
#define TRIG1_BIT 0
#define ECHO1_BIT 1
#define TRIG2_BIT 2
#define ECHO2_BIT 3
#define TCSR0     (*(unsigned volatile *) 0x40009000)
#define TCR0      (*(unsigned volatile *) 0x40009008)
#define TCSR1     (*(unsigned volatile *) 0x40009010)
#define TCR1      (*(unsigned volatile *) 0x40009018)
#define TIMEOUT   10000000

// Other
const volatile uint32_t TCSR_OFFSET = 0;
const volatile uint32_t TLR_OFFEST = 1;
const volatile uint32_t TCR_OFFSET = 2;

// Addresses for all the timers, ITP casts the int to uint32_t ptr
const uint32_t * TIMERS[] = { ITP 0x40009000, ITP 0x40009100, ITP 0x4000A000, ITP 0x4000A100,
                              ITP 0x4000B000, ITP 0x4000B100, ITP 0x4000C000, ITP 0x4000C100};

// Function declarations - implemented below
void init_program();
void show_sseg(uint8_t * sevenSegValue);
void count_display(uint8_t count);
_Bool UpButton_pressed();
_Bool DownButton_pressed();
_Bool LeftButton_pressed();
_Bool RightButton_pressed();
uint32_t * convert_timer_to_hex_address (uint8_t timer_number);

// Motor Functions
uint32_t read_L1_quad_enc(_Bool reset);
uint32_t read_R1_quad_enc(_Bool reset);
void Stop_motors();
void Pass_object();

// Drive Direction
void Turn_left();
void Turn_right();
void drive_straight(uint8_t left_distance);

// Detection Functions
// sensor1 = front sensor | sensor2 = left sensor
void set_trig1_pin();  
void set_trig2_pin();
void clear_trig1_pin();
void clear_trig2_pin();
_Bool read_echo1_pin();
_Bool read_echo2_pin();
uint8_t Sensor1_distance();
uint8_t Sensor2_distance();

// Timers
uint32_t read_stopwatch(uint8_t timer_number);
void timer_2us(unsigned t);
void configure_timers();
void start_stopwatch(uint8_t timer_number);
void timer_2us(unsigned t);
_Bool delay_1s();
_Bool delay_half_sec();

int get_Sensor1_distance();






int main (void){

    // One time initializations
    init_program();
    JA_DDR = 0x03;
    JC_DDR = 0x00;
    JB_DDR = 0x0A;

    // // State Enumerations for loop
    // enum state_type {left_object, left_front_object, passby_object, check_end, end};
    // static enum state_type state = left_object;
    // enum state_type next_state = state;
    // enum state_type last_state = state;

    // // Navigating parameters
    // uint32_t front_threshold = 3; // Distance threshold to turn after object in front
    // uint32_t left_threshold = 10; // Distance threshold to turn after passing object
    // uint8_t count = 0; // Sensor count

    while (1) {
    // uint8_t front_distance = Sensor1_distance();
    // xil_printf("Distance: %d\r\n", front_distance);
    // if (front_distance < 6){
    //     Turn_right();
    // }
    // timer_2us(100000); // 200 ms between pings

    uint8_t left_distance = Sensor2_distance();
    xil_printf("Distance: %d\r\n", left_distance);
    if (left_distance > 6){
        Turn_right();
    }
    timer_2us(100000); // 200 ms between pings
}


    // Navigating loop
    // while(1){

    //     // Sensor distance parameters
    //     // uint8_t front_distance = Sensor1_distance();
    //     // uint8_t left_distance = Sensor2_distance();    

    



    //     // Navigating state
    //     switch(state){
    //         case(left_object):
    //             // Only wall on the left of robot
    //             if(left_distance > left_threshold) next_state = passby_object;
    //             else {
    //                 drive_straight(left_distance);
    //                 next_state = left_object;
    //                 if(front_distance > front_threshold){
    //                     next_state = left_front_object; // front object encountered
    //                 }
    //             }
    //             last_state = left_object;
            
    //         case(left_front_object):
    //         // Object on left and in front of robot
    //             Stop_motors();
    //             timer_2us(500000);
    //             Turn_right();            
    //             next_state = check_end;
    //             last_state = left_front_object;
            
    //         case(passby_object):
    //             // passing object 
    //             //Pass_object();
    //             next_state = left_object;
    //             last_state = passby_object;
            
    //         case(check_end):
    //             // checking if end condition is met
    //             if(last_state == left_front_object){
    //                 next_state = end;
    //             }
    //             else next_state = left_object;
    //         case(end):
    //             // end of maze
    //             Stop_motors();
    //             next_state = end;
    //     }
    //     // Go to next location state
    //     state = next_state;
        
    // }

}


// Function Implementation - Initialization
void init_program(){
    configure_timers();
}

// Function implementation - Hardware Timers
uint32_t * convert_timer_to_hex_address (uint8_t timer_number) {
    if (timer_number > 7)
        return ITP (TIMERS[0]);
    else
        return ITP (TIMERS[timer_number]);
}

void configure_timers (){
    for (int i =  0; i < 8; i++) {
        uint32_t * timer_base_address = convert_timer_to_hex_address(i);
        uint32_t * tcr = timer_base_address + TCR_OFFSET;
        uint32_t * tcsr = timer_base_address + TCSR_OFFSET;
        *(tcr) = 0x00000000;
        *(tcsr) = 0b010010010001;
    }
}

void start_stopwatch(uint8_t timer_number){
    if (timer_number > 7){
        return;
}
    uint32_t * timer_base_address = convert_timer_to_hex_address(timer_number);
    volatile uint32_t * tcsr = timer_base_address + TCSR_OFFSET;

    *tcsr &= ~(1<<7);
    *tcsr |= 1<<5;
    *tcsr &= ~(1<<5);
    *tcsr |= 1<<7;
}

uint32_t read_stopwatch(uint8_t timer_number){
    if (timer_number > 7){
        return 0;
}
    uint32_t * timer_base_address = convert_timer_to_hex_address(timer_number);
    volatile uint32_t * tcr = timer_base_address + TCR_OFFSET;
    return (*tcr) / 100;
}

// Function implementation - SSeg
void show_sseg(uint8_t * sevenSegValue){
    static uint8_t anodeCnt = 0;
    if(read_stopwatch(1)>1000){
        anodeCnt++;
        ANODES = ~(1 << (anodeCnt % 4));
        SEVEN_SEG = sevenSegValue[anodeCnt % 4];
        start_stopwatch(1);
    }
}

// Function Implementation - Software Delays
_Bool delay_1s(){
    const uint32_t TOP = 2870000;
    static uint32_t count = 0;
    if (count == TOP){
        count = 0;
        return true;
    }
    count++;
    return false;
}

_Bool delay_half_sec(){
    const uint32_t TOP = 1350000;
    static uint32_t count = 0;
    if (count == TOP){
        count = 0;
        return true;
    }
    count++;
    return false;
}




void Turn_left(){
    // Reset motor direction and PWM pins
    JC &= ~((1 << L_PWM_OFFSET) | (1 << R_PWM_OFFSET) | (1 << LEFT1_OFFSET) | (1 << LEFT2_OFFSET) | (1 << RIGHT1_OFFSET) | (1 << RIGHT2_OFFSET));

  
    _Bool EndR = 0;
    _Bool EndL = 0;    
    uint16_t pwmCnt = 0;
    read_R1_quad_enc(1);
    read_L1_quad_enc(1);

    while(!(EndL & EndR)){
        JC |= (1<<RIGHT1_OFFSET) | (1<<LEFT1_OFFSET);
        JC &= ~(1<<RIGHT2_OFFSET) & ~(1<<LEFT2_OFFSET);
        
        if (pwmCnt <= DUTY_MOTION_START)
            JC |= (1 << L_PWM_OFFSET);
        else
            JC &= ~(1 << L_PWM_OFFSET);
        if (pwmCnt <= DUTY_MOTION_START)
            JC |= (1 << R_PWM_OFFSET);
        else 
            JC &= ~(1 << R_PWM_OFFSET);
        if (pwmCnt++ == PWM_TOP)
            pwmCnt = 0;
        if (!(read_R1_quad_enc(0) < 250) ){
            JC &= ~(1 << R_PWM_OFFSET);
            EndR = 1;
        }
        if (!(read_L1_quad_enc(0) < 170)){
            JC &= ~(1 << L_PWM_OFFSET);
            EndL = 1;
        }
    }
}

void Turn_right(){
    JC &= ~((1 << L_PWM_OFFSET) | (1 << R_PWM_OFFSET) | (1 << LEFT1_OFFSET) | (1 << LEFT2_OFFSET) | (1 << RIGHT1_OFFSET) | (1 << RIGHT2_OFFSET));

    _Bool EndL = 0;
    _Bool EndR = 0;
    uint16_t pwmCnt = 0;
    read_R1_quad_enc(1);
    read_L1_quad_enc(1);

    while(!(EndL & EndR)){
        JC |= (1<<RIGHT2_OFFSET) | (1<<LEFT2_OFFSET);
        JC &= ~(1<<RIGHT1_OFFSET) & ~(1<<LEFT1_OFFSET);
        
        if (pwmCnt <= DUTY_MOTION_START)
            JC |= (1 << L_PWM_OFFSET);
        else
            JC &= ~(1 << L_PWM_OFFSET);
        if (pwmCnt <= DUTY_MOTION_START)
            JC |= (1 << R_PWM_OFFSET);
        else 
            JC &= ~(1 << R_PWM_OFFSET);
        if (pwmCnt++ == PWM_TOP)
            pwmCnt = 0;
        if (!(read_R1_quad_enc(0) < 170) ){
            JC &= ~(1 << R_PWM_OFFSET);
            EndR = 1;
        }
        if (!(read_L1_quad_enc(0) < 250)){
            JC &= ~(1 << L_PWM_OFFSET);
            EndL = 1;
        }
    }
}


void drive_straight(uint8_t left_distance) {
    // Motor duty cycle update function to drive straight
    // JC Motor pin mapping
    JC &= ~((1 << L_PWM_OFFSET) | (1 << R_PWM_OFFSET) | 
            (1 << LEFT1_OFFSET) | (1 << LEFT2_OFFSET) | 
            (1 << RIGHT1_OFFSET) | (1 << RIGHT2_OFFSET));

    // Duty cycle parameter
    uint16_t pwmCnt = 0;

    // Initial calibrated offset — favoring slightly slower right motor
    int16_t duty_left = DUTY_MOTION_START_LEFT;
    int16_t duty_right = DUTY_MOTION_START_RIGHT;

    read_R1_quad_enc(1);
    read_L1_quad_enc(1);

    JC |= (1 << LEFT1_OFFSET) | (1 << RIGHT2_OFFSET);
    JC &= ~((1 << LEFT2_OFFSET) | (1 << RIGHT1_OFFSET));

    // Duty cycle update loop
    while (1) {

        if (pwmCnt <= duty_left)
            JC |= (1 << L_PWM_OFFSET);
        else
            JC &= ~(1 << L_PWM_OFFSET);

        if (pwmCnt <= duty_right)
            JC |= (1 << R_PWM_OFFSET);
        else
            JC &= ~(1 << R_PWM_OFFSET);

        if (++pwmCnt >= PWM_TOP) {
            pwmCnt = 0;

            int32_t left_count = read_L1_quad_enc(0);
            int32_t right_count = read_R1_quad_enc(0);
            int32_t encoder_diff = right_count - left_count;
            
            if((encoder_diff > 0) | (left_distance > 3)){
                // Right motor duty cycle too high (veering right)
                // Right encoder value higher than left and sensor farther than 3in
                duty_left += 1;
                duty_right -= 1;

            }
            if((encoder_diff < 0) | (left_distance < 3)){
                // Left motor duty cycle too high (veering left)
                // Left encoder value higher than right and sensor closer than 3in
                duty_left -= 1;
                duty_right += 1;
            }
            if(duty_left > 180){
                // Left duty cycle reaches max of 180
                duty_right -= 1;
                duty_left = 180;                
                
            }
            if(duty_right > 180){
                // right duty cycle reaches max of 180
                duty_left -= 1;
                duty_right = 180;
                
            }
        }
    }

    // Stop motors and directions
    JC &= ~((1 << L_PWM_OFFSET) | (1 << R_PWM_OFFSET) | 
            (1 << LEFT1_OFFSET) | (1 << LEFT2_OFFSET) | 
            (1 << RIGHT1_OFFSET) | (1 << RIGHT2_OFFSET));
}

void count_display(uint8_t count){
    uint8_t sevenSegValue[4] = {0};

    uint8_t sevenSegLUT[10] = {
	    0xC0,
        0xF9,
        0xA4,
        0xB0,
        0x99,
        0x92,
        0x82,
        0xF8,
        0x80,
        0x98,
    };
	sevenSegValue[0] = sevenSegLUT[count];
    sevenSegValue[1] = sevenSegLUT[0];
    sevenSegValue[2] = sevenSegLUT[0];
    sevenSegValue[3] = sevenSegLUT[0];
    show_sseg(&sevenSegValue[0]);
}


//setting functions

void set_trig1_pin(){
    // set trig of front sensor
    JB |= 0x01;
}

void set_trig2_pin(){
    // set trig of left sensor
    JB |= 0x04;
}

//clear functions

void clear_trig1_pin(){
    // clear trig of front sensor
    JB &= ~(0x01);
}

void clear_trig2_pin(){
    // clear trig of left sensor
    JB &= ~(0x04);
}


//read functions 

_Bool read_echo1_pin(){
    // reads front sensor echo pin
    _Bool echo1 = JB & 0x02;
    return echo1;
}

_Bool read_echo2_pin(){
    // reads left sensor echo pin
    bool echo2 = JB & 0x08;
    return echo2;
}


// /*
uint8_t Sensor1_distance() {
    // Trigger the sensor
    clear_trig1_pin();
    timer_2us(2);
    set_trig1_pin();
    timer_2us(10);
    clear_trig1_pin();

    // Wait for echo start
    uint32_t count = 0;
    while (!read_echo1_pin() && count < 1500) {
        timer_2us(20);
        count++;
    }

    if (count >= 1500) return 100; // timeout

    // Start stopwatch
    start_stopwatch(1);

    // Wait for echo to go low
    while (read_echo1_pin());

    uint32_t time = read_stopwatch(1);
    timer_2us(100000); // Cooldown between pings

    return time / 148;
}


// */

// /*
uint8_t Sensor2_distance() {
    // Trigger the sensor
    clear_trig2_pin();
    timer_2us(2);
    set_trig2_pin();
    timer_2us(10);
    clear_trig2_pin();

    // Wait for echo start
    uint32_t count = 0;
    while (!read_echo2_pin() && count < 1500) {
        timer_2us(20);
        count++;
    }

    if (count >= 1500) return 0; // timeout

    // Start stopwatch
    start_stopwatch(1);

    // Wait for echo to go low
    while (read_echo2_pin());

    uint32_t time = read_stopwatch(1);
    timer_2us(100000); // Cooldown between pings

    return time / 148;
}

//*/



/*
uint8_t Sensor1_distance() {
    // Variables
    uint8_t distance_front = 0;
    uint32_t count = 0;
    uint32_t time = 0;

    // FSM States
    enum state_type {send_trig, wait_for_echo, count_echo_duration, echo_falling_edge};
    static enum state_type state = send_trig;
    enum state_type next_state = state;

    while (1) {
        switch (state) {

            case send_trig:
                // Trigger pulse: HIGH for 10us
                clear_trig1_pin();
                timer_2us(2);
                set_trig1_pin();
                timer_2us(10);
                clear_trig1_pin();

                count = 0;
                next_state = wait_for_echo;
                break;

            case wait_for_echo:
                if (read_echo1_pin()) {
                    start_stopwatch(1);
                    next_state = count_echo_duration;
                } else if (count >= 1500) { // ~30ms timeout if timer_2us(20)
                    return 100;  // timeout fallback value
                } else {
                    timer_2us(20); // wait 40 µs per loop
                    count++;
                }
                break;

            case count_echo_duration:
                if (!read_echo1_pin()) {
                    next_state = echo_falling_edge;
                }
                break;

            case echo_falling_edge:
                time = read_stopwatch(1);
                distance_front = time / 148;  // Convert to inches
                timer_2us(30000); // Cooldown ~60 ms before next ping
                next_state = send_trig;
                return distance_front;

            default:
                next_state = send_trig;
                break;
        }

        state = next_state;
    }
}
*/



/*
uint8_t Sensor2_distance(void){
    // Distance calculation for left sensor
    // Tracking Variables
    uint8_t distance_left = 0; // variable to compute distance of the object from the sensor
    uint32_t count = 0; // a counter variable
    uint32_t time = 0; // variable to count the duration of echo

    // State Enumerations
    enum state_type {send_trig, wait_for_echo, count_echo_duration, echo_falling_edge, cooldown};
    static enum state_type state = send_trig;
    enum state_type next_state = state;
    while(1){
        switch (state) {
            case send_trig:   
                //send 10us pulse to trig pin then move to next state
                clear_trig2_pin();
                timer_2us(5);
                set_trig2_pin();  
                timer_2us(5);
                clear_trig2_pin();
                count = 0;
                next_state = wait_for_echo;

            case wait_for_echo:    
                // Read the echo pin, if recieved restart timer then move to echo count
                // If echo not recieved then count till TIMEOUT
                if (read_echo1_pin()){
                    start_stopwatch(1);
                    next_state = count_echo_duration;
                    break;
                }
                else if (count == TIMEOUT){
                    next_state = send_trig;
                    return(100);
                    break;
                }
                else{
                    count = count + 1;
                }


            case count_echo_duration:
                // while echo is high stay in count echo until falling edge
                while(read_echo1_pin());
                next_state = echo_falling_edge;
            

            case echo_falling_edge:
                // Get timer value to get duration of echo high
                // Compute distance in inches how far object is away from sensor
                time = read_stopwatch(1); 
                distance_left = (time*0.00034)/2; // Distance from sensor
                next_state = send_trig;
                return(distance_left);

            default:
                // If no cases met send trig
                next_state = send_trig;
            
        }
        state = next_state; // Assign state to next_state
    }
}
*/



void Stop_motors(void){
    JC &= ~((1 << L_PWM_OFFSET) | (1 << R_PWM_OFFSET) | 
            (1 << LEFT1_OFFSET) | (1 << LEFT2_OFFSET) | 
            (1 << RIGHT1_OFFSET) | (1 << RIGHT2_OFFSET));
}


void Pass_object(void){
    drive_straight(5); 
    timer_2us(1000000); 
    Stop_motors();
}


void timer_2us(unsigned t) {
    volatile unsigned cntr1;
    while (t--) {
        for (cntr1 = 0; cntr1 < 8; cntr1++);
    }
}

void Turn_180(){
    Turn_right();
    timer_2us(500000);
    Turn_right();
}

// Function implementation - Button debounces
_Bool UpButton_pressed()
{
	_Bool last_btnU = 0;
	static _Bool btnU = 0;
	last_btnU = btnU;
	btnU = BUTTONS & (1 << BTNU_OFFSET);
	return (btnU & !last_btnU);
}

_Bool DownButton_pressed()
{
	_Bool last_btnD = 0;
	static _Bool btnD = 0;
	last_btnD = btnD;
	btnD = BUTTONS & (1 << BTND_OFFSET);
	return (btnD & !last_btnD);
}

_Bool LeftButton_pressed()
{
	_Bool last_btnL = 0;
	static _Bool btnL = 0;
	last_btnL = btnL;
	btnL = BUTTONS & (1 << BTNL_OFFSET);
	return (btnL & !last_btnL);
}

_Bool RightButton_pressed()
{
	_Bool last_btnR = 0;
	static _Bool btnR = 0;
	last_btnR = btnR;
	btnR = BUTTONS & (1 << BTNR_OFFSET);
	return (btnR & !last_btnR);
}

uint32_t read_L1_quad_enc(_Bool reset){
    static uint32_t cnt = 0;
    static _Bool quad_enc_last_state = 0;

    if (reset){cnt = 0;}

    if((quad_enc_last_state == 0) && (JA & (1<<L1_QUAD_ENC_OFFSET))){
        cnt++;
    }
    quad_enc_last_state = JA & (1<<L1_QUAD_ENC_OFFSET);
    return cnt;
}


uint32_t read_R1_quad_enc(_Bool reset){
    static uint32_t cnt = 0;
    static _Bool quad_enc_last_state = 0;

    if (reset){cnt = 0;}
    if((quad_enc_last_state == 0) && (JA & (1<<R1_QUAD_ENC_OFFSET))){
        cnt++;
    }
    quad_enc_last_state = JA & (1<<R1_QUAD_ENC_OFFSET);
    return cnt;
}
