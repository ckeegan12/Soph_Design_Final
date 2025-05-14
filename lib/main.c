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
#define DUTY_MOTION_START_RIGHT 185
#define DUTY_MOTION_START_LEFT 200
#define DUTY_LEFT_STRAIGHT 2
#define DUTY_RIGHT_STRAIGHT 2
#define UPDATE_STRAIGHT 2
#define UPDATE_STRAIGHT2 4
#define PWM_TOP 255
#define PWM_TOP_STRAIGHT 8
#define LEFT_ENCODER_CNT 41 // left encoder count to 1in (1in times 41)
#define RIGHT_ENCODER_CNT 41 // right encoder count to 1in

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
void PID_controller(uint8_t left_distnace);

// Drive Direction
void Turn_left();
void Turn_right();
void drive_straight_while_monitoring(void); 
//void drive_straight_until_left_wall(void);


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


uint8_t done_letters[4] = {
                    0x5E,  // 'd'
                    0x5C,  // 'o'
                    0xA8,  // 'n'
                    0xF6   // 'e'
                    };

// state global variables
typedef enum { ACTION_DRIVE_STRAIGHT, ACTION_TURN_RIGHT, ACTION_FINAL_STATE, ACTION_TURN_LEFT} action_type;
typedef enum {STATE_NORMAL, STATE_ONE_RIGHT, STATE_TWO_RIGHTS} logic_state_type;

// driving gloabal variables
uint8_t left_duty = DUTY_LEFT_STRAIGHT;
uint8_t right_duty = DUTY_RIGHT_STRAIGHT;
static uint16_t pwmCnt = 0;

int main(void) {
    //innitialize pins
    init_program();
    JA_DDR = 0x03;
    JC_DDR = 0x00;
    JB_DDR = 0x0A;

    uint8_t count_object = 0;

    logic_state_type logic_state = STATE_NORMAL;

    while (!UpButton_pressed());

    while (1) {
        uint8_t front_distance = Sensor1_distance();
        uint8_t left_distance = Sensor2_distance();

        static action_type action = ACTION_DRIVE_STRAIGHT;

        // Determine action based on sensor readings
        if (front_distance < 3) {
            action = ACTION_TURN_RIGHT;
        } else if (left_distance > 10) {
            action = ACTION_TURN_LEFT;
        } else {
            action = ACTION_DRIVE_STRAIGHT;
        }

        // FSM logic to detect two consecutive left turns (to know when end)
        switch (logic_state) {
            case STATE_NORMAL:
                if (action == ACTION_TURN_RIGHT)
                    logic_state = STATE_ONE_RIGHT;
                break;

            case STATE_ONE_RIGHT:
                if (action == ACTION_TURN_RIGHT)
                    logic_state = STATE_TWO_RIGHTS;
                else if (action != ACTION_TURN_RIGHT)
                    logic_state = STATE_NORMAL;
                break;

            case STATE_TWO_RIGHTS:
                action = ACTION_FINAL_STATE;
                logic_state = STATE_NORMAL;
                break;
        }

        // Perform the action
        switch (action) {
            case ACTION_TURN_RIGHT:
                //track how many object encountered
                count_object++;
                Turn_right();
                timer_2us(100000);
                break;

            case ACTION_TURN_LEFT:
                timer_2us(100000);
                Pass_object();
                timer_2us(100000);
                break;

            case ACTION_DRIVE_STRAIGHT:
                //drive straight until object encountered
                drive_straight_while_monitoring();
                break;

            case ACTION_FINAL_STATE:
                show_sseg(done_letters);
                LEDS = 0xFFFF;
                count_display(count_object);
                while (1);  // halt forever
                break;
        }
        // Display count of how many objects encountered
        if(left_distance < 2){
            left_duty = UPDATE_STRAIGHT;
            right_duty = UPDATE_STRAIGHT2;
        }
        else if (left_distance > 4) {
            right_duty = UPDATE_STRAIGHT;
            left_duty = UPDATE_STRAIGHT2;
        }
        else {
            left_duty = DUTY_LEFT_STRAIGHT;
            right_duty = DUTY_LEFT_STRAIGHT;
        }
    }
    return 0;
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
  
    _Bool EndR = 0;
    _Bool EndL = 0;    
    uint16_t pwmCnt = 0;
    read_R1_quad_enc(1);
    read_L1_quad_enc(1);

    while(!(EndL & EndR)){
        JC |= (1<<RIGHT1_OFFSET) | (1<<LEFT1_OFFSET);
        JC &= ~(1<<RIGHT2_OFFSET) & ~(1<<LEFT2_OFFSET);
        
        if (pwmCnt <= DUTY_MOTION_START_LEFT && !EndL)
            JC |= (1 << L_PWM_OFFSET);
        else
            JC &= ~(1 << L_PWM_OFFSET);
        if (pwmCnt <= DUTY_MOTION_START_RIGHT && !EndR)
            JC |= (1 << R_PWM_OFFSET);
        else 
            JC &= ~(1 << R_PWM_OFFSET);
        if (pwmCnt++ == PWM_TOP)
            pwmCnt = 0;
        if (!(read_R1_quad_enc(0) < 200) ){
            JC &= ~(1 << R_PWM_OFFSET);
            EndR = 1;
        }
        if (!(read_L1_quad_enc(0) < 281)){
            JC &= ~(1 << L_PWM_OFFSET);
            EndL = 1;
        }
    }
}

void Pass_object(){
    _Bool pass = 0;
    
    JC |= (1 << LEFT1_OFFSET) | (1 << RIGHT2_OFFSET);
    JC &= ~((1 << LEFT2_OFFSET) | (1 << RIGHT1_OFFSET));

    read_L1_quad_enc(1);
    read_R1_quad_enc(1);

    while(!pass){
        if(DUTY_MOTION_START_LEFT >= pwmCnt){
            JC |= (1 << L_PWM_OFFSET);
        }
        else{
            JC &= ~(1 << L_PWM_OFFSET);
        }
        if(DUTY_MOTION_START_RIGHT >= pwmCnt){
            JC |= (1 << R_PWM_OFFSET);
        }
        else{
            JC &= ~(1 << R_PWM_OFFSET);
        }
        ++pwmCnt;
        if(pwmCnt >= PWM_TOP){
            pwmCnt = 0;       
        }
        if(read_L1_quad_enc(0) > (LEFT_ENCODER_CNT) && read_R1_quad_enc(0) > (RIGHT_ENCODER_CNT)){
            pass = 1;
        }
    }

    Turn_left();

    pass = 0;

    JC |= (1 << LEFT1_OFFSET) | (1 << RIGHT2_OFFSET);
    JC &= ~((1 << LEFT2_OFFSET) | (1 << RIGHT1_OFFSET));

    read_L1_quad_enc(1);
    read_R1_quad_enc(1);

    while(!pass){
        if(DUTY_MOTION_START_LEFT >= pwmCnt){
            JC |= (1 << L_PWM_OFFSET);
        }
        else{
            JC &= ~(1 << L_PWM_OFFSET);
        }
        if(DUTY_MOTION_START_RIGHT >= pwmCnt){
            JC |= (1 << R_PWM_OFFSET);
        }
        else{
            JC &= ~(1 << R_PWM_OFFSET);
        }
        ++pwmCnt;
        if(pwmCnt >= PWM_TOP){
            pwmCnt = 0;     
        }
        if(read_L1_quad_enc(0) > (2*LEFT_ENCODER_CNT) && read_R1_quad_enc(0) > (2*RIGHT_ENCODER_CNT)){
            pass = 1;
        }
    }
}

void Turn_right(){

    _Bool EndL = 0;
    _Bool EndR = 0;
    uint16_t pwmCnt = 0;
    read_R1_quad_enc(1);
    read_L1_quad_enc(1);

    while(!(EndL & EndR)){
        JC |= (1<<RIGHT2_OFFSET) | (1<<LEFT2_OFFSET);
        JC &= ~(1<<RIGHT1_OFFSET) & ~(1<<LEFT1_OFFSET);
        
        if (pwmCnt <= DUTY_MOTION_START_LEFT)
            JC |= (1 << L_PWM_OFFSET);
        else
            JC &= ~(1 << L_PWM_OFFSET);
        if (pwmCnt <= DUTY_MOTION_START_RIGHT)
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

void drive_straight_while_monitoring(void) {
    
    JC |= (1 << LEFT1_OFFSET) | (1 << RIGHT2_OFFSET);
    JC &= ~((1 << LEFT2_OFFSET) | (1 << RIGHT1_OFFSET));
    
    if(left_duty >= pwmCnt){
        JC |= (1 << L_PWM_OFFSET);
    }
    else{
        JC &= ~(1 << L_PWM_OFFSET);
    }
    if(right_duty >= pwmCnt){
        JC |= (1 << R_PWM_OFFSET);
    }
    else{
        JC &= ~(1 << R_PWM_OFFSET);
    }
    ++pwmCnt;
    if(pwmCnt >= PWM_TOP_STRAIGHT){
        pwmCnt = 0;
            
    }
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

//Distance Calculations

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


void Stop_motors(void){
    while(delay_half_sec()){
        JC &= ~((1 << L_PWM_OFFSET) | (1 << R_PWM_OFFSET));
    }
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
