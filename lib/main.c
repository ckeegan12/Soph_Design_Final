#include <stdint.h>
#include <stdbool.h>
#include <strings.h>
#include <sys/_intsup.h>
#include <xil_printf.h>
#include <xil_types.h>

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
#define TIMEOUT = 10000000 // How long to wait until assuming trig was lost
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
// Other
const volatile uint32_t TCSR_OFFEST = 0;
const volatile uint32_t TLR_OFFEST = 1;
const volatile uint32_t TCR_OFFSET = 2;

// Addresses for all the timers, ITP casts the int to uint32_t ptr
const uint32_t * TIMERS[] = { ITP 0x40009000, ITP 0x40009100, ITP 0x4000A000, ITP 0x4000A100,
                              ITP 0x4000B000, ITP 0x4000B100, ITP 0x4000C000, ITP 0x4000C100};

// Function declarations - implemented below
void init_program(); // One Time Initializations
_Bool delay_1s();
_Bool delay_half_sec();
void timer_2us(unsigned t);
void set_trig_pin();
void clear_trig_pin();
_Bool read_echo_pin();
void show_sseg(uint8_t * sevenSegValue);
void coord_display(uint8_t x_coord, uint8_t y_coord, uint8_t signed_x_coord, uint8_t signed_y_coord);
_Bool UpButton_pressed();
_Bool DownButton_pressed();
_Bool LeftButton_pressed();
_Bool RightButton_pressed();
uint32_t read_L1_quad_enc(_Bool reset);
uint32_t read_R1_quad_enc(_Bool reset);
uint32_t * convert_timer_to_hex_address (uint8_t timer_number);
void configure_timers();
void start_stopwatch(uint8_t timer_number);
uint32_t read_stopwatch(uint8_t timer_number);
uint8_t get_ycoordinates();
uint8_t get_xcoordinates();
// Motor Functions
uint32_t read_L1_quad_enc(_Bool reset);
uint32_t read_R1_quad_enc(_Bool reset);
// Drive Direction
_Bool Distance_Stop(uint8_t coord);
void drive_straight(uint8_t coord);
void Turn_left();
void Turn_right();
void Turn_180();

int main (void){
    // One time initializations
    init_program();
    uint8_t x_coord;
    uint8_t y_coord;
    int reset = 0;
    JA_DDR = 0x03;
    JB_DDR = 0x02;
    JC_DDR = 0x00;
    
    // coord_display(0, 0, 0, 0);

    while(!UpButton_pressed()){
        uint8_t signed_x_coord = get_xcoordinates();
        uint8_t signed_y_coord = get_ycoordinates();

        if(signed_x_coord & 0x80){
            x_coord = signed_x_coord - 0x80;
        }
        else {
        x_coord = signed_x_coord;
        }
        if(signed_y_coord & 0x80){
        y_coord = signed_y_coord - 0x80;
        }
        else {
        y_coord = signed_y_coord;
        }
        coord_display(x_coord, y_coord, signed_x_coord, signed_y_coord);

    }

    timer_2us(100000);
    
    uint8_t signed_x_coord = get_xcoordinates();
    uint8_t signed_y_coord = get_ycoordinates();

    

    while(!(reset)){
         // First Quadrant
        if(!(signed_x_coord & (0x80)) && !((signed_y_coord & (0x80)))){
            drive_straight(y_coord);
            timer_2us(50000);
            Turn_right();
            timer_2us(50000); 
            drive_straight(x_coord);
        }
         // Second Quandrant
        else if((signed_x_coord & (0x80)) && !((signed_y_coord & (0x80)))){ 
            drive_straight(y_coord);
            timer_2us(50000);
            Turn_left();
            timer_2us(50000);
            drive_straight(x_coord);            
        }
         // Third Quadrant
        else if((signed_x_coord & (0x80)) && ((signed_y_coord & (0x80)))){
            Turn_180();
            timer_2us(50000);
            drive_straight(y_coord);
            timer_2us(50000);
            Turn_right();
            timer_2us(50000);
            drive_straight(x_coord);
        }
         // Fourth Quadrant
        else if(!(signed_x_coord & (0x80)) && ((signed_y_coord & (0x80)))){
            Turn_180();
            timer_2us(50000);
            drive_straight(y_coord);
            timer_2us(50000);
            Turn_left();
            timer_2us(50000);
            drive_straight(x_coord);  
        }
        reset = 1;
    }

 	while (1)
 	{
        coord_display(x_coord, y_coord, signed_x_coord, signed_y_coord);
 	}
}

// Function Implementation - Initialization
void init_program(){
    configure_timers();
}

// Function Implementation - Ultrasonic Sensors
void set_trig_pin(){
    JB |=1;
}

void clear_trig_pin(){
    JB &=0;
}

_Bool read_echo_pin(){
    return JB & (1<<1);
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
        uint32_t * tcsr = timer_base_address + TCSR_OFFEST;
        *(tcr) = 0x00000000;
        *(tcsr) = 0b010010010001;
    }
}

void start_stopwatch(uint8_t timer_number){
    if (timer_number > 7){
        return;
}
    uint32_t * timer_base_address = convert_timer_to_hex_address(timer_number);
    volatile uint32_t * tcsr = timer_base_address + TCSR_OFFEST;

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

void timer_2us(unsigned t)
{
	volatile unsigned cntr1;
	while(t--)
		for( cntr1=0; cntr1 < 8; cntr1++);
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

uint8_t get_xcoordinates(){
    uint8_t signed_x_coord = 0; 

    signed_x_coord |= ((SWITCHES & 0xFF00) >> 8);

    return signed_x_coord;
}

uint8_t get_ycoordinates(){
    uint8_t signed_y_coord = 0; 

    signed_y_coord |= (SWITCHES & ~(0xFF00));

    return signed_y_coord;
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

void Turn_180(){
    Turn_right();
    timer_2us(500000);
    Turn_right();
}

void drive_straight(uint8_t distance) {
    JC &= ~((1 << L_PWM_OFFSET) | (1 << R_PWM_OFFSET) | 
            (1 << LEFT1_OFFSET) | (1 << LEFT2_OFFSET) | 
            (1 << RIGHT1_OFFSET) | (1 << RIGHT2_OFFSET));

    uint16_t pwmCnt = 0;
    uint16_t cycles = 0;

    // Initial calibrated offset — favoring slightly slower right motor
    int16_t duty_left = DUTY_MOTION_START_LEFT;
    int16_t duty_right = DUTY_MOTION_START_RIGHT;

    read_R1_quad_enc(1);
    read_L1_quad_enc(1);

    JC |= (1 << LEFT1_OFFSET) | (1 << RIGHT2_OFFSET);
    JC &= ~((1 << LEFT2_OFFSET) | (1 << RIGHT1_OFFSET));

    while (!(Distance_Stop(distance))) {

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
            ++cycles;

            int32_t left_count = read_L1_quad_enc(0);
            int32_t right_count = read_R1_quad_enc(0);
            int32_t diff = left_count - right_count;

            if((cycles % 10) == 0){
                
                if (diff > 0) {
                duty_left -= 1;
                } 
                if (diff < 100) {
                duty_left += 1;
                }
            }
            if (duty_left > PWM_TOP) duty_left = PWM_TOP;
            if (duty_right > PWM_TOP) duty_right = PWM_TOP;
            if (duty_left < DUTY_MOTION_START_LEFT) duty_left = DUTY_MOTION_START_LEFT;
            if (duty_right < DUTY_MOTION_START_RIGHT) duty_right = DUTY_MOTION_START_RIGHT;
        }
    }

    // Stop motors and directions
    JC &= ~((1 << L_PWM_OFFSET) | (1 << R_PWM_OFFSET) | 
            (1 << LEFT1_OFFSET) | (1 << LEFT2_OFFSET) | 
            (1 << RIGHT1_OFFSET) | (1 << RIGHT2_OFFSET));
}



void coord_display(uint8_t x_coord, uint8_t y_coord, uint8_t signed_x_coord, uint8_t signed_y_coord){
    uint8_t sevenSegValue[4] = {0};

    uint8_t Pos_sevenSegLUT[10] = {
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

    uint8_t Neg_sevenSegLUT[10] = {
		0x40,
        0x79,
        0x80,
        0x30,
        0x19,
        0x12,
        0x02,
        0x78,
        0x00,
        0x18,
    };

    if(!(signed_x_coord & 0x80)){
        sevenSegValue[2] = Pos_sevenSegLUT[x_coord % 10];
        sevenSegValue[3] = Pos_sevenSegLUT[(x_coord - (x_coord % 10)) / 10];
    }
    else if((signed_x_coord & 0x80)){
        sevenSegValue[2] = Neg_sevenSegLUT[x_coord % 10];
        sevenSegValue[3] = Neg_sevenSegLUT[(x_coord - (x_coord % 10)) / 10];
    }
    if(!(signed_y_coord & 0x80)){
        sevenSegValue[0] = Pos_sevenSegLUT[y_coord % 10];
        sevenSegValue[1] = Pos_sevenSegLUT[(y_coord - (y_coord % 10)) / 10];
    }
    else if((signed_y_coord & 0x80)){
        sevenSegValue[0] = Neg_sevenSegLUT[y_coord % 10];
        sevenSegValue[1] = Neg_sevenSegLUT[(y_coord - (y_coord % 10)) / 10];
    }
    show_sseg(&sevenSegValue[0]);
}

_Bool Distance_Stop(uint8_t coord){
    _Bool stop = false;

    if(((read_R1_quad_enc(0) >> 5) >= (coord + 0x09)) && ((read_L1_quad_enc(0) >> 5) >= (coord + 0x09))){
        stop = true;
    }
    return(stop);
}
