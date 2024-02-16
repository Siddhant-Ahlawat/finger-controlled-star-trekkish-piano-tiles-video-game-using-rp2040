/**
 * Siddhant ahlawat 
 * 
 * Mandelbrot set calculation and visualization
 * Uses PIO-assembly VGA driver.
 * 
 * Core 1 draws the bottom half of the set using floating point.
 * Core 0 draws the top half of the set using fixed point.
 * This illustrates the speed improvement of fixed point over floating point.
 * 
 * https://vanhunteradams.com/FixedPoint/FixedPoint.html
 * https://vanhunteradams.com/Pico/VGA/VGA.html
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 15-A mux
 *  - GPIO 14-B mux
 * GPIO 13-C mux
 * gpio26/ADC0- comout mux
 *1
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0 and 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */
#include "vga_graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "registers.h"
#include "math.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"
////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////// Stuff for Mandelbrot ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Fixed point data type
typedef signed int fix28 ;
#define multfix28(a,b) ((fix28)(((( signed long long)(a))*(( signed long long)(b)))>>28)) 
#define float2fix28(a) ((fix28)((a)*268435456.0f)) // 2^28
#define fix2float28(a) ((float)(a)/268435456.0f) 
#define int2fix28(a) ((a)<<28)
// the fixed point value 4
#define FOURfix28 0x40000000 
#define SIXTEENTHfix28 0x01000000
#define ONEfix28 0x10000000
int previous_sound=0;
// Maximum number of iterations
#define max_count 1000
int input_flex1=0;
int input_flex2=0;
int input_flex3=0;
int input_flex4=0;
#define LEFT_VERT 150
#define MID_VERT 240
#define RIGHT_VERT 420
#define THIRD_VERT 330
float speed_fact=2;
#define LEFT_VERT_TILES 160
#define MID_VERT_TILES 250
#define THIRD_VERT_TILES 340
#define RIGHT_VERT_TILES 430
#define SELECT_LINE_A 10
#define SELECT_LINE_B 11
#define SELECT_LINE_C 12
#define SELECT_LINE_D 13
#define RESTART_PIN 4
#define RESTART_PIN_REG ((volatile uint32_t *)(IO_BANK0_BASE + 0x010))
uint adc_x_raw;
//***************************************************************************************
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 40000            // sample rate

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             200
#define DECAY_TIME              200
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           5200
#define BEEP_REPEAT_INTERVAL    40000

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define ISR      15
#define SPI_PORT spi0

// Two variables to store core number
volatile int corenum_0  ;

// Global counter for spinlock experimenting
volatile int global_counter = 0 ;
//***********
int flag=0;



//**********************************
//***********************************************************sound

// This timer ISR is called on core 0
bool repeating_timer_callback_core_0(struct repeating_timer *t) {
    

    if (STATE_0 == 1) {
        // DDS phase and sine table lookup
        gpio_put(ISR, 1) ;
        float y;
        y=-260*sin(-1*3.141592*count_0/5200)+1740;
        volatile unsigned int phase_incr_main_3=y*(two32)/Fs;
        phase_accum_main_0 += phase_incr_main_3  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 0 ;
            count_0 = 0 ;
            flag=0;
        }
        gpio_put(ISR, 0) ;

    }

    else if (STATE_0 ==2)
       {
        // DDS phase and sine table lookup
        gpio_put(ISR, 1) ;
        float y;
        y=0.000184*count_0*count_0 + 2000; 
        volatile unsigned int phase_incr_main_2 = (y*two32)/Fs ;
        phase_accum_main_0 += phase_incr_main_2  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 0 ;
            count_0 = 0 ;
            flag=0;
        }
        gpio_put(ISR, 0) ;
    }


else if (STATE_0 ==4)
       {
        

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == 1200) {
            STATE_0 = 0 ;
            count_0 = 0 ;
            flag=0;
        }
    }

else if (STATE_0 ==5)
       {
        // DDS phase and sine table lookup
        float y;
        y=-0.5769*count_0+6000; 
        volatile unsigned int phase_incr_main_5 = (y*two32)/Fs ;
        phase_accum_main_0 += phase_incr_main_5  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 0 ;
            count_0 = 0 ;
            flag=0;
        }
    }

else if (STATE_0 ==6)
       {
        // DDS phase and sine table lookup
        float y;
        y=-0.192*count_0 + 3000; 
        volatile unsigned int phase_incr_main_6 = (y*two32)/Fs ;
        phase_accum_main_0 += phase_incr_main_6  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 0 ;
            count_0 = 0 ;
            flag=0;
        }
    }

else if (STATE_0 ==7)
       {
        // DDS phase and sine table lookup
        float y;
        y=-0.00099853142804*(count_0)*(count_0)+1.99456285608*count_0+1010; 
        volatile unsigned int phase_incr_main_7 = (y*two32)/Fs ;
        phase_accum_main_0 += phase_incr_main_7  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;

        // Ramp up amplitude
        if (count_0 < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (count_0 > BEEP_DURATION - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == 2000) {
            STATE_0 = 0 ;
            count_0 = 0 ;
            flag=0;
        }
    }

    // State transition?
    else {
        count_0 += 1 ;
        if (flag==1) {
            current_amplitude_0 = 0 ;
            STATE_0 = 1 ;
            count_0 = 0 ;
            flag=0;
        }
        if (flag==2) {
            current_amplitude_0 = 0 ;
            STATE_0 = 2 ;
            count_0 = 0 ;
            flag=0;
        }
       
       if (flag==4) {
        current_amplitude_0 = 0 ;
            STATE_0 = 4 ;
            count_0 = 0 ;
            flag=0;
       }
    
    if (flag==5) {
        current_amplitude_0 = 0 ;
            STATE_0 = 5 ;
            count_0 = 0 ;
            flag=0;
       }

       if (flag==6) {
        current_amplitude_0 = 0 ;
            STATE_0 = 6 ;
            count_0 = 0 ;
            flag=0;
       }

       if (flag==7) {
        current_amplitude_0 = 0 ;
            STATE_0 = 7 ;
            count_0 = 0 ;
            flag=0;
       }




    }

    // retrieve core number of execution
    corenum_0 = get_core_num() ;
    
    return true;
}




//**************************************************


uint act_adc() {
    adc_select_input(0);
    adc_x_raw = adc_read();
    uint adc_x = 0;
    input_flex1=0;
    input_flex2=0;
    input_flex3=0;
    input_flex4=0;
    input_flex1=gpio_get(SELECT_LINE_A);
    input_flex2=gpio_get(SELECT_LINE_B);
    input_flex3=gpio_get(SELECT_LINE_C);
    input_flex4=gpio_get(SELECT_LINE_D);
//*********************************
 if (input_flex2 == 1 ){
      adc_x=2;
        fillRect(MID_VERT,460,60,20,WHITE);
        fillRect(LEFT_VERT,460,60,20,0);
        fillRect(THIRD_VERT,460,60,20,0);
        fillRect(RIGHT_VERT,460,60,20,0);
    }
    else if (input_flex4== 1 ) {
        adc_x=4;
        fillRect(RIGHT_VERT,460,60,20,WHITE);
        fillRect(LEFT_VERT,460,60,20,0);
        fillRect(THIRD_VERT,460,60,20,0);
        fillRect(MID_VERT,460,60,20,0);
    }else if (input_flex1==1 ) {
        adc_x=1;
        fillRect(LEFT_VERT,460,60,20,WHITE);
        fillRect(MID_VERT,460,60,20,0);
        fillRect(THIRD_VERT,460,60,20,0);
        fillRect(RIGHT_VERT,460,60,20,0);
    }
    else if (input_flex3==1 ) {
        adc_x=3;
        fillRect(THIRD_VERT,460,60,20,WHITE);
        fillRect(LEFT_VERT,460,60,20,0);
        fillRect(MID_VERT,460,60,20,0);
        fillRect(RIGHT_VERT,460,60,20,0);
    }
    else if(adc_x==0)
    {fillRect(THIRD_VERT,460,60,20,0);
        fillRect(LEFT_VERT,460,60,20,0);
        fillRect(MID_VERT,460,60,20,0);
        fillRect(RIGHT_VERT,460,60,20,0);

    }

    sleep_ms(10);
    return adc_x;
}

void draw_fill_rect(short x, short y, short w, short h, char color, short inc_dec){
    fillRect(x,y,w,h,color);
    fillRect(x,y,w,inc_dec,0);
    fillRect(x,y+h,w,inc_dec,color);
    sleep_ms(10);
}

void update_score(uint score){
    fillRect(30,60,240,20,0);
    /* setCursor(30, 30); */
    /* setTextSize(3); */
    char str_score[3] = {'0', '0', '0'};
    str_score[2] = (score % 10) + '0';
    str_score[1] = ((score/10) % 10) + '0';
    str_score[0] = (((score/10)/10) % 10) + '0';
    drawChar(30, 60, str_score[0], WHITE, 0, 2);
    drawChar(45, 60, str_score[1], WHITE, 0, 2);
    drawChar(60, 60, str_score[2], WHITE, 0, 2);
}


// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
 

     uint blue_indx = 20, green_indx = 40, cyan_indx = 60, yellow_indx=0, joystick_pos = 0;
    uint curr_score = 0, buttons_status = 0;

    drawChar(30, 30, 'S', WHITE, 0, 2);
    drawChar(45, 30, 'c', WHITE, 0, 2);
    drawChar(60, 30, 'o', WHITE, 0, 2);
    drawChar(75, 30, 'r', WHITE, 0, 2);
    drawChar(90, 30, 'e', WHITE, 0, 2);
    drawChar(105, 30, ':', WHITE, 0, 2);
    update_score(curr_score);

    sleep_ms(5000);

    while(true) {
        while (true){
            joystick_pos = act_adc();
            char info[100];
      sprintf(info, "ADC:%d| ", adc_x_raw);
      setCursor(0,0);
      setTextColor2(WHITE, BLACK);
      setTextSize(1);
      writeString(info);



            if (cyan_indx > 355/speed_fact) {
                cyan_indx = 0;
                fillRect(RIGHT_VERT_TILES,360,40,100,0);
                if (joystick_pos ==4) {
                    sleep_ms(40);
                    fillRect(RIGHT_VERT_TILES,360,40,100,RED);
                    flag=1;
                    sleep_ms(40);
                    fillRect(RIGHT_VERT_TILES,360,40,100,0);
                    curr_score += 1;
                    update_score(curr_score);
                } else {
                    fillRect(RIGHT_VERT,460,60,20,0);
                    flag=2;
                    break;
                }
            }


            if (yellow_indx > 355/speed_fact) {
                yellow_indx = 0;
                fillRect(THIRD_VERT_TILES,360,40,100,0);
                if (joystick_pos ==3) {
                    sleep_ms(40);
                    fillRect(THIRD_VERT_TILES,360,40,100,RED);
                    flag=1;
                    sleep_ms(40);
                    fillRect(THIRD_VERT_TILES,360,40,100,0);
                    curr_score += 1;
                    update_score(curr_score);
                } else {
                    fillRect(THIRD_VERT,460,60,20,0);
                    flag=2;
                    break;
                }
            }

            if (green_indx > 355/speed_fact) {
                green_indx = 0;
                fillRect(MID_VERT_TILES,360,40,100,0);
                if (joystick_pos == 2) {
                    sleep_ms(40);
                    fillRect(MID_VERT_TILES,360,40,100,RED);
                    flag=1;
                    sleep_ms(40);
                    fillRect(MID_VERT_TILES,360,40,100,0);
                    curr_score += 1;
                    update_score(curr_score);
                } else {
                    fillRect(MID_VERT,460,60,20,0);
                    flag=2;
                    break;
                }
            }

            if (blue_indx > 355/speed_fact) {
                blue_indx = 0;
                fillRect(LEFT_VERT_TILES,360,40,100,0);
                if (joystick_pos==1) {
                    sleep_ms(40);
                    fillRect(LEFT_VERT_TILES,360,40,100,RED);
                    flag=1;
                    sleep_ms(40);
                    fillRect(LEFT_VERT_TILES,360,40,100,0);
                    curr_score += 1;
                    update_score(curr_score);
                } else {
                    fillRect(LEFT_VERT,460,60,20,0);
                    flag=2;
                    break;
                }
            }
            

            draw_fill_rect(LEFT_VERT_TILES,(blue_indx*speed_fact),40,100,BLUE,speed_fact);
            draw_fill_rect(MID_VERT_TILES,(green_indx*speed_fact),40,100,GREEN,speed_fact);
            draw_fill_rect(RIGHT_VERT_TILES,(cyan_indx*speed_fact),40,100,CYAN,speed_fact);
             draw_fill_rect(THIRD_VERT_TILES,(yellow_indx*speed_fact),40,100,YELLOW,speed_fact);
            cyan_indx++;
            green_indx++;
            blue_indx++;
            yellow_indx++;
            //speed_fact= speed_fact+ 0.1;
        }

        fillRect(LEFT_VERT_TILES,(blue_indx*speed_fact),40,100,0);
        fillRect(MID_VERT_TILES,(green_indx*speed_fact),40,100,0);
        fillRect(RIGHT_VERT_TILES,(cyan_indx*speed_fact),40,100,0);
         fillRect(THIRD_VERT_TILES,(yellow_indx*speed_fact),40,100,0);

        drawChar(180, 240, 'G', WHITE, 0, 5);
        drawChar(210, 240, 'A', WHITE, 0, 5);
        drawChar(240, 240, 'M', WHITE, 0, 5);
        drawChar(270, 240, 'E', WHITE, 0, 5);
        drawChar(300, 240, ' ', WHITE, 0, 5);
        drawChar(330, 240, 'O', WHITE, 0, 5);
        drawChar(360, 240, 'V', WHITE, 0, 5);
        drawChar(390, 240, 'E', WHITE, 0, 5);
        drawChar(420, 240, 'R', WHITE, 0, 5);
        drawChar(450, 240, '!', WHITE, 0, 5);
        drawChar(480, 240, '!', WHITE, 0, 5);
        
        buttons_status = register_read(RESTART_PIN_REG);
        printf("0x%08x\n", buttons_status);
        while (buttons_status == 0){
            buttons_status = register_read(RESTART_PIN_REG);
            printf("0x%08x\n", buttons_status);
            sleep_ms(10);
        }

        fillRect(180,240,400,100,0);
        curr_score = 0;
        update_score(curr_score);

        }
    




    // Indicate thread end
    PT_END(pt) ;
}








int main() {

    gpio_init(RESTART_PIN);
    gpio_set_dir(RESTART_PIN, GPIO_IN);

     gpio_init(SELECT_LINE_A);
    gpio_set_dir(SELECT_LINE_A, GPIO_IN);

    gpio_init(SELECT_LINE_B);
    gpio_set_dir(SELECT_LINE_B, GPIO_IN);

    gpio_init(SELECT_LINE_C);
    gpio_set_dir(SELECT_LINE_C, GPIO_IN);

    gpio_init(SELECT_LINE_D);
    gpio_set_dir(SELECT_LINE_D, GPIO_IN);
    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    /* int pattern_array[6] = {20, 80, 20, 120, 60, 20} */
    
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
  
//****************************************
 // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;


    gpio_init(ISR) ;
    gpio_set_dir(ISR, GPIO_OUT) ;
    gpio_put(ISR, 0) ;

// set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Create a repeating timer that calls 
    // repeating_timer_callback (defaults core 0)
    struct repeating_timer timer_core_0;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    add_repeating_timer_us(-25, 
        repeating_timer_callback_core_0, NULL, &timer_core_0);


//*******************************************
  

   

        // Add core 0 threads
    pt_add_thread(protothread_core_0) ;
        pt_schedule_start ;
    
}
