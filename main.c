#include <htc.h>
#include <pic16f1936.h>
#include <stdio.h>
#include <stdint.h>


#define _XTAL_FREQ          4000000

#pragma config WDTE = OFF

#define true                        1
#define false                       0

#define SWITCH_PRESSED              0

#define PWM_PERIOD_IN_NSEC          250000  
#define TOSC_IN_NSEC                250

#define DEBOUNCE_TIME_IN_USEC       20

//typedef enum 
//{
//    THOUSANDS_PLACE = 0,
//    HUNDREDS_PLACE,
//    TENS_PLACE,
//    UNIT_PLACE,
//    MAX_DIGIT

//} T_DIGIT_POS;

#define NORMAL_DELAY_MS     1
#define MODERATE_DELAY_MS   2
#define LESS_DELAY_MS       1
#define LESSER_DELAY_MS     1
#define LEAST_DELAY_MS      1

//Define pins for 7segment
#define _DIGIT_1000s_       PORTBbits.RB0 
#define _DIGIT_100s_        PORTBbits.RB1 
#define _DIGIT_10s_         PORTBbits.RB2 
#define _DIGIT_1s_          PORTBbits.RB3 

#define _INC_SWITCH_        PORTBbits.RB4
#define _DEC_SWITCH_        PORTBbits.RB5



uint16_t duty_cycle;
unsigned long pwm_period,on_time;
uint16_t pwm_value ;

uint8_t on_time_int;
uint16_t max_value, curr_value, last_value;

uint8_t last_value_status = 0;
uint8_t is_pwm_configured;

uint8_t is_pwm_loaded ;
uint8_t pos;
                            /*   --THOUSANDS_PLACE--,--HUNDREDS_PLACE--,--TENS_PLACE--,--UNIT_PLACE--  */
//uint8_t digit_delay[MAX_DIGIT] = { NORMAL_DELAY_MS, NORMAL_DELAY_MS, NORMAL_DELAY_MS, NORMAL_DELAY_MS };
//uint8_t f_carry_10s = false, f_carry_100s = false, f_carry_1000s = false;

uint8_t digipos[4];



const unsigned char DisplayNum[10]= {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F}; //common cathode
//const uchar hexvalue[]= {0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xF8,0x80,0x90};     //common anode
//const uchar hexvalue[] = {0x7E,0x12,0xBC,0xB6,0xD2,0xE6,0xEE,0x32,0xFE,0xF6};    // amit-uncle


void load_pwm(uint8_t pwm_value)
{
    unsigned char lsb,msb;

    lsb = (pwm_value & 0x0003);       // Put 2 bits in CCP5CON
    CCP5CON |= (lsb << 5);

    msb = (unsigned char) (pwm_value >> 2);   // Put 6 bits in CCPR5L
    CCPR5L = msb;

    TMR2ON = 1;  // CCP1 is based on timer2 with CCPTMRS0 -- not using

    for(;;)
    {
        if( TMR2IF == 1 )
        {
            break;
        }
    }
}


void initPwm(void)
{

    CCP5CON = 0;
    TMR2 = 0;

    // TOSC = 250 ns , freq = 4Mhz   ------ 16mhz - 62.5ns 
    // pwm_freq = 4 khz (assume)
    // pwm_period(t) = 250 * 1000 ns
    // pwm_period = 1/4khz = 0.00025 sec

    PR2 = (uint8_t)((PWM_PERIOD_IN_NSEC/(4*TOSC_IN_NSEC )) - 1);  // 250 - 1 = 249 **** pwm period
    CCP5CON = 0x0C;   // pwm mode
    TMR2IF = 0;
    TMR2IE = 0;
    load_pwm(0);
    TRISA4 = 0;       //CCP1 = RC1
}


uint16_t map_value_to_pwm_resolution(uint16_t x,uint16_t max_value ) 
{
  return (uint16_t)((x) * (float)(255 / max_value))  ;
}


void init_port(){
    TRISB = 0x30; //defining pins as O/P -- RB0,1,2,3 : I/P: RB4,5
    TRISC = 0x00; //defining pins as O/P
    TRISA4 = 0;
    PORTC = 0x00; //Sending 0 logic - all pins
    OSCCON = 0x6A;   //clk = 4Mhz
    ANSELB = 0x00;
    PORTB = 0x00;
}

uint8_t is_inc_switch_pressed()
{
    uint8_t switch_status =  false;

    if(_INC_SWITCH_ == SWITCH_PRESSED)
    {
        //Wait time more then bouncing period
        __delay_ms(DEBOUNCE_TIME_IN_USEC);
        //switch_status = _INC_SWITCH_ ? true : false;
        if(_INC_SWITCH_ == SWITCH_PRESSED)
        {
            switch_status = true ;
        }    
        else
        {
            switch_status = false ;
        } 
    }

    return switch_status ;
}

uint8_t is_dec_switch_pressed()
{
    uint8_t switch_status =  false;

    if(_DEC_SWITCH_ == SWITCH_PRESSED)
    {
        //Wait time more then bouncing period
        __delay_ms(DEBOUNCE_TIME_IN_USEC);
        if(_DEC_SWITCH_ == SWITCH_PRESSED)
        {
            switch_status = true ;
        }    
        else
        {
            switch_status = false ;
        }   
    }
    
    return switch_status ;
}


void init_timer(void)
{
	T0CS = 0;			// select internal clock
    GIE = 1;			// enable global interrupts
	TMR0IF = 0;          // TMR0 register did not overflow
	TMR0IE = 1;			// enable timer interrupt
	OPTION_REG = 0b10000000;
}


void __interrupt() isr(void)

{
    
    if (TMR0IF == 1 ){
        
       
        if ( pos == 0 )  //units
        {
            LATC = digipos[pos];
            _DIGIT_1s_ = 0;
            _DIGIT_10s_ = 1;
            _DIGIT_100s_ = 1;
            _DIGIT_1000s_ = 1;
           
        }
        else if(pos == 1 )  //tens
        {
           
                LATC = digipos[pos];  
                _DIGIT_10s_ = 0;
                _DIGIT_1s_ = 1;
                _DIGIT_100s_ = 1;
                _DIGIT_1000s_ = 1;
            
        }
        else if(pos == 2 )  //hundreds
        {
                LATC = digipos[pos];
                _DIGIT_100s_ = 0;
                _DIGIT_1000s_ = 1;
                _DIGIT_1s_ = 1;
                _DIGIT_10s_ = 1;
            
        }
    
        else if (pos == 3)  //thousands
        {        
                LATC = digipos[pos];
                _DIGIT_1000s_ = 0;
                _DIGIT_1s_ = 1;             
                _DIGIT_10s_ = 1;              
                _DIGIT_100s_ = 1;
                
                
        }
        if(pos >= 3  )
        {
            pos = 0;
        }
        else pos++;
        
         TMR0IF = 0;
    }
}

void display_digit(uint16_t value)
{
     digipos[0] = DisplayNum[(value % 10)];         //units
     digipos[1] = DisplayNum[(value / 10) % 10];    //tens
     digipos[2] = DisplayNum[(value / 100) % 10];   //hundreds
     digipos[3] = DisplayNum[(value / 1000) % 10];  //thousand
}

void display_SET_command()
{
    digipos[0] = 0x78;
    digipos[1] = 0x79;
    digipos[2] = 0x6D;
    digipos[3] = DisplayNum[0];
}

void display_FR_command()
{
    digipos[0] = 0x50;
    digipos[1] = 0x71;
    digipos[2] = DisplayNum[0];
    digipos[3] = DisplayNum[0];
}

void set_delay_and_display(uint16_t counter )
{
    if(counter < 10)
    {   
        display_digit(curr_value);
        __delay_ms(200);
    }
    else if(counter >= 10 && counter < 100)
    {
        display_digit(curr_value);
        __delay_ms(50);
    }
   
}

void get_and_set_increment()
{
    uint16_t counter = 0 ;
    
    do{

        if(curr_value < max_value)
        {
           curr_value += 1 ;counter += 1; 
        }
        else
        {
           curr_value = max_value;
        }
        
        if(counter < 100)
        {
           set_delay_and_display(counter);
        }
        
        if(counter >= 100 && curr_value < max_value ) 
        {
           curr_value += 3;
           display_digit(curr_value);
        }
        else if(counter >= 300 && curr_value < max_value)
        {
            curr_value += 5;
            display_digit(curr_value);
        }
        
        
        
    }while(is_inc_switch_pressed());

}

void get_and_set_decrement()
{
    uint16_t counter = 0; 
    do{

        if(curr_value > 0)
        {
           curr_value -= 1 ;counter += 1;
        }
        else
        {
           curr_value = 0;
        }
        
        if(counter < 100)
        {
           set_delay_and_display(counter);
        }
        
        if(counter >= 100 && curr_value > 0  ) 
        {
           curr_value -= 3;
           display_digit(curr_value);
        }
        else if(counter >= 300 && curr_value > 0)
        {
            curr_value -= 5;
            display_digit(curr_value);
        }
        
        
        
    
    }while(is_dec_switch_pressed());

}


void init_EEPROM()
{
    /* IF NONE SW PRESSED THEN START FROM HERE // write lower byte -- 3 and write upper byte -- 4 , storelastvalue -- 2 */

    uint16_t last_lower_byte_rd, last_upper_byte_rd, max_lower_byte_rd, max_upper_byte_rd = 0;

   
    max_lower_byte_rd = (uint8_t)eeprom_read(0);
    max_upper_byte_rd = (uint8_t)eeprom_read(1);
    max_value = (uint16_t)((max_upper_byte_rd << 8) | max_lower_byte_rd) ; //total upper+lower
    
    last_value_status = (uint8_t)eeprom_read(3);
   // uint8_t last_rd = (uint8_t)eeprom_read(4);
    
    if( last_value_status == 0x3F )
    {
        last_lower_byte_rd = (uint8_t)eeprom_read(4);
        last_upper_byte_rd = (uint8_t)eeprom_read(5);
        last_value = (uint16_t)((last_upper_byte_rd << 8) | last_lower_byte_rd) ; //total upper+lower
        
    }
    if ( last_value_status == 0xAA ) 
    {
       last_value = 0;
    }
}


void check_and_set_max()
{
    /* IF INITIALLY INC_SW IS PRESSED THEN SET MAX VALUE (MANDATORY FOR FIRST TIME) */

    uint8_t max_value_setting_selected = false;

    if (is_inc_switch_pressed())
    {
        __delay_ms(5000);
     
        while(is_inc_switch_pressed())         // Check if _INC_SWITCH_ is still pressed after 5 sec
        {
           
           display_SET_command();
           max_value_setting_selected  = true;
        }
    }
    
        
        
    if(max_value_setting_selected)
    {   
        //GIE = 1;    // enable global interrupts
        curr_value = 0;
        max_value = 10000;
        while(true)
        {
            uint8_t some_value_set = false;

            if(is_inc_switch_pressed())
            {
                get_and_set_increment();
                some_value_set = true;
            }

            if(is_dec_switch_pressed())
            {
                get_and_set_decrement();
                some_value_set = true;
            }
            
            display_digit(curr_value);
            
            if (some_value_set)
            {
                uint16_t max_eeprom = curr_value; 
                uint8_t max_lower_byte_wr, max_upper_byte_wr = 0 ;
                
                max_lower_byte_wr =  max_eeprom & 0xFF ;          // Get the lower 8 bits
                max_upper_byte_wr = (max_eeprom >> 8) & 0xFF ;    // Shift right by 8 bits and get the lower 8 bits
                eeprom_write(0,max_lower_byte_wr);               // lower byte at 0
                eeprom_write(1,max_upper_byte_wr);               // upper byte at 1
                some_value_set = true;
            }

        }   
    }
}

void set_factory_settings()
{

    /* IF INITIALLY DEC_SW IS PRESSED THEN GIVE OPTION TO STORE LAST VALUE */
    uint8_t factory_setting_mode_enabled = false;
    if (is_dec_switch_pressed())
    {
        
        __delay_ms(5000);
        

        while(is_dec_switch_pressed())         // Check if _INC_SWITCH_ is still pressed after 5 sec
        {
           			// disable global interrupts
            factory_setting_mode_enabled = true;
          
            display_FR_command();
        }
    }
    
    
    
    if (factory_setting_mode_enabled)
    {
        uint8_t f_store_last_value ;
        
        while(true)
        {   uint8_t i,j = 0;
            while(true)
            {   
                GIE = 0;
                PORTC = 0b1101110;          // display y (yes) -- if selected then store last value
                _DIGIT_1s_ = 0;
                __delay_ms(1);
                _DIGIT_10s_ = _DIGIT_100s_ = _DIGIT_1000s_ = 1;
                
                if (i == 0 )
                {
                    f_store_last_value = 0x3F;
                    eeprom_write(3, f_store_last_value);
                    ++i;
                }    
                if(is_inc_switch_pressed())
                {
                    break;
                }
            }

            while(true)
            {
                GIE = 0;
                PORTC = 0b1010100;          // display n (no) -- if selected then don't store last value
                _DIGIT_1s_ = 0;
                __delay_ms(1);
                _DIGIT_10s_ = _DIGIT_100s_ = _DIGIT_1000s_ = 1;
                
                f_store_last_value = 0xAA;
                eeprom_write(3, f_store_last_value);
                
                if (j == 0 )
                {
                    f_store_last_value = 0xAA;
                    eeprom_write(3, f_store_last_value);
                    ++j;
                
                }
                if (is_dec_switch_pressed())
                {
                    break;
                }
            }
        }
    } 

}

void init()
{   
    duty_cycle = 0;
    pwm_value = 0;
    is_pwm_loaded = false;
    
    init_port();
     
    initPwm();
    
    init_timer();
    
    check_and_set_max();

    set_factory_settings();
    
    init_EEPROM();

    for (uint8_t counter = 0 ; counter < 5 ; counter ++)
    {   
        display_digit(max_value);
        __delay_ms(800);    
    }
    
    
}

/**
 * @brief main enrty point
 *
 * @param none
 *
 * @return none
 */


void main()
{
    uint8_t any_switch_pressed = false;
    display_digit(0);
    init();
    curr_value = last_value;
    while (true)
    {
        /* check if increment/decrement switch pressed */
        if(is_inc_switch_pressed())
        {
            get_and_set_increment();
            any_switch_pressed = true;
        }
        else if(is_dec_switch_pressed())
        {
            get_and_set_decrement();
            any_switch_pressed = true;
        }
        
        display_digit(curr_value);
        
        //if(any_switch_pressed)     /* if increment/decrement switch pressed; update new pwm values */
       // {
            duty_cycle = map_value_to_pwm_resolution(curr_value,max_value);
            load_pwm(duty_cycle);
        //}
        
        
        if ( last_value_status == 0x3F && (last_value != curr_value) )       /* if last store is enabled and value not stored already; update in eeprom */
        {
            uint8_t last_lower_byte_wr, last_upper_byte_wr ; 

            last_lower_byte_wr =  curr_value & 0xFF ;          // Get the lower 8 bits
            last_upper_byte_wr = (curr_value >> 8) & 0xFF ;    // Shift right by 8 bits and get the lower 8 bits
            eeprom_write(4,last_lower_byte_wr);                    // lower byte at 4
            eeprom_write(5,last_upper_byte_wr);                    // upper byte at 5
        }
        
        last_value = curr_value;
        
        if (!(is_pwm_loaded) && (last_value_status == 0x3F))
        {
            duty_cycle = map_value_to_pwm_resolution(curr_value,max_value);
            load_pwm(duty_cycle);

            is_pwm_loaded = true;
        }
               
        any_switch_pressed = false;    // Reset
    }

}
