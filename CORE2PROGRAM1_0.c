#include <18f8722.h> //<> drivermap first "" local map first
#DEVICE HIGH_INTS=TRUE
#DEVICE ADC=10				//ADC returns 10 bytes
#DEVICE PASS_STRINGS=IN_RAM  // Compiler will generate string pointers to RAM memory(new V5 compilers only)
//#ZERO_RAM
#fuses HS,H4,NOWDT,NOMCLR		// H4 is PLL
#include "Plecotrons.h" //local folder
#include <string.h>
#include <stdlib.h>

#use delay(clock=clockf, crystal=clockf_4)	// primary used in most included libraries

//#device ICDdebug = true

//SECONDARIES
#use i2c(master, I2C1,NOINIT,STREAM=IO) //SDA=PIN_C4 SCL=PIN_C5 init manual later
#use i2c(master, I2C2,NOINIT,STREAM=FRONT)// 
#use rs232(UART2,baud=9600,PARITY=N,RTS = Pin_E7,CTS = Pin_E6,TXISR,RECEIVE_BUFFER = 32,TRANSMIT_BUFFER = 32,STREAM=USB)//

#byte PORTD = getenv("SFR:PORTD")	//Can use PORTD as variable to read w/o changing tris (realtime pinlevel)
#byte PORTE = getenv("SFR:PORTE")
#byte LATH = getenv("SFR:LATH")		//reading the port variable for label accessing  and bit_set(VARlbl,bit) latchlevel
#byte LATJ = getenv("SFR:LATJ")         
#byte LATD = getenv("SFR:LATD")		
#byte LATF = getenv("SFR:LATF")

#include <modbus2.c>
#include "TESTER_modbus_slave_interrupts.c"

int16 *portpoint = getenv("SFR:PORTD"); // memory address of first input port

short futilebit = 0;
int8  futileint = 0;

//I2C IO
   	int8 IO_Expander_addresses[2]={0xE8,0xEA};   // out 17-32 , in 1-16
	int8 LED_Expander_addresses[4]={0xE8,0xEA,0xEE,0xEC};  // LEDS 1-16 17-31 49-64 32-48
	int8 RGB_address = 0xC4;
	int8 ANIN_address = 0x90;
 	int8 Expander_mask_buffer_1[4];
	int8 Disp_address = 0x50;	
	unsigned int8 current_modbus_address;
	short I2C_display_OK,I2C_in_expander_OK,I2C_out_expander_OK,I2C_rgb_OK,I2C_AN_expander_OK,I2C_ext_OK;
	unsigned int8 I2C_led_expander_OK;
	int16	rgb_sequence[2][5];
	short	RGB_sequence_changed;
	int8	RGB_sequence_slot;

// MODBUS IO

	int8 coils[Q_COIL_BANKS];   //0x// (32 outputs/8coils) 4 direct + 4 requests + 4 virtual 0x
	int8 test_coils[Q_COIL_BANKS];
	int8 inputs[Q_INPUT_BANKS]; //1x// 16 inputs 8 direct and 8 I2C or 16 I2C
   	int16 input_regs[Q_INPUT_REGS] ; //3x// analog inputs
   	int16 hold_regs[Q_HOLDING_REGS] ;	//4x// analog outputs / memory

// IO BUFFERS/FLAGS

	unsigned long analog_value[16];
	int I2C_inputs[2];
	short button_pressed;
	short displaying_init;
	//int32 minitimer[4];
	int8 I2C_fail_out_cnt,I2C_fail_in_cnt,I2C_fail_AN_cnt,I2C_fail_RGB_cnt,I2C_fail_LCD_cnt,I2C_fail_led_cnt;


//-------- MENU -----------//
//(It's a RAMeater)
struct menu_s	menu_root ;

unsigned int8 menu_depths_init[MAIN_MENU_DEPTH] = 				{SUB_MENU_DEPTH_0,
																SUB_MENU_DEPTH_1,
																SUB_MENU_DEPTH_2,
																SUB_MENU_DEPTH_3,
																SUB_MENU_DEPTH_4,
																SUB_MENU_DEPTH_5};
unsigned int8 arr_choices[MAIN_MENU_DEPTH][SUB_MENU_DEPTH] = {	{CHOICES_0},
																{CHOICES_1},
																{CHOICES_2},
																{CHOICES_3},
																{CHOICES_4},
																{CHOICES_5} 
															  };

signed int8 menu_stack[3]; //signed is easy to reversecount and loopback to max at "smaller than zero"
unsigned int8 active_stack = 0;//active counter
//unsigned int8 main_selector;
//unsigned int8 menu_depth;
//unsigned int8 item_nr;
short count_up,count_down,wait_for_count,confirm,save_request,save_it;

//------------------------//
			
unsigned char return_buffer[17];



//char vowels[6] = {'A','E','I','O','U','Y'};
//COMM HW
extern int8 t_next_in; 
extern int8 t_next_out;
extern int8 next_in ; 
extern int8 next_out ;
extern int8 r_buffer[R_BUFFER_SIZE];
extern int8 t_buffer[T_BUFFER_SIZE]; 
//extern char c;
//char c2;
short usb_state;

int init_time_modbus_detect;
short modbus_detected;
short modbus_init_time_passed;
int modbus_time;
int8 coils_timeouts_arr[32] ;

//int8 modbus_settings.address = MODBUS_ADDRESS ;  //safety default set > 0
int16 modbus_baudrates[7] = {96,144,192,384,576,762,1152};//*100
//int16 modbus_turnaround;
struct settings_1 modbus_settings;
short led_test,test_choice,plc_test;
 
//COMMAND CHECKER
///int match;
	unsigned int curr_str_len = 0;
	int inter_buffer_1[BUFFER_SIZE];
	int command_type ; //0 = pure int, 1 = command, 2 = garbage
	short neg_sign = 0;
	unsigned int command_idx = 0;
	unsigned int ASCII_command_idx = 0;
	unsigned int var_idx = 0;
int	command_size_ctr;
int item_count;
int old_command_size ;
int new_command_size;
///int trigger_index; 
///int Triglen_abs;
///char cmd_buf[COMMAND_BUFFER_SIZE];
char trigger_char[17]; 
short new_char,new_command;
///short exit_comp;
///int add_match_value;
int8 ib_next_in,ib_next_out;
int8 inter_buffer_idx;
	long var[VAR_DEPTH];
	char cmd[COMMAND_DEPTH][16];
	char ASCII_cmd[3];

// FATI FUNCTION

struct DF_time time_A ; // rain sensor switchover asymmetric hysterisis time left minutes and seconds
struct DF_time time_B ;	//screen maximum run time left
struct DF_time time_C ;	//clock
unsigned int PRIO,last_PRIO;
short bypass_delay_timer; // used for manual control override
short enable_percip_reaction_delay,do_precip_timer_loop;
short local_rain_flag,open_flag,close_flag,time_rain_started_enable,time_rain_stopped_enable;
short local_rain_decision_enable;
short keep_open_timer_initialized,keep_closed_timer_initialized;
short can_open,can_close,run_close,run_open;
short direction;
short button_done;
short lockdown;
//short modbus_emergency;
short emergency_switch_event,local_emergency_flag;
short screen_is_opened,screen_is_closed;
int error;
int inhibit_error_nr;
int32 cnt[32];
int8 cnt_start[4];
long clock_1_value,clock_2_value;
short max_run_overflow;
short  LOCAL_manual_PRIO_1_busy, HMI_manual_PRIO_2_busy, HMI_weather_PRIO_3_busy, LOCAL_weather_PRIO_4_busy;
int request_tokens;
short HMI_weather_open_request,HMI_weather_close_request;
short HMI_manual_close_request, HMI_manual_open_request;
short HMI_close_request_flag, HMI_open_request_flag;  
short manual_open_flag, manual_close_flag;
short LOCAL_manual_close_request, LOCAL_manual_open_request;
short LOCAL_weather_open_request,LOCAL_weather_close_request;
short open_time_overrun,close_time_overrun;
short manual_request;
short manual_button;
short percip_event_flag;

//state_t Manual_state,Local_state,Auto_state,HMI_state; // Represents this states as ON OFF now
enum main_menu_enum{PLC,DISPLAY,MOD_SETTINGS,OVERRIDES,IO_STATES};
enum baud_rates_enum{_9600,_14400,_19200,_38400,_57600,_115200}; //0-5
enum parity_enum{NONE,EVEN,UNEVEN};
enum overrides_enum{num_LCD_TEST,num_PLC_TEST};


int loop_count;
int32 runner;
extern short do_1_int;
int16 tester,tester2,tester_temp;
extern short sample_RGB_int;
unsigned int16 rgb_count;
//int16 analog_read;


int8 minitimer[8];


int8 swap_bits(int8 c)
{
   return ((c&1)?128:0)|((c&2)?64:0)|((c&4)?32:0)|((c&8)?16:0)|((c&16)?8:0)|((c&32)?4:0)|((c&64)?2:0)|((c&128)?1:0);
}
unsigned int16 tick_difference(unsigned int16 current, unsigned int16 previous) 
   {
   return(current - previous);
   }
   

#include "testprogramCORE2.c" 
#include "PLC_menudisplay.c"

#include "FATI2_modbus_slave_init.c"

void main()
{
	tester = 1 ;//runninglight test var
	tester2 = 0x000F ;//runninglight test var
	loop_count = 5;
	init();

	while(TRUE)
	{
		modbus_detected = check_MODBUS_active();
		stack_inputs();
		hold_regs[0] = ((((int16)inputs[1])<<8)+inputs[0]);	//Holding register 0 has representation of 16 digital inputs //// OPLOSSEN OPLOSSEN//	  
		button_handler(); //part of tester
		if(sample_analogs)	
		{
			stack_analog_inputs();
			sample_analogs = 0;
		}
		if (hold_regs[31])
		{
				test_program(); // will run when no buttons pressed for a while
		}		
		PLC_function();

		while(next_in != next_out)  //data in incoming buffer USB
		{
        	Data_processing();
		}//while n_i
	}//while 1
}//main

void PLC_function() // timing handling
{
	if(time_to_sync_indicator)
	{
		if(bit_test(MATCH_FLAG_COIL)) // CONSTANT COIL! NOT TEMP REQUEST FLAG
		{
			rgb_sequence[0][4] = 0x00F0; //GREEN
			rgb_sequence[1][4] = 0x800C;
		}
		else
		{
			rgb_sequence[0][4] = 0x0000; //	NO COLOR
			rgb_sequence[1][4] = 0xFF00;	//FULL ENABLE
		}
		RGB_sequence_slot ++;
		if(RGB_sequence_slot > 4)
			RGB_sequence_slot = 0;

		set_RGB(1,rgb_sequence[0][RGB_sequence_slot],(int8)(rgb_sequence[1][RGB_sequence_slot]>>8),(int8)(rgb_sequence[1][RGB_sequence_slot]));
		time_to_sync_indicator= 0;
	}
	if(!led_test)
		set_front_leds(inputs,0,2);
	//	represent_inputs(inputs[0],inputs[1],0,0); // indication on front leds
	else
		set_front_leds(test_coils,0,8);
//		represent_inputs(test_coils[0],test_coils[1],test_coils[2],test_coils[3]);		

	if(refresh_display_int) //show init or handler
	{
		if(displaying_init) //display init 
		{
			if( displaying_init_timeout)
				displaying_init_timeout--;
			else
			{
				displaying_init = 0;
				displaying_init_timeout = 4;
			}
		}
		else //if (done_display_init)
		{
			display_handler();
		}
		refresh_display_int = 0;		
	}
	if(FATI_FUNCTION)
	{	
		lock_coils_direct_access = TRUE;	
			//TEST FOR EMERGENCY INTERLOCK AND HANDLE
		local_rain_decision_enable = (!modbus_detected && modbus_init_time_passed );		//determine standalone operation

		if (local_emergency_flag == bit_test(LOCAL_EMERGENCY_BUTTON))	// watch out INVERSION cancels out to ==
		{
			emergency_switch_event = 1;	// events flags change in status
			local_emergency_flag = !bit_test(LOCAL_EMERGENCY_BUTTON);		// INVERSION !!
		}
		lockdown_handler();				// DETERMINE EMERGENCY AND ACT ON OUTPUTS WHEN EMERGENCY 

			// TEST SCREEN SENSORS

		screen_is_closed = !bit_test(CLOSE_SENSOR);
		screen_is_opened = !bit_test(OPEN_SENSOR);
			// COIL RUN HANDLING
		coils_timeouts_arr[0] = coils_timeouts_arr[1] = 0;	//no auto timeout for open/close request

		poll_coil_deadman_timers();		// perform coil timeouts for valves
		poll_requests(); 				// resample requests parallel 
		pool_requests();				// execute conditionally in serial manner and set timer inhibits
			//CONDITIONAL POST TIMER SET BY enable_ AND LOCKED IN BY do_
		if ( (enable_percip_reaction_delay || do_precip_timer_loop)  )			//  enables percipitation delay timer	
		{										// do_loop keeps post timer flowing
			enable_percip_reaction_delay = 0;
			post_timer_handling(direction);		// post timers used to delay requests in pooling
												// Direction given by last motor direction
		}
		else
		{
			keep_open_timer_initialized = 0;	//will reset timer init
			keep_closed_timer_initialized = 0;
		}
		if(!lockdown)
		motor_run_handler();		// requests get executed here

	}
	else if (WATERCONTROL_FUNCTION)
	{
		lock_coils_direct_access = TRUE;
		if (local_emergency_flag == bit_test(LOCAL_EMERGENCY_BUTTON))	// watch out INVERSION cancels out to ==
		{
			emergency_switch_event = 1;	// events flags change in status
			local_emergency_flag = !bit_test(LOCAL_EMERGENCY_BUTTON);		// INVERSION !!
		}
		lockdown_handler();	
		poll_coil_deadman_timers();		// perform coil timeouts for valves
		
	}
	else
	{
		poll_coil_deadman_timers();

		lock_coils_direct_access = FALSE;
		output_J(coils[2]);
		output_H(coils[3]);
		I2C_START(IO);				
   
		if(I2C_WRITE(IO_Expander_addresses[0])==0)
		{
			I2C_out_expander_OK = 1;
			I2C_WRITE(0x02);	//r02
			I2C_WRITE(coils[0]);
			I2C_WRITE(coils[1]);
			i2c_stop(IO);
				
		}
		else
		{
			i2c_stop(IO);
			output_low(RESET_I2C_OUTPUTS);			
			I2C_out_expander_OK = 0;
			I2C_fail_out_cnt++;
			if ((int8)stamp100ms - minitimer > I2C_FAIL_MAX)
			{
				I2C_fail_out_cnt = 0;
				output_high(RESET_I2C_OUTPUTS);	
				init_out_expanders();
				minitimer[0] = (int8)stamp100ms;
			}
		}
	}	
	if(!led_test && !plc_test)
		set_front_leds(coils,4,4);
//		represent_outputs(coils[0],coils[1],coils[2],coils[3],0x00,coils[4],0);
	
	else
		set_front_leds(test_coils,4,4);
//		represent_outputs(test_coils[0],test_coils[1],test_coils[2],test_coils[3],test_coils[4],test_coils[5],1);
	
}//END void PLC_function()
	

void motor_run_handler()			
{
//can_ is only reset by this operation NOT SET as can_ must be one already
	can_close &= (!close_time_overrun &&(SCREEN_MAGNETICS_BYPASS || (!screen_is_closed && !bit_test(error,3)))) ;  // A one on close sensor(active) disarms close , sensor high is active
	can_open  &= (!open_time_overrun  &&(SCREEN_MAGNETICS_BYPASS || (!screen_is_opened && !bit_test(error,3)))) ;  // An active error will keep eventual can_open

	if (can_close && (!keep_open_timer_initialized || bypass_delay_timer) && !run_close) // initiate run close
	{
		bypass_delay_timer = 0;
		clock_2_value = RUNTIME_MAX;
		can_close = 0;
		run_close  = 1;
		for(int j = 0 ; j < 5;j++)	//ALL BLUE ONCE
		{
			if(rgb_sequence[0][j] > 0) 
			{	
				rgb_sequence[0][j] = 0x0F00; //BLUE
				rgb_sequence[1][j] = 0xFF00; //FULL CONTINUOUS
			}
		}
		run_open  = 0;
		direction = CLOSE;
		clock_2_int = 0;
	//	screen_is_opened = 0;
		inhibit_error_nr = 0b00000000;
		max_run_overflow = 0;	
		last_PRIO = PRIO;		
	}
	else if (can_open && (!keep_closed_timer_initialized || bypass_delay_timer) && !run_open)	// initiate run open
	{
		bypass_delay_timer = 0;
		clock_2_value = RUNTIME_MAX;
		can_open = 0;
		run_open = 1;
		for(int j = 0 ; j < 5;j++)	//ALL BLUE ONCE
		{
			if(rgb_sequence[0][j] > 0) 
			{	
				rgb_sequence[0][j] = 0x0CC0; //BLUE
				rgb_sequence[1][j] = 0xFF00; //FULL CONTINUOUS
			}
		}
		run_close = 0;
		direction = OPEN; 	
		clock_2_int = 0;
	//	screen_is_closed = 0;
		inhibit_error_nr = 0b00000000;	
		max_run_overflow = 0;
		last_PRIO = PRIO;
	}
	if (run_close || run_open)			//close has even bits open has uneven bits CANT BE BOTH
	{ 
		if (PRIO > 0 && !PRIO_handled_at_end)
		{
				switch (PRIO) // movement priority		
				{
					case 1:				// local manual
					{
					
					}
					break;
					case 2:				// hmi manual
					{
						HMI_manual_PRIO_2_busy = 0;
					}
					break;
					case 3:				//hmi event
					{
						HMI_weather_PRIO_3_busy = 0;
					}
					break;
					case 4:				//local event
					{
						LOCAL_weather_PRIO_4_busy = 0;
					}
					break;
			}	
		}

		
		if (clock_2_int && !local_emergency_flag)	// RUNNING OVERTIME CHECK
		{											// Emergency button inhibits timeout

			clock_2_int = 0;
			display_timer_2 = 1;
			time_B = fill_timer_value(clock_2_value);
			if ( clock_2_value == 0)
			{
				max_run_overflow = 1;
			}
			else
			clock_2_value--;
					
		}
		
		if (run_close)
		{
			if (max_run_overflow || (screen_is_closed && !SCREEN_MAGNETICS_BYPASS ) )// || bit_test(error,3) )
			{
				run_close  = 0;
				bit_clear(CLOSE_SCREEN);
				display_timer_2 = 0;
				if (max_run_overflow )
					close_time_overrun = 1;
			}
			else	// start run close physically
			{
				screen_is_opened = 0;
				bit_set(CLOSE_SCREEN);
				bit_clear(OPEN_SCREEN);
				open_time_overrun = 0; //closing resets open_overrun_flag
			}	
		}
		else //if(run_open)
		{ 
		if (max_run_overflow || (screen_is_opened && !SCREEN_MAGNETICS_BYPASS)) // || bit_test(error,3) )		// indicated both cases as open..???
			{
				run_open  = 0;
				bit_clear(OPEN_SCREEN);
        		display_timer_2 = 0;
				if (max_run_overflow)
					open_time_overrun = 1;
			}	
			else
			{
				screen_is_closed = 0;
				bit_clear(CLOSE_SCREEN);
				bit_set(OPEN_SCREEN)  ;
				close_time_overrun = 0;//opening resets close_overrun_flag
			}	
			inhibit_error_nr = 0b00000000;	
		}
	}	
	else		// lets use not running as a clear handler HMI_manual_PRIO_2_busy
	{
		bit_clear(OPEN_SCREEN);
		bit_clear(CLOSE_SCREEN);
		manual_request = 0;
		if(PRIO > 0 && PRIO_handled_at_end)
		{
			switch (PRIO)		
			{
				case 1:				// local manual
				{
					
				}
				break;
				case 2:				// hmi manual
				{
					HMI_manual_PRIO_2_busy = 0;
				}
				break;
				case 3:				//hmi event
				{
					HMI_weather_PRIO_3_busy = 0;
				}
				break;
				case 4:				//local event
				{
					LOCAL_weather_PRIO_4_busy = 0;
				}
				break;
			}
		}
	}
}	


void Data_processing()	
{
 	char char_in;
	struct rtn c_s_1;		
			
	for ( char_in = bgetc();	char_in != NULL;char_in = bgetc()) //pull as much as you can in one loop
	{// GET 1 by 1 from inbuffer
	
										
		if(char_in != LINEFEED)		///n 0x0A
		{	
			if( (char_in == 0x2D) ) // sign will not register when not pure digits
			{
				if(command_type == 0 && !neg_sign)
					neg_sign = 1;
				else
					command_type = 1;	//double sign casts from pure int to command
			}
			if(isprint(char_in) && char_in != 0x7F)	// is real character and not DEL
			{
				if (char_in != 0x20) //not space 
				{	
					if((!isdigit(char_in)) && command_type == 0)
						 command_type = 1;
										
					if (curr_str_len <= BUFFER_SIZE) 
					{
						curr_str_len++;
					}
					inter_buffer_1[ib_next_in] = char_in;
					if(ib_next_in++ >= BUFFER_SIZE) 
						ib_next_in = 0;
				}
			
				else if(curr_str_len > 0 )	// CHAR IS SPACE AND COMMAND WITH LENGTH = READY TO COMPARE AND CATALOG
				{							// interbuffer has a string of current command_length
					if(	command_type == 0)  // variable
					{	
					// come out with number of digits and sign
						c_s_1 = sint16_atoi(&inter_buffer_1[ib_next_out],curr_str_len,neg_sign);	//declaration of struct element

						if (c_s_1.OK )
						{
							//command_var1_ok = 1;
							var[var_idx] = c_s_1.value.s_value;
						
						}
						else
						{
						//	command_var1_ok = 0;
							var[var_idx] = 0;	
						}
						var_idx++;
						ib_next_out = (ib_next_out + curr_str_len)% BUFFER_SIZE;
						command_type = 0;
						curr_str_len = 0;
					
					}		
					else if(command_type == 1)
					{
						
						for(int n=0;n<curr_str_len;n++)
						{
							cmd[command_idx][n]=inter_buffer_1[ib_next_out];
						
							if (ib_next_out++ >= BUFFER_SIZE)
							ib_next_out = 0;
						}
						command_idx++;
						command_type = 0;
						curr_str_len = 0;
					}
	
				}
				//0length does nothing
			}//real char 
			else  //char is not a real string char
			{
				command_type = 2;
				ASCII_cmd[ASCII_command_idx]=inter_buffer_1[ib_next_out];
				
				if (ib_next_out++ >= BUFFER_SIZE)
					ib_next_out = 0;
						
					ASCII_command_idx++;
					command_type = 0;
					curr_str_len = 0;
			}

		}	//not linefeed
		else//LINEFEED start to interpret command
		{

		}
	}// char in = NULL quits loop	
}
	
void bputc(char c)  //write char to txreg
{
   	int ni;
	short restart;
  	restart=t_next_in==t_next_out;
   	t_buffer[t_next_in]=c;
   	ni=(t_next_in+1) % T_BUFFER_SIZE; // circular index
   	while(ni==t_next_out);//push head one from ass
   		t_next_in=ni;
   	if(restart)//in = out or buffer empty but index holding
      	enable_interrupts(INT_TBE2);
}

char bgetc() //read from rxreg Buffer filled by interrupt processing
{ 
char ch;
   	item_count = 0;
//	new_char = 0;
	if((next_in!=next_out))
  	{  
  	 	ch=r_buffer[next_out];
// 		new_char = 1;
		item_count++ ;
		if (++next_out == R_BUFFER_SIZE)
		{
        next_out=0;  
		}
  	}
	else 
	{
		ch = NULL;
	}
	return ch;
} 

int load_trigger(t_index)
{
	switch(t_index) 
	{
		case 0 :
		strcopy(trigger_char,Trigger1);   // match_bit 0 coupled
		break;
		case 1 :
		strcopy(trigger_char,Trigger2);	// match_bit 1 coupled
		break;
		case 2 :
		strcopy(trigger_char,Trigger3);
		break;
		case 3 :
		strcopy(trigger_char,Trigger4);
		break;
		case 4 :
		strcopy(trigger_char,Trigger5);
		break;
		case 5 :
		strcopy(trigger_char,Trigger6);
		break;
		case 6 :
		strcopy(trigger_char,Trigger7);
		break;
	}
	 return(strlen(Trigger_char));	
}

short check_modbus_active()
{
		if(init_time_modbus_detect > 0)			//starts counting at bootup == init busy
		{
			modbus_init_time_passed = 0;        // INIT still running
			if(init_time_int)						// timer INT
			{
				init_time_modbus_detect--;		
				init_time_int = 0;
			}
		return(0);
		}	
		else
		{
			modbus_init_time_passed = 1;
		}

		

		if (MODBUS_CONTROLLED)								//  enabled  in header(TRUE)
    	{
   			if (bit_test(BUS_COMMAND_COIL))						//virtual coil check for SET by address response
   			{
				modbus_time = MODBUS_INTERLOCK_TIMEOUT;					//   keep resetting the timervalue every time coil is set by HMI
				return (1);		
       	  	}

       		else
			{	
     			if(modbus_time > 0 )  							//start countdown
	       		{
					if (mod_count)		//	int_trigger 100ms
					{
						modbus_time--;
						mod_count = 0;
					}
				return(1);
	       		}
				else
				{		
					return (0);
	  	 		}     
			}
	    }
}

 
void poll_coil_deadman_timers()
{
	///////// ------------------- DEADMAN'S Watch on requests ----------------------///////
	/// PUTS REQUESTS INTO COIL SETS
	int c_idx,cnt_idx;

	for(int start_bank = 0;start_bank < 4;start_bank++)				// TEST COILBANK 4-7 and SET ONLY COILBANK 0-2, COILBANK 3 STATUSBANK(BUSLEDS)
	{	
		for (int bit_idx=0;bit_idx < 8;bit_idx++)				//	COILS[ 33- 56]
		{
		//	current_coil_number = 4*j + i;
			c_idx = bit_idx;
			cnt_idx = bit_idx +(start_bank*8);
			if (bit_test(coils[start_bank+4],(c_idx)))	// test request
			{
				cnt[cnt_idx] = STAMP100ms;		// set count to current stamp
	
		//		if(start_bank<3 || (c_idx  >= 5 && c_idx  < 7 && FATI_FUNCTION))		//restrict from busleds on start_bank3 (directly set) exept for busleds
					bit_set (coils[start_bank],c_idx);		// set real coil
					
				if(start_bank>0 || bit_idx >1 || !FATI_FUNCTION ) //RESTRICT FATI's OPEN CLOSE OUTPUTS
				{
					bit_clear(coils[start_bank+4],c_idx); 	// coilsstart_banks 0 -  3 are real, 4 - 7 requests
					bit_set (cnt_start[start_bank],c_idx );	// indicate timer enable
				}
			}
			if (bit_test(cnt_start[start_bank],c_idx ) && coils_timeouts_arr[cnt_idx])	//current timer running and	0 value disables timeout
			{
				if ((STAMP100ms - cnt[cnt_idx]) > coils_timeouts_arr[cnt_idx]) // coil timeouts
				{
					bit_clear(coils[start_bank],c_idx);
					bit_clear(cnt_start[start_bank],c_idx);	// ALL OUTPUTS/LEDS HAVE OWN dead time
				}
			}
		} 
	}
}  


void poll_requests()
{
//----------------------------------------REQUEST POLLING------------------------------------------------------------------//
	HMI_close_request_flag = bit_test(CLOSE_REQUEST_COIL);		//----------------------- !!	HMI HERPROGRAMMEREN  !! 27/05/19
	bit_clear(CLOSE_REQUEST_COIL);
	HMI_open_request_flag = bit_test(OPEN_REQUEST_COIL);		// voor deze ook
	bit_clear(OPEN_REQUEST_COIL);
	manual_open_flag = bit_test(MANUAL_OPEN);
	manual_close_flag = bit_test(MANUAL_CLOSE);
	percip_event_flag =  bit_test(PERCIP_EVENT_COIL);
	bit_clear(PERCIP_EVENT_COIL);
	
	if (HMI_close_request_flag || HMI_open_request_flag)
	{
//		source = HMI;
		if(percip_event_flag) //by rain
		{
			if (PRIO > 3)
			{
				PRIO = 3; // lift PRIO if needed
			}
			if(HMI_close_request_flag)
			{
				HMI_weather_close_request = 1;
				HMI_weather_open_request = 0;	
				(request_tokens &= 0b11001111)|= 0b00010000;
				HMI_close_request_flag = 0;
			}
			else if (HMI_open_request_flag)
			{
				HMI_weather_close_request = 0;
				HMI_weather_open_request = 1;
				(request_tokens &= 0b11001111)|= 0b00100000;
				HMI_open_request_flag = 0;
			}
	
			percip_event_flag = 0;
		}
		else  // not by rain
		{			
			if (PRIO > 2)
			{
				PRIO = 2;
			}
			if(HMI_close_request_flag)
			{
				HMI_manual_close_request = 1;
				((request_tokens &= 0b11110011)|= 0x04);
				HMI_manual_open_request = 0;
				HMI_close_request_flag = 0;
			}
			else if (HMI_open_request_flag)
			{
				HMI_manual_close_request = 0;
				HMI_manual_open_request = 1;
				((request_tokens &= 0b11110011)|= 0x08);
				bit_clear(OPEN_REQUEST_COIL);
				HMI_open_request_flag = 0;
			}
		}
		
	}	// SO FAR FOR HMI REQUEST POLLING

	if((manual_close_flag || manual_open_flag) && !button_done)
	{	
	//	source = LOCAL;
		if(manual_close_flag )
		{	
			manual_button = 1;
			LOCAL_manual_close_request = 1;
			LOCAL_manual_open_request = 0;
			((request_tokens &= 0b11111100)|=0x01);
			PRIO = 1;
			button_done = 1;
		}
		else if(manual_open_flag )
		{	
			manual_button = 1;
			LOCAL_manual_close_request = 0;
			LOCAL_manual_open_request = 1;
			((request_tokens &= 0b11111100)|=0x02);
			PRIO = 1;
			button_done = 1;
		}
	}
 	if (!manual_close_flag && !manual_open_flag && button_done) 
		{
			display_timer_2 = 0;
			manual_button = 0;
			button_done = 0;
			LOCAL_manual_PRIO_1_busy = 0;
			run_close = run_open = 0;		// run will stop when button in released
			
		}

	//----------------------------------------LOCAL RAIN DECISION---------------------------------------------------------------------//


		if (local_rain_flag == bit_test(RAIN_SENSOR))  // autonomous rain decision, normally inversed so equal = change
			{
				local_rain_flag = !bit_test(RAIN_SENSOR); // reinit inverse
	
				if(local_rain_flag)
				{
					LOCAL_weather_close_request = 1;
					(request_tokens &= 0b00111111)|= 0x40;
					LOCAL_weather_open_request = 0;
				} 	
				else
				{
					LOCAL_weather_open_request = 1;
					((request_tokens &= 0b00111111)|= 0x80);
					LOCAL_weather_close_request = 0;
				} 	

			}
	
// REQUESTS ARMED
} 

void pool_requests()
{
	switch (PRIO)
	{
		case  1:
		{
			if(LOCAL_manual_close_request)									// MANUAL REQUEST applies directly without weather hysterisis
			{
				can_open = 0; // disable request
				can_close = 1;	// close request
				LOCAL_manual_close_request = 0;  //single request handling
				LOCAL_manual_PRIO_1_busy = 1;
				manual_request = 1;
				bypass_delay_timer = 1;
				break;	
			}	
			else if (LOCAL_manual_open_request)
			{		
				can_open = 1;
				can_close = 0;	
				LOCAL_manual_open_request = 0;
				LOCAL_manual_PRIO_1_busy = 1;
				manual_request = 1;
				bypass_delay_timer = 1;
				break;		// skip futher case check
			}
			if( !LOCAL_manual_PRIO_1_busy && !manual_button )				// handled when local_button goes 0
			{
				PRIO++;	  //no break trough here so up to case 2 immediately
			}
			else
			break;							
		}						
		case  2 :
		{
			if(HMI_manual_close_request)
			{	
				can_open = 0;
				can_close = 1;
				HMI_manual_close_request = 0;
				HMI_manual_PRIO_2_busy = 1;
				manual_request = 1;
				bypass_delay_timer = 1;
				break;
			}
			else if(HMI_manual_open_request)
			{
				can_open = 1;
				can_close = 0;
				HMI_manual_open_request = 0;
				HMI_manual_PRIO_2_busy = 1;
				manual_request = 1;
				bypass_delay_timer = 1;
				break;
			}
			if(!HMI_manual_PRIO_2_busy)				// handled when operation end by sensor or overrun 
			{
				PRIO ++;
			}
			else
			break;
		}
		case 3 :
		{
			if(!local_rain_decision_enable)			// handled after weather hysterisis delay
			{
				if(HMI_weather_close_request) 
				{
					keep_closed_timer_initialized = 0;
					enable_percip_reaction_delay = 1;
					can_open = 0;
					can_close = 1;
					HMI_weather_close_request = 0;
					HMI_weather_PRIO_3_busy = 1;									
					manual_request = 0;
					
					break;		
				}
				else if(HMI_weather_open_request)
				{
					keep_open_timer_initialized = 0;	
					enable_percip_reaction_delay = 1;				
					can_open = 1;
					can_close = 0;
					HMI_weather_open_request = 0;
					HMI_weather_PRIO_3_busy = 1;
					manual_request = 0;

					break;
				}
				else
				break;
			}
			else
			{
				HMI_weather_PRIO_3_busy = 0;
				PRIO ++;						//skip to next with NO BREAK
			}
		}
		case 4 :
		{
			if(!local_rain_decision_enable)
			{
			PRIO = 3;
			LOCAL_weather_PRIO_4_busy = 0;
			}
			else
			{	
				if(  !HMI_weather_PRIO_3_busy)
				{
					if(LOCAL_weather_close_request &&  !keep_open_timer_initialized)
					{
						can_open = 0;
						can_close = 1;
						LOCAL_weather_close_request = 0;
						LOCAL_weather_PRIO_4_busy = 1;
						enable_percip_reaction_delay = 1;
						manual_request = 0;
					}
					else if(LOCAL_weather_open_request && !keep_closed_timer_initialized)
					{	
						can_open = 1;
						can_close = 0;
						LOCAL_weather_open_request = 0;
						LOCAL_weather_PRIO_4_busy = 1;
						enable_percip_reaction_delay = 1;
						manual_request = 0;
					}
				}
			}
		break;
		}
		default :
			PRIO = 4;
		break;
	}
} 

void clear_busy_PRIOS()
{
//	LOCAL_manual_PRIO_1_busy = 0;   //  only by button
	HMI_manual_PRIO_2_busy = 0;
	HMI_weather_PRIO_3_busy = 0;
	LOCAL_weather_PRIO_4_busy = 0;
}

void post_timer_handling(int dir)
{
	if (dir)	// 1 IS DROOG sensor IS OPEN 
	{				
		if (!keep_open_timer_initialized)		// inits new event time reset
		{
			clock_1_value = AFTER_SUN_DELAY;		//Load time
			time_A = fill_timer_value(clock_1_value);					
			keep_open_timer_initialized = 1;		//init stop done
			keep_closed_timer_initialized = 0;		//init start reset
			clock_1_int = 0;
		}								// NO BREAK FOR GOING DIRECTLY TO CASE TRUE
		//INIT DONE	
		do_precip_timer_loop = 1;				// LOCK LOOP until return reset on timer empty
		if (clock_1_int)				// every second
		{
			if(clock_1_value > AFTER_SUN_DELAY)			//	 ONCE HAD A OVERFLOW PROBLEM 4/06/19
			{
				clock_1_value = AFTER_SUN_DELAY;
			}	
			if (clock_1_value > 0)
			{
				clock_1_value--;
			}
				clock_1_int = 0;
    			display_timer_1 = 1;
				time_A = fill_timer_value(clock_1_value);
		}
		if (clock_1_value == 0)		// immediate response undependent on clock_int
		{			
			keep_open_timer_initialized = 0;
			do_precip_timer_loop = 0;			//UNLOCK LOOP		
			
		}
	}
	
	else	// 0 IS NAT sensor is GESLOTEN
	{		
		if (!keep_closed_timer_initialized)
		{
			keep_open_timer_initialized = 0;		//init stop reset
			clock_1_value = AFTER_RAIN_DELAY;		//Load time								
			time_A = fill_timer_value(clock_1_value);					
			keep_closed_timer_initialized = 1;		//init start done
			clock_1_int = 0;
		}	// NO BREAK GOTO TRUE
				//INIT DONE	
		if(clock_1_value > AFTER_RAIN_DELAY)		//	 HAD SOME OVERFLOW PROBLEM 4/06/19
		{
			clock_1_value = AFTER_RAIN_DELAY;
		}
		do_precip_timer_loop = 1;
		if (clock_1_int )				// every second
		{
			if (clock_1_value > 0)
				clock_1_value--;	

				clock_1_int = 0;
				display_timer_1 = 1;
				time_A = fill_timer_value(clock_1_value);
		}
		if (clock_1_value == 0)
		{
			keep_closed_timer_initialized = 0;
			do_precip_timer_loop = 0;
			display_timer_1 = 0;
		}
		
	}

}			

struct DF_time fill_timer_value(long timer_seconds)
{
	struct DF_time x ;		// local variable x with DF_elements
	x.hours = (timer_seconds / 3600);
	x.minutes = ((timer_seconds / 60)-(x.hours*60));
	x.seconds = timer_seconds - (x.minutes*60);
	return x;
}

struct rtn sint16_atoi(char *begin_char_ptr, unsigned int digits,short negative)  // declaring return of function in a struct c_s fashion
{
	struct rtn C_S_ATOI; 	//dumpstruct, returned when sint16_ATOI is called
	signed int16 intercalc;	//tempcalc

	*(begin_char_ptr + digits) = '\0'; //nakijken voor overflow

	if (digits == 5)
	{
		if((atol(begin_char_ptr+1)) > (2767 + (int)negative))	 //convert last 4 and check for > 2767 / -2768  
		{
			if(*begin_char_ptr <= '2')
			{
				intercalc = atol(begin_char_ptr);
				C_S_ATOI.OK = 1;
			}
			else
			{
				return(0);	//	out of 16 bit range
			}		
		}
		else if(*begin_char_ptr <= '3')
		{
			intercalc = atol(begin_char_ptr);
		}	
		else
		//	C_S_ATOI.OK = 0;
			return(0);
			
	
	}
	else if (digits < 5)
	{
		intercalc = atol(begin_char_ptr);
		C_S_ATOI.OK = 1;
	}

	if(negative)
		intercalc = -intercalc;	
	
	C_S_ATOI.value.s_value = intercalc;
	C_S_ATOI.OK = 1;

	return (C_S_ATOI);
}

void stack_inputs()
{
		inputs[0] = ((~PORTE & 0x0f)<<4) | (~PORTD & 0x0f);	//Merge bits D0-3 and E0-3
		//POLL AND SET INPUTS AND INPUT REGISTER
		I2C_START(IO);
   		if(	I2C_WRITE(IO_Expander_addresses[1]) ==0)
		{
			I2C_in_expander_OK = 1;
			I2C_WRITE(0x00);		//input port
			I2C_START(IO,2);
			I2C_WRITE(IO_Expander_addresses[1]+1);
			I2C_READ(IO); // do blank on first
			inputs[1] = ~I2C_READ(IO,0);		
		}
		else
			I2C_in_expander_OK = 0;

		i2c_stop(IO);
	//	inputs[1] = I2C inputs port IO1_n
}
void stack_analog_inputs()
{
	
			for (int n = 0;n < 8;n++)//10 bitter on-chip
		{
			set_adc_channel(n+4);                // the next read_adc call will read channel 0
			delay_us(20); 
			read_adc(ADC_START_ONLY);
			delay_ms(10);
          	// a small delay is required after setting the channel  and before read
			analog_value[n]=read_adc(ADC_READ_ONLY);	
		}

	for (int n =0;n<8;n++)//12 bitter i2c
	{
		I2C_START(IO);		
   		if(	I2C_WRITE(ANIN_address & 0xFE)==0)	
		{
			I2C_AN_expander_OK = 1;
			I2C_WRITE(0x84 | (n<<4));	//r02
			I2C_START(IO);
			I2C_WRITE(ANIN_address | 0x01);
			I2C_inputs[0]= I2C_READ(IO,1);
			I2C_inputs[1] = I2C_READ(IO,0);
			analog_value[8+n] = ((int16)I2C_inputs[0]<<8) + I2C_inputs[1];
		}
		else
		{
			I2C_AN_expander_OK = 0;
			analog_value[8+n] = 0;
		}
		i2c_stop(IO);
		delay_ms(1);
	}
	for (int q =0;q<16;q++)
		{
			input_regs[q] = analog_value[q];
		}
}
/*void represent_inputs(int c_byte0,int c_byte1,int c_byte2,int c_byte3)
{
	I2C_START(FRONT);		
   	if(I2C_WRITE(LED_Expander_addresses[0])==0)		//top leds
	{	
		bit_set(I2C_led_expander_OK,0);
		I2C_WRITE(0x02);	//r02
		I2C_WRITE(~c_byte0);
		I2C_WRITE(~c_byte1);	
		i2c_stop(FRONT);	
	}	

	else	
	{
		i2c_stop(FRONT);		
		bit_clear(I2C_led_expander_OK,0);
		I2C_fail_led_cnt++;
		if ((int8)stamp100ms - minitimer[3] > I2C_FAIL_MAX)
		{
			I2C_fail_out_cnt = 0;
			init_led_expanders();
			minitimer[3] = (int8)stamp100ms;
		}
	}

	
}
*/
void represent_outputs(int c_byte0,int c_byte1,int c_byte2,int c_byte3,int c_byte4,int c_byte5,short direct) // LED OUTPUTS ON FRONT
{
	short general_fail = 0;
	I2C_START(FRONT);	
	
   	if(I2C_WRITE(LED_Expander_addresses[3])==0)		//bot leds
	{	
		bit_set(I2C_led_expander_OK,2);
		I2C_WRITE(0x02);	//r02
		if (direct)
		{
			I2C_WRITE(~c_byte2);
			I2C_WRITE(~c_byte3); 
		}
		else
		{
			I2C_WRITE(lockdown?(~c_byte2 | PINSJ_AFFECTED_BY_EMERGENCY)	 : ~c_byte2); 
			c_byte3 =(c_byte3<< 4) | (c_byte3 >> 4);
			I2C_WRITE(lockdown?((~c_byte3) | PINSH_AFFECTED_BY_EMERGENCY) : ~c_byte3);		
		}
	i2c_stop(FRONT);	
	}	
	else
	{
		i2c_stop(FRONT);
		bit_clear(I2C_led_expander_OK,2);
		general_fail = 1;
		if ((int8)stamp100ms - minitimer[3] > I2C_FAIL_MAX)
		{
			I2C_fail_out_cnt = 0;
			init_led_expanders();
			minitimer[3] = (int8)stamp100ms;
		}
	}

	I2C_START(FRONT);		
   	if(I2C_WRITE(LED_Expander_addresses[2])==0)		//bot leds
	{	
		bit_set(I2C_led_expander_OK,3);
		I2C_WRITE(0x02);	//r02
		if (direct)
		{
			I2C_WRITE(~c_byte0);
			I2C_WRITE(~c_byte1); 
		}
		else
		{
			I2C_WRITE(lockdown?(~c_byte0 | I2C_PORT0_AFFECTED_BY_EMERGENCY)	 : ~c_byte0); 
			I2C_WRITE(lockdown?(~c_byte1 | I2C_PORT1_AFFECTED_BY_EMERGENCY)	 : ~c_byte1); 	
		}
		i2c_stop(FRONT);
	}	 
	else
	{
		i2c_stop(FRONT);
		bit_clear(I2C_led_expander_OK,3);
		general_fail = 1;
	}
I2C_START(FRONT);		
   	if(I2C_WRITE(LED_Expander_addresses[1])==0)		//OFF
	{	
		bit_set(I2C_led_expander_OK,1);
		I2C_WRITE(0x02);	//r02
		I2C_WRITE(~c_byte4);
		I2C_WRITE(~c_byte5);	
		i2c_stop(FRONT);
	}
	else	
	{
		i2c_stop(FRONT);		
		bit_clear(I2C_led_expander_OK,1);
		I2C_fail_led_cnt++;
		general_fail = 1;
		
	}
if 	(general_fail)
	{
		
		bit_clear(I2C_led_expander_OK,2);
		bit_clear(I2C_led_expander_OK,3);
		output_low(RESET_I2C_OUTPUTS);							
		I2C_fail_led_cnt++;
				if ((int8)stamp100ms - minitimer[3] > I2C_FAIL_MAX)
				{
					I2C_fail_out_cnt = 0;
					output_high(RESET_I2C_OUTPUTS);	
					init_out_expanders();
					general_fail = 0;
					minitimer[3] = (int8)stamp100ms;
				}
	}
}
//output leds from test or PLC channel
short set_front_leds(int8 *buffer,offset,bytes)
{
	short general_fail = 0;
	int n,expander_address;
	short start_bank;
	start_bank = offset & 0x01;
if(bytes && (offset+bytes)<=8)
{
	for (n=0;n<((bytes)>>1);n++)
	{	
		expander_address = ((offset+(2*n))>>1);
		I2C_START(FRONT);
		if(I2C_WRITE(LED_Expander_addresses[expander_address])==0)		//bot leds
		{	
			bit_set(I2C_led_expander_OK,n);
			I2C_WRITE(0x02+ start_bank);	//r02
			
			I2C_WRITE(~(*(buffer+(2*n))));
		if(!start_bank && (bytes-n-1))
			{
				I2C_WRITE(~(*(buffer+(2*n)+1))); 
			}
			start_bank = 0;
			i2c_stop(FRONT);	
		}	
		else
		{
			i2c_stop(FRONT);
			bit_clear(I2C_led_expander_OK,n);
			general_fail = 1;
			bit_clear(I2C_led_expander_OK,n);
			output_low(RESET_I2C_OUTPUTS);							
			I2C_fail_led_cnt++;
			if ((int8)stamp100ms - minitimer[3] > I2C_FAIL_MAX)
			{
				I2C_fail_out_cnt = 0;
				output_high(RESET_I2C_OUTPUTS);	
				minitimer[3] = (int8)stamp100ms;
			}
		}	
	}
}
else 
	return (0);

if(general_fail)
		init_out_expanders();
return (!general_fail);
}

void lockdown_handler()	// LOCKDOWN DECISION
{
	if ((!bit_test(LOCAL_EMERGENCY_BUTTON)))// || (bit_test(MODBUS_EMERGENCY_FLAG) && MODBUS_CONTROLLED) )
	{
   		lockdown = 1;
		if (FATI_FUNCTION)
		{
			LOCAL_manual_open_request = 0;	// LOCAL weather autorequests persist in request stack !!
			LOCAL_manual_close_request = 0;
			HMI_manual_open_request = 0;
			HMI_manual_close_request = 0;
			run_open = 0;
			run_close = 0;
			bit_clear(OPEN_SCREEN);
			bit_clear(CLOSE_SCREEN);
			do_precip_timer_loop = 0;
			manual_request = 0;
			clear_busy_PRIOS();
		}
		bit_set(RED_FLASHER_COIL);
		for(int j = 0 ; j < 5;j++)
		{
			if(rgb_sequence[0][j] == 0) 
			{	
				rgb_sequence[0][j] = 0x000F; //RED
				rgb_sequence[1][j] = 0xFF00; //CONTINUOUS
			}
		}
	}

	else if(lockdown)		// NORMAL OUTPUT voltage to outputs 
	{
		lockdown = 0;
		bit_clear(RED_FLASHER_COIL);
		for(int j = 0 ; j < 5;j++)	//CLEAR RGB SEQUENCE ONCE after lockdown
		{
			if(rgb_sequence[0][j] > 0) 
			{	
				rgb_sequence[0][j] = 0x0000; //NONE
				rgb_sequence[1][j] = 0xFF00; //CONTINUOUS
			}
		}
	}

if(plc_test)
{
	output_H(test_coils[2] );		// MASKED TRUE for disabling while emergency 
	output_J(test_coils[3] );

	I2C_START(IO);		
   	if(I2C_WRITE(IO_Expander_addresses[0])==0)		//out coils
	{	
		I2C_out_expander_OK = 1;
		I2C_WRITE(0x02);	//r02
		I2C_WRITE(test_coils[0]); 
		I2C_WRITE(test_coils[1]); 	
	}	
	else 
		I2C_out_expander_OK = 0;

	i2c_stop(IO);
}
else
{

	output_H(lockdown?(coils[2]& ~PINSH_AFFECTED_BY_EMERGENCY) : coils[2] );		// MASKED TRUE for disabling while emergency 
	output_J(lockdown?(coils[3]& ~PINSJ_AFFECTED_BY_EMERGENCY) : coils[3] );

	I2C_START(IO);		
   	if(I2C_WRITE(IO_Expander_addresses[0])==0)		//out coils
	{	
		I2C_out_expander_OK = 1;
		I2C_WRITE(0x02);	//r02
		I2C_WRITE(lockdown?(coils[0] & ~I2C_PORT0_AFFECTED_BY_EMERGENCY)	 : coils[0]); 
		I2C_WRITE(lockdown?(coils[1] & ~I2C_PORT1_AFFECTED_BY_EMERGENCY)	 : coils[1]); 	
	}	
	else 
		I2C_out_expander_OK = 0;

	i2c_stop(IO);

	}			
}

void data_to_RGB(int16 colour_1,int8 intensity,int8 duty_cycle)
{
 	if(	I2C_WRITE(0xE0)==0)
{
    	int8 colour;
		I2C_WRITE(0x80);//control + increment settings
		I2C_WRITE(0x01);//MODE REG1
        I2C_WRITE(0x25);//MODE REG2
colour = (int8)((colour_1&0x000F)<<4);
        I2C_WRITE(colour);
colour = (int8)(colour_1&0x00F0);
        I2C_WRITE(colour);
colour = (int8)((colour_1&0x0F00)>>4);
        I2C_WRITE(colour);
colour = (int8)((colour_1&0xF000)>>8);
        I2C_WRITE(colour);
        
        I2C_WRITE(intensity);//grppwm
        I2C_WRITE(duty_cycle);//blink/24
        I2C_WRITE(0xFF);//OUTPUT SETTINGS
	}
}
void set_RGB(char output_channel,int16 colour_1,int8 intensity,int8 duty_cyle)//,int color_2,int interval_on,int interval_off
{
	if(output_channel == 0)
	{
	i2c_start(FRONT);
	data_to_RGB(colour_1,intensity,duty_cyle);
	i2c_stop(FRONT);
	}
	else if(output_channel == 1)
	{
	i2c_start(IO);
	data_to_RGB(colour_1,intensity,duty_cyle);
	i2c_stop(IO);
	}

             
                


}
