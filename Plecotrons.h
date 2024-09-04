
#define DESIGNER_NAME						"Guy VandenBroeck"
#define RELEASE_DATE						"10/03/2023"
#define HARDWARE_VERSION					"2.1"
#define SOFTWARE_VERSION					"TESTER 2"

#define TIMER_FREQUENCY       			(clockf_4)
//#define TIMER_3_DEC						(TIMER3_FREQUENCY/10)
#define USE_WITH_PC                   	1				 
#ifndef MODBUS_TIMER_USED
#define MODBUS_TIMER_USED 				MODBUS_TIMER_T1
#endif		
#define MODBUS_TIMER_UPDATE  			MODBUS_TIMER_ISR
#define MODBUS_TYPE                   	MODBUS_TYPE_SLAVE
#define MODBUS_SERIAL_TYPE              MODBUS_RTU     //use MODBUS_ASCII for ASCII mode
#define MODBUS_SERIAL_RX_BUFFER_SIZE    64
#define MODBUS_SERIAL_BAUD              76800
#define MODBUS_PARITY                  	"EVEN"

#define MODBUS_ADDRESS             		1//default
#define SWAP16_8COILS					FALSE

//#define MODBUS_GETDATA_TIMEOUT       10      //ifndeffed in driver
#define MODBUS_SERIAL_INT_SOURCE 		MODBUS_INT_RDA
#define MODBUS_SERIAL_ENABLE_PIN   		PIN_B4   // Controls DE for RS485 RX low, TX high
#define MODBUS_SERIAL_RX_ENABLE    		PIN_B5 // Controls RE for RS485 keep low
#define MODBUS_CONTROLLED				TRUE

#define MODBUS_TIMEOUT					20		// 20*100us = 2ms
#define MODBUS_INTERLOCK_TIMEOUT		120		// 120*100ms = 12s  adddress must be active within a arbitrairy 12 second period to switchover
#define BUS_FLAG_TIMEOUT				1		// 100 msec
#define COMMAND_FLAG_TIMEOUT			50		// 10*100 msec
#define FEEDBACK_FLAG_TIMEOUT			1		// 10*100 msec
#define IRRIGATION_INTERLOCK_TIMEOUT	10		// 10*100ms = 1s will be kicked when DISABLE_ACTIVE and ENABLE_ACTIVE COILS are developped----- TO DO
#define MODBUS_GETDATA_TIMEOUT 			5		// MUST BE AT LEAST ONE, 0 = 256// 10*100us = 1ms 	
#define DISPLAY_IDLE_TIME				20	// 10 sec
#define DEFAULT_COIL_TIMEOUT 			30	//*100ms
#define RED_FLASHER_TIMEOUT				2
#define BUS_COMMAND_TIMEOUT				10
#define MATCH_FLAG_TIMEOUT				20

#define TEST						FALSE
#define FATI_FUNCTION				TRUE
#define WATERCONTROL_FUNCTION		FALSE
#define OUT_CARD					TRUE
#define	IN_CARD						TRUE
#define	FRONT_CARD					TRUE
#define SCREEN_MAGNETICS_BYPASS 	TRUE



#define Q_COIL_BANKS		8
#define	Q_INPUT_BANKS		2
#define	Q_INPUT_REGS		32
#define	Q_HOLDING_REGS		32


//#define MODBUS_TIMER_USED 				MODBUS_TIMER_T1

#define clockf                			40000000
#define clockf_4             			10000000
#define CLOCKS_10MS						100000   //(TIMER_FREQUENCY/4/100)
#define CLOCKS_100us					1000
#define PINSH_AFFECTED_BY_EMERGENCY		0	    // all pins affected
#define PINSJ_AFFECTED_BY_EMERGENCY		0		// 4MSB need decoupling from frontleds (E8 0_4 to 0_7), out12(bit3) = flasher
#define I2C_PORT0_AFFECTED_BY_EMERGENCY 0
#define I2C_PORT1_AFFECTED_BY_EMERGENCY	0


#define OPEN							1
#define CLOSE 							0
#define LOCAL							0
#define HMI								1
#define LINE_BUFFERS 					3
#define PRIO_handled_at_end				FALSE
#define	RESET_FRONT_EXPANDERS			Pin_D7   
#define	RESET_I2C_INPUTS				Pin_B0
#define	RESET_I2C_OUTPUTS				Pin_B1
#define	ANALOG_MUX_PORT_CH				Pin_B0
#define USB_CON_SENSE_PIN				Pin_C0
#define BUTTON_1						Pin_G4
#define BUTTON_2						Pin_G3
#define BUTTON_3						Pin_C1
#define BUTTON_4						Pin_C2

#define BUFFER_SIZE 			32
#define R_BUFFER_SIZE 			32
#define T_BUFFER_SIZE 			64
#define COMMAND_BUFFER_SIZE		30
#define number_of_triggers 		7
#define Trigger1 				"SCREEN"			
#define Trigger2 				"VALVE"			
#define Trigger3 				"SENSOR"				
#define Trigger4 				"OPEN"			
#define Trigger5 				"CLOSE"			
#define Trigger6 				"TEST"
#define Trigger7 				"OVERRIDE"	
#define LINEFEED				0x0A		//LineFeed
#define COMMAND_DEPTH			3	//		
#define VAR_DEPTH				3

#define DISPLAY_CONTRAST_MAX	200
#define DISPLAY_CONTRAST_MIN	30

#define I2C_FAIL_MAX			2	//x100ms



//#define LOCK_COILS_DIRECT_ACCESS	FALSE  // NEEDS TO BE TRUE FOR FATI

//DEFINED PLC OUTPUTS					OUTPUTS
//----------PHYSICAL INPUTS/OUTPUTS----------//
//DEFINED INPUTS						INPUTS
#define OPEN_SENSOR	     				inputs[0],0
#define CLOSE_SENSOR     				inputs[0],1
#define RAIN_SENSOR	     				inputs[0],2
#define 	    MANUAL_OPEN				inputs[0],3
#define 		MANUAL_CLOSE			inputs[0],4
//#define 		"EMPTY"					inputs[0],5
//#define 		"EMPTY"					inputs[0],6
#define LOCAL_EMERGENCY_BUTTON			inputs[0],7
//I2C PORTS
#define	OPEN_SCREEN						coils[0],0 //coil 1 LED 40 //outword 24
#define	CLOSE_SCREEN					coils[0],1 //coil 2 LED 39 //outword 25

#define 		OUTLET_1				coils[0],0 //coil 1 LED 38	//outword 26
#define 		OUTLET_2				coils[0],1 //coil 2
#define 		OUTLET_3				coils[0],2 //coil 3 LED 38	//outword 26
#define 		OUTLET_4				coils[0],3 //coil 4 LED 37	//outword 27
#define 		OUTPUT_5				coils[0],4 //coil 5 LED 36	//outword 28
#define 		OUTPUT_6				coils[0],5 //coil 6 LED 35	//outword 29
#define 		OUTPUT_7				coils[0],6 //coil 7 LED 34	//outword 30
#define 		OUTPUT_8				coils[0],7 //coil 8 LED 33	//outword 31
#define 		OUTPUT_9				coils[1],0 //coil 9 LED 32  //outword 16
#define 		OUTPUT_10				coils[1],1 //coil 10 LED 31 //outword 17
#define 		OUTPUT_11				coils[1],2 //coil 11 LED 41 //outword 18
#define 		OUTPUT_12				coils[1],3 //coil 12 LED 42 //outword 19
#define 		OUTPUT_13				coils[1],4 //coil 13 LED 43 //outword 20
#define 		OUTPUT_14				coils[1],5 //coil 14 LED 44 //outword 21
#define 		OUTPUT_15				coils[1],6 //coil 15 LED 45 //outword 22
#define 		OUTPUT_16				coils[1],7 //coil 16 LED 46 //outword 23
//DIRECT PORTS
#define 		VALVE_1					coils[2],0 //coil 17 LED 47 //outword 8
#define 		VALVE_2					coils[2],1 //coil 18 LED 48 //outword 9
#define 		VALVE_3					coils[2],2 //coil 19 LED 49 //outword 10
#define 		VALVE_4					coils[2],3 //coil 20 LED 50 //outword 11
#define 		VALVE_5					coils[2],4 //coil 21 LED 62* //outword 12
#define 		VALVE_6					coils[2],5 //coil 22 LED 61* //outword 13
#define 		VALVE_7					coils[2],6 //coil 23 LED 60 //outword 14
#define 		VALVE_8					coils[2],7 //coil 24 LED 59 //outword 15
#define 		VALVE_9					coils[3],0 //coil 25 LED 58 //outword 0
#define 		VALVE_10				coils[3],1 //coil 26 LED 57 //outword 1
#define 		VALVE_11				coils[3],2 //coil 27 LED 56 //outword 2
#define 		VALVE_12				coils[3],3 //coil 28 LED 56 //outword 3
#define 		RED_FLASHER_COIL		coils[3],4 //coil 29 LED 54			//outword (0x,29)
#define 		BUS_COMMAND_COIL		coils[3],5 //coil 30 LED 53			//outword (0x,30)
#define 		BUS_EVENT_COIL			coils[3],6 //coil 31 OUTPUTLED 52	//outword (0x,31)
#define 		MATCH_FLAG_COIL			coils[3],7 //coil 32 OUTPUTLED 51	//outword (0x,32)

///------------------- VIRTUAL REQUEST COUPLED TO IDX - 4 ---------------///
#define OPEN_REQUEST_COIL				futileint,0 // vcoil 33				//0x,33
#define CLOSE_REQUEST_COIL				futileint,0 // vcoil 34
#define 		REQUEST_VALVE_1					coils[6],0
#define 		REQUEST_VALVE_2					coils[6],1
#define 		REQUEST_VALVE_3					coils[6],2 // vcoil 35
#define 		REQUEST_VALVE_4					coils[6],3
#define 		REQUEST_VALVE_5					coils[6],4
#define 		REQUEST_VALVE_6					coils[6],5 
#define 		REQUEST_VALVE_7					coils[6],6 
#define 		REQUEST_VALVE_8					coils[6],7 
#define 		REQUEST_VALVE_9					coils[7],0 
#define 		REQUEST_VALVE_10				coils[7],1 
#define 		REQUEST_VALVE_11				coils[7],2 
#define 		REQUEST_VALVE_12				coils[7],3 
#define 		REQUEST_EMERGENCY_COIL			coils[7],4 
	
///------------------- PURE  VIRTUAL BIT MEMORY ---------------///

#define SELF_RUN_COIL					futileint,0	//vcoil 57
#define PERCIP_TIMER					futileint,0	//vcoil 58 
#define PERCIP_EVENT_COIL				futileint,0	//vcoil 59
//#define	MODBUS_REQ_DETECTED				coils[5],3	//vcoil 60
#define BUS_COMMAND_FLAG				coils[7],5	//vcoil 61
#define MODBUS_EVENT_PARSED 			coils[7],6	//vcoil 62
#define MATCH_COMMAND_FLAG				coils[7],7	//vcoil 63
//#define MODBUS_EMERGENCY_FLAG			coils[7],7	//vcoil 64

#define FLOW_1							input_regs[16] //pulses on fast port
#define FLOW_2							input_regs[17]
#define FLOW_3							input_regs[18]
#define FLOW_4							input_regs[19]
#define FLOW_5							input_regs[20]
#define FLOW_6							input_regs[21]
#define FLOW_7							input_regs[22]
#define FLOW_8							input_regs[23]

//coils 0-3 real, 4-7 req, 8-11 mem
#define AFTER_RAIN_DELAY		600
#define AFTER_SUN_DELAY			80
#define RUNTIME_MAX				70
#define DROOG					1
#define NAT						0




#define DISP_BUFFER_LEN 17	// 16char + NULL
#define SUB_MENU_DEPTH 		8
#define MAIN_MENU_DEPTH 	6
#define SUB_MENU_DEPTH_0	3
#define SUB_MENU_DEPTH_1	1
#define SUB_MENU_DEPTH_2	7
#define SUB_MENU_DEPTH_3	2
#define SUB_MENU_DEPTH_4	4
#define SUB_MENU_DEPTH_5	4

#define CHOICES_0	0,0,0,0,0,0,0,0		
#define CHOICES_1	1,0,0,0,0,0,0,0
#define CHOICES_2	7,3,1,1,1,1,1,0
#define CHOICES_3	2,2,0,0,0,0,0,0
#define CHOICES_4	2,2,2,2,0,0,0,0
#define CHOICES_5	2,2,2,2,0,0,0,0
#define EIGHT_0		0,0,0,0,0,0,0,0	

#define CURR_MENU	 menu_root.current_menu
#define CURR_SUBMENU menu_root.main_menu[CURR_MENU].current_smenu
#define CURR_CHOICE  menu_root.main_menu[CURR_MENU].choice[CURR_SUBMENU]


struct DF_time 	//sort elements like this
{
int seconds;
int minutes;
long hours;
};
struct DF_time fill_timer_value(long timer_seconds);

struct settings_1
{
	unsigned short enable;
	unsigned int8 parity;
	unsigned int8 address;
	unsigned int16 l_modbus_timeout;
	unsigned int16 l_valve_timeout;
	unsigned int16 l_turnaround;
	unsigned int32 ll_baudrate;

};


void init(void);
void init_led_expanders(void);
void init_rgb(void);
void init_out_expanders(void);

void init_in_expanders(void); 

//void serial_isr1();
//void serial_isr();
void Data_processing(void);	
void test_program(void);
void PLC_function(void);

char bgetc(void);
void bputc(char);
int load_trigger(int);

short check_MODBUS_active(void);
void poll_modbus();

void poll_coil_deadman_timers(void);		// perform coil timeouts for valves
void poll_requests(void); 				// resample requests parallel 
void pool_requests(void);
void motor_run_handler(void);
void post_timer_handling(short);
void clear_busy_PRIOS();
void lockdown_handler();
void lcd_init();
void ANIN_init();
void stack_inputs();
void stack_analog_inputs();
void represent_inputs(int,int,int,int);
void represent_outputs(int,int,int,int,int,int,short);
void show_8count();
void show_modbus_stats();
void show_response_time();
void show_counters();
void show_errors();
void show_select();
void display_testactive_clear();
void RX_to_ASCII();
void display_goto(unsigned int,unsigned int);
void display_handler();
void clear_all_write_buffers();
void clear_disp_write_buffer(unsigned int)	;
void time_to_display(int,int,int,int,int);
void write_display(unsigned int,short, int);
void lcd_puts(char*,unsigned int);
void lcd_putc(char);
void button_handler(void);
short set_front_leds(int8*,int,int);
void set_RGB(char ,int16 ,int8 ,int8);

unsigned char *pluralize(char *);
unsigned char *actionize(char *,short);
unsigned char *pasturize(char *,short);

struct rtn	//returnstruct
{
	short OK;
	union value 				//overloading of variable
	{
		signed int16 s_value;
		unsigned int16 u_value;
	};//
};//
struct rtn sint16_atoi(char *, unsigned int,short);	//pointer to first char,digits,(self referencing struct with char *

typedef struct 
	{
//	unsigned int8 ssub_menu_depths[SUB_MENU_DEPTH] ;	//prototyped but not assigned(no value init yet)
	unsigned int8 choice[SUB_MENU_DEPTH] ;
	unsigned int8 current_smenu;
	union menu_value 				//overloading of value variable(fixed 16bits without compiler pointertype complaints)
	{
		signed int16 s_value;
		unsigned int16 u_value;
		short bit_value;
	};
}header_sub_menu_t;//type defined for header use, defines in runcode are different memlocatios



struct menu_s
{
	header_sub_menu_t 	main_menu[MAIN_MENU_DEPTH] ;
//	unsigned int8 sub_menu_depths[MAIN_MENU_DEPTH];
	unsigned int8 current_menu;	
};//


typedef enum{OFF = 0,ON,ERR} state_t;
typedef enum{PROBLEMOS,BEL_APELDOORN,MISERIE,SH1T_HIT_FAN,AI_CABRON,SHAFT_NICHT} fun_error;
typedef enum{OKIDOKI,CAN_DO,TUTTOBENE,ROLLING,GREASED_N_GOING,CHECKOSH,SHAFT_WHOL} fun_ok;



/* NEWHAVEN NHD RGB display
To display normal text, just enter its ASCII number. 
A number from 0x00 to 0x07 displays the user defined custom character, 
0x20 to 0x7F displays the standard set of characters, 
0xA0 to 0xFD display characters and symbols that are factory-masked on the ST7066U controller. 
0xFE is reserved.
 
0xFE 0x41 None Display on 100uS 
0xFE 0x42 None Display off 100uS 
0xFE 0x45 1 Byte Set cursor 100uS 
0xFE 0x46 None Cursor home 1.5mS 
0xFE 0x47 None Underline cursor on  1.5mS 
0xFE 0x48 None Underline cursor off  1.5mS 
0xFE 0x49 None Move cursor left one place  100uS 
0xFE 0x4A None Move cursor right one place  100uS 
0xFE 0x4B None Blinking cursor on  100uS 
0xFE 0x4C None Blinking cursor off  100uS 
0xFE 0x4E None Backspace  100uS 
0xFE 0x51 None Clear screen  1.5mS 
0xFE 0x52 1 Byte Set contrast   500uS 
0xFE 0x53 1 Byte Set backlight brightness* 100uS 
0xFE 0x54 9 Byte Load custom character  200uS 
0xFE 0x55 None Move display one place to the left  100uS 
0xFE 0x56 None Move display one place to the right   100uS 
0xFE 0x61 1 Byte Change RS-232 BAUD rate  3mS 
0xFE 0x62 1 Byte Change I2C address  3mS 
0xFE 0x70 None Display firmware version number   4mS 
0xFE 0x71 None Display RS-232 BAUD rate  10mS 
0xFE 0x72 None Display I2C address   4mS 

*/