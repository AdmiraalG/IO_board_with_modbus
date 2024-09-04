// ------------------------------------------------------------------------------------------- 	// 
// 								Interrupt routine												//
// ------------------------------------------------------------------------------------------- 	// 
extern struct settings_1	modbus_settings;
extern unsigned int8 current_modbus_address;


unsigned int16 buscol;
int16 timer2_tweak = 0x000E;
int16 timer3_tweak = 0x3CFD;
short lock_coils_direct_access;
unsigned int8 modbus_command_history[8];
unsigned int8 curr_history_byte;

extern 	int16 input_regs[Q_INPUT_REGS] ; //3x analog inputs
extern  int8 coils[Q_COIL_BANKS] ;
extern	int8 inputs[Q_INPUT_BANKS]; // 16 inputs 8 direct and 8 I2C or 16 I2C
extern 	int16 hold_regs[Q_HOLDING_REGS]; //3x analog inputs
int16 event_count, message_count;
extern struct DF_time time_C ;	//clock

int32 STAMP100us;
int32 STAMP100ms;
// DECLARED TROUGH DRIVER MODBUS2.H
// extern int1 modbus_timeout_enabled; // found in modbus phy_layer_rtu2.c
// extern int1 modbus_timeout_request; // the delay wait is timed with interrupt now
// extern int1 modbus_timed_out;
 
int16 ten_100mscounter;
int16 two_100mscounter;
int16 var_100mscounter;
int32 debug_stamp[2];
int32 max_stamp[2];
int32 duration_stamp[2];
int16 flow_counter[8];
//extern int16 flow[8];
int count_done;


short next_display_int,trigger_show_modfunc;
int32 counter_10ms;
int16 modbus_timer;
int1 busy_timeout;


short new_max_1;

int max_live_time;
short time_to_sync_leds;
short time_to_sync_indicator;
short sample_display,sample_RGB_int,run_test_int,do_1_int;
short clock_1_int,clock_2_int;
short init_time_int; 
short sample_analogs; 
short mod_count ;
//short displaying_init,stop_display_init;
int  displaying_init_timeout = 4;
unsigned int button_byte;
short refresh_display_int; 
short	auto_display_timedec;
	

int8 	next_in ; 
int8 	next_out ;
int8 t_next_in; 
int8 t_next_out;
char r_buffer[R_BUFFER_SIZE];
char t_buffer[T_BUFFER_SIZE]; 
char c; //incoming char

int t2ints,t3ints;
int n_i; //incrementor

unsigned int16 little_10ms_delay;

#INT_TIMER2 		//100us

void TIMER2_isr()                          		// counting every 1964 clocks LOW PRIO OVER RDA
{ 
	int16 *portpoint = getenv("SFR:PORTD"); 	// memory address of first port	
												//	t2ints++;
	int flowcount_done_idx,port_index ;	

	set_timer2(timer2_tweak);					// comming every 101.7us scoped
	stamp100us ++;								// not for full process timings... only correct between RDA useage

												//routine pulled out original driver set of modbus.h//
 	flowcount_done_idx=1;						//start at first bit
	for (int m=0;m<2;m++)  						// 2 times 
	{
		int8 *temp;
		temp = (int8 *)portpoint+m; 			// INCREMENTS ON POINTERS TAKE ACCOUNT OF POINTER WIDTH!!so temp casted pointer to int8 or will add *2 pos
												// PORT D AND E ARE ADJACENT 8 BIT MEM LOCATIONS so only address of PORTD is needed
		for ( n_i=0;n_i<4;n_i++,flowcount_done_idx <<= 1)		//shift to next bit
		{
		port_index = (1<<n_i); 					// scan sequence
			
			if ( ~(*temp) & port_index)  // BIT ON??? Inverted value of port pointer masked with scanning index
			{
				if(~count_done & flowcount_done_idx)	// bit operand on int variable
				{
					count_done |= flowcount_done_idx; // use bit at position to indicate handled count
					flow_counter[n_i]++;	
				}
			}
			else 		//bit off
				count_done &= ~flowcount_done_idx; //toggles bits off one by one as 8 bit flagregister
		}
	}
	if(modbus_timeout_enabled)
	{
		if(modbus_timeout_request)					// stays around until modbus timed out
		{	
			modbus_timeout_request = 0;				// re-request
			busy_timeout = 1;
			modbus_timer = 0 ;		
		}
	
		if (busy_timeout)
		{
			if(modbus_timer >=  MODBUS_GETDATA_TIMEOUT)
			{
				busy_timeout = 0;
				modbus_timed_out = 1;
				modbus_timer = 0 ;		// charge for next request trigger
			}
			else
			modbus_timer++;
		}
	}
	else
	{
		modbus_timeout_request = 0;				
		busy_timeout = 0;
		modbus_timer = 0 ;	
	}	
// END MODBUS EXCERPT									
	if(MODBUS_kbhit())
	{
		message_count++;
		poll_modbus(); 							// every 2 ms
		duration_stamp[0] = Stamp100us - debug_stamp[0];
		if(max_stamp[0] < duration_stamp[0])
		{
			max_stamp[0] = duration_stamp[0];
			new_max_1 = 1;
			max_live_time = 0;
		}
	}
}
void poll_modbus() // CALLED BY INTERRUPT ON RS485 MODBUS PROTOCOL MATCH
{
    
//	genuine_modbus_command = 1;
	bit_set(BUS_COMMAND_FLAG);	// bus sends MODBUS commands(nevermind address) resets timout
	bit_set(BUS_COMMAND_COIL); // NEEDED IMMEDIATELY FOR READBACK OK TO HMI
 
//check address against our address, 0 is broadcast
   	if((modbus_rx.address == current_modbus_address) /*|| modbus_rx.address == 0*/)
	{
		bit_set(MATCH_COMMAND_FLAG);
       	int8 data[16] = {0,0,0,0,0,0,0,0,0};
        int8 swap;
        int8 bits_to_go;      
        int8 last_byte_idx,total_bytes;
        int8 bits_leftover;
        int8 input_array_index;				
        bits_to_go = modbus_rx.data[3];       
        bits_leftover = (modbus_rx.data[1] - ((modbus_rx.data[1] >>3)<<3)); // laastse bit positie
        input_array_index = (bits_to_go/8);   // grootste aangesproken coil index van de n x 8 coils/inputs
        if (bits_leftover == 0)
               input_array_index --;

	modbus_command_history[((curr_history_byte++) & 0x07)] = modbus_rx.func;
   
		switch(modbus_rx.func)
       	{
           	case FUNC_READ_COILS:    //read coils 01 gaat ook naar FUNC_READ_DISCRETE_INPUT tot aan break
           	case FUNC_READ_DISCRETE_INPUT:    //read inputs 02
				// data 0 = hibyte offset, 1 =lobyte , 2 = hibyte number of bits, 3 is lobyte 
            		total_bytes = last_byte_idx = ( ((modbus_rx.data[3] +  modbus_rx.data[1]) / 8) - 1 );                // byte array index maximum enkel op lobyte !! MODBUS 1 based to 0 based array -> last_byte_idx = bitindex 0 - 7             
				// LOGICAL ORS FOR : higbytes must be 0 (0-255 bits max model) Must be expanded for highbyte use,sum of lowbytes must not exceed max i/o
					if((modbus_rx.data[0] || modbus_rx.data[2] || modbus_rx.data[1]) >= (Q_COIL_BANKS *8) || (modbus_rx.data[3]+modbus_rx.data[1]) > (Q_COIL_BANKS *8) && modbus_rx.func == FUNC_READ_COILS ) // 64 inputs and outputs addressable anders error
                		modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
					else if ((modbus_rx.data[0] || modbus_rx.data[2] || modbus_rx.data[1]) >= (Q_INPUT_BANKS *8) || (modbus_rx.data[3]+modbus_rx.data[1]) > (Q_INPUT_BANKS *8) && modbus_rx.func == FUNC_READ_DISCRETE_INPUT ) // 64 inputs and outputs addressable anders error
                		modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
                  	else
                  	{
                  		int n = 0;
            			int u = 0;
            			n = (input_array_index); // n = at maximum
						//Fill data[n]
                  		while (bits_to_go > 0)
                  		{

                        	if (n > 0)                   // eerst bytes zonder laatste
                        	{ 
                           		for (u = 0 ; u < 8  ;u++)
                           		{
                              		data[n] = data[n] << 1;                     //plaatske vrijmaken voor copy
                              		if(modbus_rx.func == FUNC_READ_COILS)         //coil copy
                              		{
                                		if (bit_test(coils[last_byte_idx],7-u))
                                 		{
                                 			data[n]++;                        // +1 als coil = 1 anders +0
                                 		}   
                              		}
                              		else                                 //input copy
                              		{
                              			if (bit_test(inputs[last_byte_idx],7-u))
                                		{
                                 			data[n]++;
                                 		}   
                              		}
                              		bits_to_go --;                              // next bit         
                           		}
                           		if( last_byte_idx > 0)
                           			last_byte_idx--;	// coilindexke lager
                        		n--; 
                        	}
                        	else                        // laatste coils apart want misschien onvolledig
                        	{
                           		if (bits_leftover > 0) //rest verwerking laatste onvolledige byte als die er is
                           		{ 
                              		bits_to_go  = 8;   // data vullen tot aan 8e bit teller op 8
                           		}                  
                           
                        		if(modbus_rx.func == FUNC_READ_COILS)         //coil copy
                           		{
                           			for (u = 0 ; u < 8  ;u++)
                              		{
                                 		data[0] = data[0] << 1;
                                 		if (bit_test(coils[last_byte_idx],7-u))
                                 		{
                                 			data[0] ++;
                                 		}   
                                 		bits_to_go --;         
                              		}      
                           		}
                        		else
                           		{
                           		for (u = 0 ; u < 8  ;u++)
                              		{
                                 		data[0] = data[0] << 1;
                                 		if (bit_test(inputs[last_byte_idx],7-u))
                                 		{
                                 			data[0] ++;
                                 		}   
                                 		bits_to_go --;         
                              		}      
                           		}
                        	}   
                     	}
                  
       					data[input_array_index] = data[input_array_index] & (0xFF>>(bits_leftover));   //[0] fill out values with ones in last data, quantity not always multiple of 8
 						  
						if(SWAP16_8COILS)
						{
							for (n=0;n<total_bytes;n+=2)
							{
	             				swap = data[n];		//rearranging data for output to responses
                				data[n] = data[n+1];
                				data[n+1] = swap;
                				        
							}
						}
                		if(modbus_rx.func == FUNC_READ_COILS)
                 			modbus_read_coils_rsp(modbus_settings.address, (input_array_index + 1), &data);               // weet niet of de [0] moet, 
                                                                                //zonder werkt het mss met data als int of int array
                		else
                     		modbus_read_discrete_input_rsp(modbus_settings.address, (input_array_index + 1), &data);         // was bug in driver

                		event_count++;
						bit_set(MODBUS_EVENT_PARSED);
               		}
            		break;
            		case FUNC_READ_HOLDING_REGISTERS:
            		case FUNC_READ_INPUT_REGISTERS:
						//data[0] H_offset,data[1] L_offset,data[2] H_q_bytes,data[3] L_q_bytes
						if(((((int16)modbus_rx.data[0]+modbus_rx.data[2])<<8) + modbus_rx.data[1] + modbus_rx.data[3]) > Q_INPUT_REGS) 
						{
                        	if(  modbus_rx.func == FUNC_READ_INPUT_REGISTERS)
                        	   	modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
							else 
                        	   	modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
						}
                        else	// OK Function
                        {
                          	if(modbus_rx.func == FUNC_READ_HOLDING_REGISTERS)
                              	modbus_read_holding_registers_rsp(modbus_settings.address,(modbus_rx.data[3]*2),hold_regs+modbus_rx.data[1]);//data 3 are word16 and bytes are requested
                           	else
                           		modbus_read_input_registers_rsp(modbus_settings.address,(modbus_rx.data[3]*2),input_regs+modbus_rx.data[1]);

                     //	modbus_read_input_registers_rsp(unsigned int8 address, unsigned int8 byte_count, unsigned int16 *input_data);
                        event_count++;
						bit_set(MODBUS_EVENT_PARSED);
                        }
            		break;
         			case FUNC_WRITE_SINGLE_COIL:      //write coil

            			last_byte_idx = ( ( modbus_rx.data[1]) / 8  );                // byte array index maximum 
               			if(modbus_rx.data[0] || modbus_rx.data[3] || modbus_rx.data[1] > (Q_COIL_BANKS *8)) // 8 coils x 8 bit
                  			modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               			else if(modbus_rx.data[2] != 0xFF && modbus_rx.data[2] != 0x00)
                  			modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_VALUE);
               			else
               			{
                 			//if(!(lock_coils_direct_access && (last_byte_idx < 4)))			//LOCK ADDED 1/2 until coil 32 so 4 banks of 8
							//{
               	 				if(modbus_rx.data[2] == 0xFF)
								{
									if(!(lock_coils_direct_access && (last_byte_idx < 4)))			//LOCK ADDED 1/2 until coil 32 so 4 banks of 8
	                   					bit_set(coils[last_byte_idx],bits_leftover );
        						}   						
								else
                   					bit_clear(coils[last_byte_idx],bits_leftover );		
               				//}
							modbus_write_single_coil_rsp(modbus_settings.address,modbus_rx.data[1],((int16)(modbus_rx.data[2]))<<8);
               				event_count++;
							bit_set(MODBUS_EVENT_PARSED);
						}
               		break;
            		case FUNC_WRITE_SINGLE_REGISTER:
               			if(modbus_rx.data[0] || modbus_rx.data[1] > (Q_HOLDING_REGS))
                  			modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               			else
               			{
                  			hold_regs[modbus_rx.data[1]] = make16(modbus_rx.data[2],modbus_rx.data[3]);
                  			modbus_write_single_register_rsp(modbus_settings.address,
                            make16(modbus_rx.data[0],modbus_rx.data[1]),
                            make16(modbus_rx.data[2],modbus_rx.data[3]));
							event_count++;
							bit_set(MODBUS_EVENT_PARSED);
      			        }
               		break;
            		case FUNC_WRITE_MULTIPLE_COILS:
						//data 1 is address of first coil
               			if(modbus_rx.data[0] || modbus_rx.data[2] || modbus_rx.data[1] >= (Q_COIL_BANKS*8) || modbus_rx.data[3]+modbus_rx.data[1] > (Q_COIL_BANKS *8))
                  			modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               			else
               			{
                  			int i,j,coil_idx,coil_byte_i,Q,/*zerofills_in_last_coil,last_coildata_idx*/current_data_idx,first_coil_idx,data_in_last_coil,last_bit_idx;
							
							last_bit_idx = (modbus_rx.data[1]+ modbus_rx.data[3] + 1 ) ; // off0 + 1 bit is last_byte_idx 0 is coils[0}],0
							//	total_bits is modbus_rx.data[3] + 1 and	begin bit index is modbus_rx.data[1]
							data_in_last_coil = ((last_bit_idx-1) & 0x07); 
         					//zerofills_in_last_coil = 8 - data_in_last_coil ; // restbits niet deelbaar door 8 niet nodig want teller i stopt vanzelf
							Q = (last_bit_idx - 1)  ; // zero based
							//last_coildata_idx = 4 + modbus_rx.data[4];	// data[4](bytes of coildata) 5 + modbus_rx.data[4] - 1
							first_coil_idx = (modbus_rx.data[1] >> 3);//offset delen door 8 dus LAAGSTE COILBANK EERST
							
		        			for(i=modbus_rx.data[3],j=0; i >  0 ; i--,j++) // voor elke bit maxidx naar offsetidx (i to 0)
							{
							//	coil_idx = first_coil_idx + ((8-data_in_last_coil + j)>>3)-1 ; 
								coil_idx = first_coil_idx + (((modbus_rx.data[1] & 0x07)+ j)>>3);//(byteindex)  determine 8 bit coilarray index ... end last_byte_idx 1-based
				 								
               					coil_byte_i = (((modbus_rx.data[1] & 0x07)+j)& 0x07); //mod8 loop met setpunt
                     
								current_data_idx = (5+(j>>3));//coildata starts at data[5] and shifts per 8
                    			
								if(bit_test(modbus_rx.data[current_data_idx],(j&0x07))) // Lower data first starting at bit 0
								{
									if(!(lock_coils_direct_access && (coil_idx < 4))) // SET ONLY REQUEST COILS
	 		               	        	bit_set(coils[coil_idx],(coil_byte_i));// lowest offset firtst to bit 7(coil N8)
    							}				
	      		        	   else
                 			       bit_clear(coils[coil_idx],coil_byte_i);		//CLEAR ALLOWED
    	             								
							}
						
                 			modbus_write_multiple_coils_rsp(modbus_settings.address,
                      		make16(modbus_rx.data[0],modbus_rx.data[1]),
                      		make16(modbus_rx.data[2],modbus_rx.data[3]));
                  			event_count++;
							bit_set(MODBUS_EVENT_PARSED);
					
							
               			}			
               		break;
            		case FUNC_WRITE_MULTIPLE_REGISTERS:

               			if((modbus_rx.data[0] || modbus_rx.data[2] ||
                  			modbus_rx.data[1] >= (Q_HOLDING_REGS)) || modbus_rx.data[3]+modbus_rx.data[1] > Q_HOLDING_REGS)
                  			modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_DATA_ADDRESS);
               			else
              			 {
                 			int i,j;

                			for(i=0,j=5; i < modbus_rx.data[4]/2; ++i,j+=2)
                     			hold_regs[i+modbus_rx.data[1]] = make16(modbus_rx.data[j],modbus_rx.data[j+1]);

                  			modbus_write_multiple_registers_rsp(modbus_settings.address,
                            make16(modbus_rx.data[0],modbus_rx.data[1]),
                            make16(modbus_rx.data[2],modbus_rx.data[3]));
               			}
						event_count++;
						bit_set(MODBUS_EVENT_PARSED);
               		break;
            		default:    //We don't support the function, so return exception
               			modbus_exception_rsp(modbus_settings.address,modbus_rx.func,ILLEGAL_FUNCTION);
         		}
//	set_RGB(1,0x0000,0,0);//,int color_2,
     	}
	
  // 		 else // modbus_rx.address != modbus_settings.address
    //		  {
	//	
    //		  }


   	
}  
#INT_TIMER3	//10ms interrupts TWEAKED to 10ms
void TIMER3_isr()                          		// counting up LOW PRIO OVER RDA
{ 						
	t3ints++;						// Dangling over the overflow point and back
	set_timer3(timer3_tweak);
	counter_10ms++;	
	if(little_10ms_delay)
		little_10ms_delay--;//stops at 0
	// Having a "feeling distinguished" moment for doing it like a pro //C1-2 G3-4
		button_byte = ((*getenv("SFR:PORTC") & 0x06) | (*getenv("SFR:PORTG") & 0x18)) >> 1; //TRIAL OK (WOEHOOO MOMENT)
 	// low_nibbles holds button states
	if (counter_10ms >= 10)	// 100 msec			//counter runs at correct speed
	{
		STAMP100ms++;
		time_to_sync_leds = 1;
		
 		ten_100mscounter ++;
   		two_100mscounter ++;
		var_100mscounter ++;
		counter_10ms = 0;
		mod_count = 1;
	//	next_display_int = 1;
	
	}
	if (two_100mscounter >= 2)
	{
		two_100mscounter = 0;
		run_test_int = 1;
		refresh_display_int = 1;
//		flush_counts = 1;
	 	 
		next_display_int = 1;

	}    
 	if (var_100mscounter >= 5)
    {
		sample_RGB_int = 1;
    	var_100mscounter = 0;
		do_1_int = 1;
		sample_analogs = 1;
		trigger_show_modfunc = 1;
  
    }
    
   	if (ten_100mscounter >= 10)
    {
		time_to_sync_indicator = 1;
 		sample_display = 1;
		if(new_max_1)
		{
			max_live_time++;
		
			if(max_live_time >= 10)	// 10 sec
			{
				max_live_time = 0;
				new_max_1 = 0;
				max_stamp[0] = 0;
			}
		}
		
    	time_C.seconds++;				//struct time_C is just live counter clock
		clock_1_int = 1;
		clock_2_int = 1;
		init_time_int = 1;
		auto_display_timedec = 1;

		for (int n=0;n<8;n++)
		{
			input_regs[16+n] = flow_counter[n] ;
			flow_counter[n] = 0;
		}
		if (time_C.seconds > 59) 
   		{
 			time_C.minutes++; 
        	time_C.seconds=0;
   		} 
		if (time_C.Minutes > 59) 
    	{
        	 time_C.hours++; 
        	 time_C.minutes=0;        
		} 
      	ten_100mscounter = 0;	

	
	}
}
// RS232-485 MODBUS

#INT_TBE2			// fires when transmit data is empty again
void serial_TBE2_irq() 
{

   if(t_next_in!=t_next_out)	// while indexes not equal spit chars to harware
   {							//t_next_in is head of worm
      putc(t_buffer[t_next_out],USB);//t_next_out is butt
      t_next_out=(t_next_out+1) % T_BUFFER_SIZE;//butt comes closer
   }
   else
      disable_interrupts(int_tbe2); //nothing to send so shut off interrupts
}

#INT_RDA2		// fires when data in buffer
void serial_RDA2_irq() 
{ 
   int t; 

   c=r_buffer[next_in]=getc(USB); 
   t=next_in; 
   if (++next_in == R_BUFFER_SIZE)
            next_in=0;  
   if(next_in==next_out) 
	 		next_in=t;    			// last one		 
}
#INT_BUSCOL2
void buscollision()
{
buscol+=1;
}

