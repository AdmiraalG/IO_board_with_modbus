//PIN 21 doet moeilijk
//pinMode(21,OUTPUT);
 // digitalWrite(21,0);

//FOR ALL IO EXPANDERS
//00h Input port 0 read  xxxx xxxx[1] 
//01h Input port 1 read  xxxx xxxx 
//02h Output port 0 read/write 
//03h Output port 1 read/write byte 1111 1111 
//04h Polarity Inversion port 0 read/write byte 0000 0000 
//05h Polarity Inversion port 1 read/write byte 0000 0000 
//06h Configuration port 0 read/write byte 1111 1111 
//07h Configuration port 1 read/write byte 1111 1111 
//FOR PCAL :
//40h Output drive strength register 0 read/write byte 1111 1111 
//41h Output drive strength register 0 read/write byte 1111 1111 
//42h Output drive strength register 1 read/write byte 1111 1111 
//43h Output drive strength register 1 read/write byte 1111 1111 
//44h Input latch register 0 read/write byte 0000 0000 
//45h Input latch register 1 read/write byte 0000 0000 
//46h Pull-up/pull-down enable register 0 read/write byte 0000 0000 
//47h Pull-up/pull-down enable register 1 read/write byte 0000 0000 
//48h Pull-up/pull-down selection register 0 read/write byte 1111 1111 
//49h Pull-up/pull-down selection register 1 read/write byte 1111 1111 
//4Ah Interrupt mask register 0 read/write byte 1111 1111  
//4Bh Interrupt mask register 1 read/write byte 1111 1111 
//4Ch Interrupt status register 0 read byte 0000 0000 
//4Dh Interrupt status register 1 read byte 0000 0000 
//4Fh Output port configuration register read/write byte 0000 0000

#include <RS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h> //The Wire library implementation uses a 32 byte buffer, therefore any communication should be within this limit. Exceeding bytes in a single transmission will just be dropped.
                  //The Wire library uses 7 bit addresses throughout
#include <GyverTM1637.h>
//#include <string.h>
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
#define Serial SERIAL_PORT_USBVIRTUAL
#endif
#define INVERTED 0xFF
#define UNINVERTED 0x00

#define NUMPIXELS         1  // The number of NeoPixels (usually 1 for built-in)


#define NEO_PIN_1             88  // The pin where the NeoPixel is connected
#define RELAIS_1_PIN           6 
#define RELAIS_2_PIN           7 
#define PULSE_PIN_1           22
#define PULSE_PIN_2           24
#define PULSE_PIN_3           26
#define PULSE_PIN_4           28
#define PULSE_PIN_5           30
#define PULSE_PIN_6           32
#define PULSE_PIN_7           34
#define PULSE_PIN_8           36
#define THERMAL_PIN_1         5      
#define THERMAL_PIN_2         3
#define DIAG_1_PIN            4
#define DIAG_2_PIN            2
#define INT_1_PIN             13
#define INT_RELAIS_PIN        12
#define RESET_I2C_RELAIS_PIN  11
#define RESET_HUB_PIN         10
#define RESET_I2C_1_PIN       9
#define RESET_I2C_2_PIN       8
#define SE_H_1_PIN            25      
#define SE_L_1_PIN            27
#define SE_H_2_PIN            33
#define SE_L_2_PIN            35
#define ERR_1_PIN             29
#define ERR_2_PIN             37
#define C_SENS_1_PIN          31
#define C_SENS_2_PIN          39
//RX_PIN is 0 by default
#define TX_PIN                1
#define RE_PIN                23
#define DE_PIN                38

#define IO_PORT_1_2_ADDRESS 0x20 // 7-bit I2C address for PCAL6416A
#define NPN_N_DRIVERS_ADDRESS 0x21 // A0 HIGH
#define LEDS_RELAIS_ADDRESS 0x77 // 7-bit I2C address for PCA9539R both A0,A1 HIGH
#define ADC_1_ADDRESS 0x48 // 7-bit I2C address for ADS7828
#define ADC_2_ADDRESS 0x49 // A0 HIGH

#define INTERNAL_ADC_REF TRUE

#define IO_PORT1_1_POL  UNINVERTED
#define IO_PORT1_2_POL  UNINVERTED
#define NPN_POL  0x00
#define DRIVERS_POL  0x00
#define IO_LEDS_POL  0x00
#define IO_RELAIS_POL  0x00
#define MAX_PULSES    750


enum I2C_address {
    io_port_1_2 = 0,
    npn_n_drivers,
    leds_relais_drivers,
    adc_1,
    adc_2
} I2C_pointer_1;  // I2C_pointer_1 is of type I2C_address

uint8_t I2C_LIST[5] ={IO_PORT_1_2_ADDRESS,NPN_N_DRIVERS_ADDRESS,LEDS_RELAIS_ADDRESS,ADC_1_ADDRESS,ADC_2_ADDRESS};
uint8_t I2C_IO_POL_LIST[6] ={IO_PORT1_1_POL,IO_PORT1_2_POL,NPN_POL,DRIVERS_POL,IO_LEDS_POL,IO_RELAIS_POL}; //INPUT POLARITIES for first 3 expanders
const uint8_t channelMap[2][8] = {0, 4, 1, 5, 2, 6, 3, 7, 
                                  0, 4, 1, 5, 2, 6, 3, 7};
uint8_t PULSE_PINS[8] = {PULSE_PIN_1,PULSE_PIN_2,PULSE_PIN_3,PULSE_PIN_4,PULSE_PIN_5,PULSE_PIN_6,PULSE_PIN_7,PULSE_PIN_8};

const int numCoils = 32;
const int numDiscreteInputs = 32;
const int numHoldingRegisters = 32;
const int numInputRegisters = 64;


const int ledRed = 41;
int blink_led(unsigned int, bool);
int blink_led(void);

  uint32_t  tc5_counter;
  uint16_t  main_counter_1;
  uint16_t  ms_counter;
  uint16_t  s_counter;
  bool      time_to_com_ipt;
  bool      poll_modbus_ipt;


 bool last_state[8],new_state[8]; 
 uint16_t  ipt_pulse_count[8];
 uint16_t  pulse_count_store[8];

bool I_O_ok;
char string[32];
bool COM_printed;
bool Modbus_server_started;
long packet_counter;
unsigned int greenledspeed; 
bool blink_led_running;
bool auto_update_RGB;
void writeExpanderPin(uint8_t, bool ,uint8_t );
void testprog();
void modbus_init();
uint16_t readADS7828Channel(uint8_t, uint8_t,uint8_t );
uint8_t   ADS7828_error;
void scanI2C();
void print_voltages();
void print_errors();
bool COM_live; 
bool ready_for_MODBUS;

Adafruit_NeoPixel Board_RGB(NUMPIXELS, NEO_PIN_1, NEO_GRB + NEO_KHZ800);

GyverTM1637 sevenseg_1(42,43);
GyverTM1637 sevenseg_2(44,45);
GyverTM1637 sevenseg_3(46,47);
GyverTM1637 sevenseg_4(48,49);
GyverTM1637 sevenseg_5(50,51);
GyverTM1637 sevenseg_6(52,53);


// Interrupt Service Routine for TC4 at 100us
void TC5_Handler() 
{
  // Check if this is a match interrupt
  if (TC5->COUNT16.INTFLAG.bit.MC0) 
  { 
    main_counter_1++;
    // Clear the interrupt flag
    TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    

    if(ready_for_MODBUS && poll_modbus_ipt)
    {
     
      if(int packetReceived = ModbusRTUServer.poll()) //PRIORITY OVER LOOP SO KEEP IN INTERRUPT(responsetime guaranteed!)
      {
        packet_counter++; // packetReceive exists here!
        poll_modbus_ipt = 0;
      }
   
    }
    tc5_counter++;
    for (int i=0;i<8;i++)
    {
      new_state[i] = !digitalRead(PULSE_PINS[i]); //counting at highest frequency 5kHz (Nyquist limit)
      if(last_state[i] == !new_state[i])
      {
        ipt_pulse_count[i] ++;
        last_state[i] = new_state[i];
      }
    }
    digitalWrite(LED_BUILTIN, 1);
    
    if (main_counter_1 > 9)
    { 
      poll_modbus_ipt = 1;
     if(ms_counter < 999)
     {
        ms_counter++;
     }
     else // 999ms_overflow
     {
        ms_counter = 0;
        s_counter++;
        time_to_com_ipt = 1;
          for (int i=0;i<8;i++)
        {
          pulse_count_store[i] = ((ipt_pulse_count[i]+ ModbusRTUServer.holdingRegisterRead(i+8)) * ModbusRTUServer.holdingRegisterRead(i+16))/100;
          ModbusRTUServer.holdingRegisterWrite(i,pulse_count_store[i]); 
          ipt_pulse_count[i] = 0;        
        }
     }
      main_counter_1 = 0;
    }
  }
   digitalWrite(LED_BUILTIN, 0);
}


void loop() 
{
    
    // map the coil values to the discrete input values    
  for (int i = 0; i < numCoils; i++) 
  {
    int coilValue = ModbusRTUServer.coilRead(i);

    ModbusRTUServer.discreteInputWrite(i, coilValue);
  }
  for(int i=0;i > 8;i++)
  {
    ModbusRTUServer.inputRegisterWrite(i,pulse_count_store[i]);
  }
  sevenseg_1.displayInt(s_counter);
  sevenseg_2.displayInt(pulse_count_store[0]);
  sevenseg_3.displayInt(pulse_count_store[1]);
 
 // scanI2C();
 //print_voltages();
 
  if(auto_update_RGB)
    Board_RGB.show();
  if(Serial && time_to_com_ipt)
  {
    print_pulses();
    //read the current value of the coil
    //int coilValue = ModbusRTUServer.coilRead(0x00);
    Serial.print("Packet ");
    Serial.print(packet_counter,DEC);
    Serial.println(" Received");
  } 

  time_to_com_ipt = 0;
  //if(COM_printed)
  //  testprog();
}

  int blink_led(unsigned int blink_speed,bool continuous)
  {
    static unsigned int blink_static = blink_speed;
    static bool cont_bool = continuous;

    if (!blink_static)
    {
      digitalWrite(ledRed,LOW);
      return 0;
    }
    else
    {  
    static unsigned long last_time = 0;
    unsigned long now = millis();
    if ((now - last_time) > blink_speed)
    {
      last_time = now;
      digitalRead(ledRed) ? digitalWrite(ledRed,LOW):digitalWrite(ledRed,HIGH);
      if(!cont_bool)
      return 0;
    }
    return 1;
    }  
  }
  
  void writeExpanderPin(int pin, bool state, uint8_t address) {
    uint8_t registerAddress;
    uint8_t pinMask;

    if (pin < 8) {
        registerAddress = 0x02; // Register for Port 0
        pinMask = 1 << pin;
    } else {
        registerAddress = 0x03; // Register for Port 1
        pinMask = 1 << (pin - 8);
    }

    // Read current state
    Wire.beginTransmission(address);
    Wire.write(registerAddress);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    uint8_t currentState = Wire.read();

    // Set or clear the bit
    if (state) {
        currentState |= pinMask;
    } else {
        currentState &= ~pinMask;
    }

    // Write the new state back
    Wire.beginTransmission(address);
    Wire.write(registerAddress);
    Wire.write(currentState);
    Wire.endTransmission();
}

void testprog()
{
I2C_pointer_1 = io_port_1_2;

for (int j =0;j<3;j++){
  writeExpanderPin(0, HIGH,I2C_LIST[I2C_pointer_1+j]);
 writeExpanderPin(8, HIGH,I2C_LIST[I2C_pointer_1+j]);
delay(50);
  for(int i=0;i<8;i++){
     writeExpanderPin(i, LOW,I2C_LIST[I2C_pointer_1+j]); 
     writeExpanderPin(i+8, LOW,I2C_LIST[I2C_pointer_1+j]); 
   if (i<7)
   {
     writeExpanderPin(i+1, HIGH,I2C_LIST[I2C_pointer_1+j]); 
     writeExpanderPin(i+9, HIGH,I2C_LIST[I2C_pointer_1+j]); 
 
   }
  delay(50);
  }
}

  digitalWrite(RELAIS_1_PIN,HIGH);
  delay(100);
  digitalWrite(RELAIS_1_PIN,LOW);
  delay(100);
  digitalWrite(RELAIS_2_PIN,HIGH);
  delay(100);
  digitalWrite(RELAIS_2_PIN,LOW);
  delay(100);

}

uint16_t readADS7828Channel(uint8_t address, uint8_t channel,uint8_t map) 
{
  uint8_t configByte = ((channelMap[map][channel]|0x08) << 4) | 0x04 | (INTERNAL_ADC_REF << 3); // Single-ended mode, internal reference, power-down between A/D converter readings
 // Single-ended mode, PD1=PD0=0 (00) ADC on at all time
  
  Wire.beginTransmission(address);
  Wire.write(configByte);
  Serial.print("(");
  Serial.print(configByte);
  Serial.print(")");
  Wire.endTransmission();
  
  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return (msb << 8) | lsb;
  }
  return 0; // Return 0 if read fails
}

void scanI2C() 
{
    byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Done.\n");

  delay(5000); // Wait 5 seconds for the next scan
}


void print_pulses()
{
  for(int j = 0;j<8;j++)
   {
      Serial.print(pulse_count_store[j] );
      Serial.print("  ");
   
   }
  Serial.println("");
}
void print_voltages()
{
  Serial.println(s_counter);
  for(int i = 0;i<2;i++)
  { 
  
    Serial.print("ADC ");
    Serial.print(i);
   for(int j = 0;j<8;j++)
   {
     uint16_t adcValue = readADS7828Channel(I2C_LIST[3+i], j,i); // Read from channel 0
  
     float voltage = (adcValue / 4096.0) * 7.5; // Assuming a 5V reference
      Serial.print(" ");
      Serial.print(j);
      Serial.print(" : ");
      Serial.print(adcValue);
      Serial.print(", ");
      Serial.print(voltage);
      Serial.print(" V ; ");
      delay(100); // 
    }
    Serial.println("" );
  }
  
}

void print_errors()
{
if(!COM_printed)
    {
      COM_live = true;
       if(Modbus_server_started)
      {
        Serial.println("Modbus server started");
        if(I_O_ok) 
        {
          Serial.println("I/O organisation OK"); 
        }
        else
        {
          Serial.println("I/O organisation FAIL");
        }
      }
      if(ADS7828_error)
      {
        Serial.println(ADS7828_error);
        if(ADS7828_error & 0x01)
          Serial.println("ADS7828_1 not found!");
        if (ADS7828_error & 0x02) 
          Serial.println("ADS7828_2 not found!");  
      }
      else
        Serial.println("ADS7828 OK"); 

      //Print_I_O();
      COM_printed = true;
    }
  
  else
  {
    COM_printed = false; 
  }
}
 