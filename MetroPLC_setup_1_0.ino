void setup() 
{
  ready_for_MODBUS = 0;
  tc5_counter = 0;
  GCLK->GENCTRL[6].reg = GCLK_GENCTRL_DIV(3) |       // Divide the 48MHz clock source by divisor 3: 48MHz/3 = 16MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK6
                         //GCLK_GENCTRL_SRC_DFLL;      // Generate from 48MHz DFLL clock source
                         //GCLK_GENCTRL_SRC_DPLL1;     // Generate from 100MHz DPLL clock source
                        GCLK_GENCTRL_SRC_DPLL0;     // Generate from 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL6);               // Wait for synchronization 

  // Enable GCLK0 for TC5
  GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK6 | GCLK_PCHCTRL_CHEN;
  // Disable TC5 before configuration
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC5->COUNT16.SYNCBUSY.bit.ENABLE);
  // Configure TC5: 16-bit counter, prescaler DIV8, and match PWM mode (works for generating interrupts)
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_PRESCALER_DIV8 | // Set prescaler to 16, 16MHz/16 = 1MHz
                           TC_CTRLA_PRESCSYNC_PRESC | // Set the reset/reload to trigger on prescaler clock
                           TC_CTRLA_MODE_COUNT16;
  while (TC5->COUNT16.SYNCBUSY.bit.CTRLB);
  TC5->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MPWM;
 

  // Set compare match value for 100 microseconds
  TC5->COUNT16.CC[0].reg = 500;
  while (TC5->COUNT16.SYNCBUSY.bit.CC0);
    TC4->COUNT16.CC[1].reg = 1;          // Pulse
  while (TC4->COUNT16.SYNCBUSY.bit.CC1);
  // Enable interrupt on compare match
  TC5->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  NVIC_EnableIRQ(TC5_IRQn);
  // Enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC5->COUNT16.SYNCBUSY.bit.ENABLE);
  Serial.begin(9600);
  auto_update_RGB = 1;
  Wire.begin(); // Initialize PCAL6416A and the PCA9539R manually
  I2C_pointer_1 = io_port_1_2;
    for (int i = 0; i < 3; i++) 
    {
      Wire.beginTransmission(I2C_LIST[I2C_pointer_1+i]);
      Wire.write(0x02);
      Wire.write(0x00); // Set output d-latches inputs to 0x00 (latches at STOP)
      Wire.write(0x00);
      if (Wire.endTransmission() != 0) {
          Serial.print("Error in setting output latches ");
          Serial.println(i);
      }
  
      Wire.beginTransmission(I2C_LIST[I2C_pointer_1+i]);
      Wire.write(0x04); // Configuration register for Port 1
      Wire.write(I2C_IO_POL_LIST[(2*i)]); // invert polarity PORT 1
      Wire.write(I2C_IO_POL_LIST[(2*i)+1]); // invert polarity PORT 2
      if (Wire.endTransmission() != 0) 
      {
        Serial.print("Error in setting polarity configuration ");
        Serial.println(i);
      }
      Wire.beginTransmission(I2C_LIST[I2C_pointer_1+i]);
      Wire.write(0x06); // Configuration register for Port 0
      Wire.write(0x00); // Set all Port 0 pins as outputs
      Wire.write(0x00); // Set all Port 1 pins as outputs
      if (Wire.endTransmission() != 0) 
      {
        Serial.print("Error in setting port configuration ");
        Serial.println(i);
      }
    }//for 0 to 3
    I2C_pointer_1 = adc_1;
    ADS7828_error = 0;//reset errorflag
  for (int i = 0;i<2;i++)
  {
    Wire.beginTransmission(I2C_LIST[3+i]);  
    if (Wire.endTransmission() != 0) 
    {
      ADS7828_error += (i+1);
    }
  }
  Board_RGB.begin(); // Initialize the NeoPixel library
  Board_RGB.setPixelColor(0, Board_RGB.Color(255, 0, 0)); // Set the first NeoPixel to red
  Board_RGB.show();

  GyverTM1637 *segment_ptr;
  segment_ptr = &sevenseg_1;

  for (int i = 0; i < 6; i++) 
  {
    segment_ptr->clear();
    segment_ptr->brightness(7);
    segment_ptr->displayInt(0000);
    segment_ptr ++;
  }

  tc5_counter = 0;
  main_counter_1 = 0;
  ms_counter = 0;
  s_counter = 0;

  I_O_ok = true;
  COM_printed = 0;
  blink_led_running = 0;
  Modbus_server_started = 0;
  packet_counter = 0;

  //ModbusRTUServer.setTimeout(10);
  // start the Modbus RTU server, with (slave) id 10
  RS485.setPins(TX_PIN, DE_PIN, RE_PIN);//TX/DE/RE
  //ModbusServer.setTimeout(10);
  modbus_init();

  pinMode(PULSE_PIN_1,INPUT);
  pinMode(PULSE_PIN_2,INPUT);
  pinMode(PULSE_PIN_3,INPUT);
  pinMode(PULSE_PIN_4,INPUT);
  pinMode(PULSE_PIN_5,INPUT);
  pinMode(PULSE_PIN_6,INPUT);
  pinMode(PULSE_PIN_7,INPUT);
  pinMode(PULSE_PIN_8,INPUT);
  pinMode(THERMAL_PIN_1,INPUT);
  pinMode(THERMAL_PIN_2,INPUT);
  pinMode(DIAG_1_PIN,INPUT);
  pinMode(DIAG_2_PIN,INPUT);
  pinMode(INT_1_PIN,INPUT);
  pinMode(INT_RELAIS_PIN,INPUT);
  pinMode(ERR_1_PIN,INPUT);
  pinMode(ERR_2_PIN,INPUT);
  pinMode(C_SENS_1_PIN,INPUT);
  pinMode(C_SENS_2_PIN,INPUT);

  pinMode(RELAIS_1_PIN,OUTPUT);
  digitalWrite(RELAIS_1_PIN,0);
  pinMode(RELAIS_2_PIN,OUTPUT);
  digitalWrite(RELAIS_2_PIN,0);

  pinMode(RESET_I2C_RELAIS_PIN,OUTPUT);
  digitalWrite(RESET_I2C_RELAIS_PIN,0);
  pinMode(RESET_HUB_PIN,OUTPUT);
  digitalWrite(RESET_HUB_PIN,0);
  pinMode(RESET_I2C_1_PIN,OUTPUT);
  digitalWrite(RESET_I2C_1_PIN,0);
  pinMode(RESET_I2C_2_PIN,OUTPUT);
  digitalWrite(RESET_I2C_2_PIN,0);
  pinMode(SE_H_1_PIN,OUTPUT);
  digitalWrite(SE_H_1_PIN,0);
  pinMode(SE_L_1_PIN,OUTPUT);
  digitalWrite(SE_L_1_PIN,0);
  pinMode(SE_H_2_PIN,OUTPUT);
  digitalWrite(SE_H_2_PIN,0);
  pinMode(SE_L_2_PIN,OUTPUT);
  digitalWrite(SE_L_2_PIN,0);

  pinMode(ledRed, OUTPUT);
  digitalWrite(ledRed, LOW);

    for(int i=0;i< 8;i++)
  {
    ModbusRTUServer.holdingRegisterWrite(i,1);
    ModbusRTUServer.holdingRegisterWrite(i+8,0);  // pulsers offset
    ModbusRTUServer.holdingRegisterWrite(i+16,100); // pulsers span percent
  }
  digitalWrite(1, 1);
  delay(3000);
  digitalWrite(1, 0);
  ready_for_MODBUS = 1;
 print_errors();
}

void modbus_init()
{
 if (!ModbusRTUServer.begin(0x0A, 76800,SERIAL_8E1)) 
  {
    Modbus_server_started = 0;
  }
  else
  {
    Modbus_server_started = 1;
    if (!ModbusRTUServer.configureCoils(0x00, numCoils)) 
    {
      I_O_ok = false; 
    }
    if (!ModbusRTUServer.configureDiscreteInputs(0x00, numDiscreteInputs)) 
    {
      I_O_ok = false; 
    }  
    if (!ModbusRTUServer.configureInputRegisters(0x00, numInputRegisters)) 
    {
      I_O_ok = false; 
    }  
    if (!ModbusRTUServer.configureHoldingRegisters(0x00, numHoldingRegisters)) 
    {
      I_O_ok = false; 
    }  
    for(int i=0;i<32;i++) //init to 0
    {
      ModbusRTUServer.coilWrite(i,0);
      ModbusRTUServer.discreteInputWrite(i,0);
      ModbusRTUServer.inputRegisterWrite(i,0);
      ModbusRTUServer.holdingRegisterWrite(i,0);
      ModbusRTUServer.holdingRegisterWrite(i+32,0);
    }
  }
}