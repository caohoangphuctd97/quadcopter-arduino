#include <Wire.h>

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;                                              // mặc định bằng 0 khi khai báo kiểu byte
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, start;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;
int throttle;
int esc_1, esc_2, esc_3, esc_4;
int pid_roll_setpoint,pid_pitch_setpoint,pid_yaw_setpoint;

float gyro_x, gyro_y, gyro_z;

int temperature;
float gyro_x_cal, gyro_y_cal, gyro_z_cal;
float Gyro_raw_error_x, Gyro_raw_error_y;

long loop_timer;
int lcd_loop_counter;

float angle_roll, angle_pitch;
float angle_roll_acc, angle_pitch_acc;;
float angle_X, angle_Y;

float rad_to_deg = 180/3.141592654;         //This value is for pasing from radians to degrees values
float acc_x, acc_y, acc_z,acc_total_vector;       
float acc_x_cal, acc_y_cal, acc_z_cal;
float Acc_angle_error_x, Acc_angle_error_y;
boolean set_gyro_angles;
float angle_pitch_output, angle_roll_output;


float Kp_roll = 3.25 ;               
double Ki_roll;// =0.0 ;  //0.6            
float Kd_roll;// =0.0;                
int max_roll = 400;                    

float Kp_pitch =3.25 ;  
float Ki_pitch ;  
float Kd_pitch =0.0;  
int max_pitch = max_roll;          

float Kp_yaw = 4.0;                
float Ki_yaw = 0.02;               
float Kd_yaw = 0.0;                
int max_yaw = 400;    

unsigned long time,timePrev,elapsedTime;


float error_roll, error_pitch, error_yaw;
float error_sum_roll, previous_error_roll, delta_err_roll;
float error_sum_pitch,previous_error_pitch,delta_err_pitch;
float error_sum_yaw, previous_error_yaw, delta_err_yaw; 
int pid_output_roll,pid_output_pitch,pid_output_yaw,battery_voltage;

void setup(){
  DDRD |= B11110000; 
  DDRB |= B00010000;
  Serial.begin(57600);
  setup_mpu_6050_registers(); 
 
  PCICR |= (1 << PCIE0);    
  PCMSK0 |= (1 << PCINT0); 
  PCMSK0 |= (1 << PCINT1);  
  PCMSK0 |= (1 << PCINT2);  
  PCMSK0 |= (1 << PCINT3);  

  int cout = 0;  
  while(cout < 2000){
    start ++;                                        
    PORTD |= B11110000;                              //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                         //Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
    PORTD &= B00001111;                              //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                        //Wait 3 milliseconds before the next loop.
    read_mpu_6050_data();                                              //        Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z; 
    cout += 1;
    if(start == 125){                                //Every 125 loops (500ms).
      digitalWrite(12, !digitalRead(12));            //Change the led status.
      start = 0;                                     //Start again at 0.
    }
  }
  start = 0;
 
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000; 
  
  for(int a=0; a<2000; a++){
    read_mpu_6050_data();
    acc_x_cal += acc_x/4096.0;
    acc_y_cal += acc_y/4096.0;
    acc_z_cal += acc_z/4096.0;
  }
  acc_x_cal /= 2000;
  acc_y_cal /= 2000;
  acc_z_cal /= 2000;
  
 

  acc_total_vector = sqrt((acc_x_cal*acc_x_cal)+(acc_y_cal*acc_y_cal)+(acc_z_cal*acc_z_cal));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  Acc_angle_error_x = asin((float)acc_y_cal/acc_total_vector)* 57.296;       //Calculate the pitch angle
  Acc_angle_error_y = asin((float)acc_x_cal/acc_total_vector)* -57.296;
  
  battery_voltage = (analogRead(0)) * 1.2317;
  zero_timer = micros();
  
}

void loop(){

  read_mpu_6050_data();
  gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;

  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle

  angle_pitch_acc -= Acc_angle_error_x;                                              //Accelerometer calibration value for pitch //trừ cho giá trị ban đầu khi đứng yên
  angle_roll_acc -= Acc_angle_error_y;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;

  
  battery_voltage = battery_voltage * 0.92 + (analogRead(0)) * 0.09853;
// if(analogRead(A2) != Ki_roll){ 
//  Ki_roll = analogRead(A2);
//  Ki_roll = map(Ki_roll,0,1023,0,500);
//   Ki_roll /=1000;
//  error_sum_pitch = 0;
//  error_sum_roll = 0;
// // Ki_pitch =Ki_roll; 
//  }

  Serial.print(esc_1);
  Serial.print("\t");
  Serial.print(esc_2);
  Serial.print("\t");
  Serial.print(esc_3);
  Serial.print("\t");
  Serial.print(esc_4);
//  Serial.print("\t");
//  Serial.println(pid_output_roll);
//  Serial.print("\t");
//  Serial.println(elapsedTime);
//  Serial.print("\t");
  Serial.print(angle_roll_output);
  Serial.print("\t");
  Serial.println(angle_pitch_output);
  pid_roll_setpoint = 0;
  if(receiver_input_channel_1 > 1516)pid_roll_setpoint = (receiver_input_channel_1 - 1516);
  else if(receiver_input_channel_1 < 1506)pid_roll_setpoint = (receiver_input_channel_1 - 1506);

  pid_roll_setpoint /= 10.9;                                                
  
  
  pid_pitch_setpoint = 0; 
  if(receiver_input_channel_2 > 1526)pid_pitch_setpoint = (receiver_input_channel_2 - 1526)/10.9;
 else if(receiver_input_channel_2 < 1516)pid_pitch_setpoint = (receiver_input_channel_2 - 1516)/10.9;                                                                               

  
//  pid_yaw_setpoint = 0;
//  
//  if(receiver_input_channel_3 > 1050){ 
//    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
//    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
//  }
  if(receiver_input_channel_3 < 1150 && receiver_input_channel_4 < 1130){
    start = 1;
    error_sum_roll=0;
    previous_error_roll=0;
    error_sum_pitch=0;
    previous_error_pitch=0;
    error_sum_yaw=0;
    previous_error_yaw=0;
  }
  if(start == 1 && receiver_input_channel_3 < 1150 && receiver_input_channel_4 > 1800)start = 0;
  calculate_pid();

  
  
  throttle = receiver_input_channel_3;
 
  if (start == 1){ 
  esc_1 = throttle - pid_output_roll + pid_output_pitch;
  esc_2 = throttle - pid_output_roll - pid_output_pitch;
  esc_3 = throttle + pid_output_roll - pid_output_pitch;
  esc_4 = throttle + pid_output_roll + pid_output_pitch;
   if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

//  esc_2 = 1000;
//  esc_4 = 1000;

  if (esc_1 < 1100) esc_1 = 1100;                                        
  if (esc_2 < 1100) esc_2 = 1100;                                         
  if (esc_3 < 1100) esc_3 = 1100;                                         
  if (esc_4 < 1100) esc_4 = 1100;                                         
  
  if(esc_1 > 1800)esc_1 = 1800;                                           
  if(esc_2 > 1800)esc_2 = 1800;                                           
  if(esc_3 > 1800)esc_3 = 1800;                                          
  if(esc_4 > 1800)esc_4 = 1800;
  }
  else{
    esc_1 = 1000;                                                           
    esc_2 = 1000;                                                           
    esc_3 = 1000;                                                           
    esc_4 = 1000;                                                           
  }
//  Serial.print(esc_2);
//  Serial.println(esc_4);
  while(micros() < 4000 + zero_timer);                 
  zero_timer = micros();                                     
  PORTD |= B11110000;
  timer_channel_1 = esc_1 + zero_timer ;   
  timer_channel_2 = esc_2 + zero_timer ;   
  timer_channel_3 = esc_3 + zero_timer ;   
  timer_channel_4 = esc_4 + zero_timer ;   
  
  while(PORTD >= 16){                                        
    esc_loop_timer = micros();                               
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111; 
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111; 
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111; 
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111; 
}
  
}
ISR(PCINT0_vect){
  //Channel 1=========================================
  if(last_channel_1 == 0 && PINB & B00000001 ){         //Input 8 changed from 0 to 1
    last_channel_1 = 1;                                 //Remember current input state
    timer_1 = micros();                                 //Set timer_1 to micros()
  }
  else if(last_channel_1 == 1 && !(PINB & B00000001)){  //Input 8 changed from 1 to 0
    last_channel_1 = 0;                                 //Remember current input state
    receiver_input_channel_1 = micros() - timer_1;      //Channel 1 is micros() - timer_1
  }
  //Channel 2=========================================
  if(last_channel_2 == 0 && PINB & B00000010 ){         //Input 9 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    timer_2 = micros();                                 //Set timer_2 to micros()
  }
  else if(last_channel_2 == 1 && !(PINB & B00000010)){  //Input 9 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
    receiver_input_channel_2 = micros() - timer_2;      //Channel 2 is micros() - timer_2
  }
  //Channel 3=========================================
  if(last_channel_3 == 0 && PINB & B00000100 ){         //Input 10 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    timer_3 = micros();                                 //Set timer_3 to micros()
  }
  else if(last_channel_3 == 1 && !(PINB & B00000100)){  //Input 10 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
    receiver_input_channel_3 = micros() - timer_3;      //Channel 3 is micros() - timer_3
  }
  //Channel 4=========================================
  if(last_channel_4 == 0 && PINB & B00001000 ){         //Input 11 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if(last_channel_4 == 1 && !(PINB & B00001000)){  //Input 11 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }
}
void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050 địa chỉ
  Wire.write(0x6B);                                                    //Send the requested starting register  thanh ghi năng lượng
  Wire.write(0x00);                                                    //Set the requested starting register   set chế độ
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register  thanh ghi cấu hình acc
  Wire.write(0x10);                                                    //Set the requested starting register   set 00010000
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register   set 00001000
  Wire.endTransmission();                                              //End the transmission
}
void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable 2byte lấy 1 byte dịch qua 8
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable ví dụ 0000000011101111 <<8 = 1110111100000000|10001000 = 1110111110001000
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}

void calculate_pid(){
  //Roll calculations
  
  error_roll = pid_roll_setpoint + angle_pitch_output;
//  if(-2>error_roll || error_roll>3)
  error_sum_roll +=  error_roll;
  
 
  delta_err_roll = (error_roll - previous_error_roll);
  previous_error_roll = error_roll;

  if (error_sum_roll >= 400) error_sum_roll = 400;
    else if (error_sum_roll <= -400) error_sum_roll = -400;

  // PID = e.Kp + ∫e.Ki + Δe.Kd
  pid_output_roll = (Kp_roll * error_roll) + (Ki_roll * error_sum_roll) +(Kd_roll* delta_err_roll*250);
  if (pid_output_roll >= 400) pid_output_roll = 400;
    else if (pid_output_roll <= -400) pid_output_roll = -400;
  
  
  //Pitch calculations
  error_pitch = pid_pitch_setpoint - angle_roll_output;
  
  if(-2>error_pitch || error_pitch>3)
    error_sum_pitch +=  error_pitch;
 
  delta_err_pitch = error_pitch - previous_error_pitch;
  previous_error_pitch = error_pitch;

  if (error_sum_pitch >= 400) error_sum_pitch = 400;
    else if (error_sum_pitch <= -400) error_sum_pitch = -400;

  // PID = e.Kp + ∫e.Ki + Δe.Kd
  pid_output_pitch = (Kp_pitch * error_pitch) + (Ki_pitch *error_sum_pitch) +(Kd_pitch* delta_err_pitch/4*1000);
  if (pid_output_pitch >= 400) pid_output_pitch = 400;
    else if (pid_output_pitch <= -400) pid_output_pitch = -400;

  
  
  //Yaw calculations
//  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
//  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
//  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
//  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
//  
//  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
//  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
//  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
//    
//  pid_last_yaw_d_error = pid_error_temp;
}
