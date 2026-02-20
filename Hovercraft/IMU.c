// All the definitions we need for the IMU
#define ACCEL_SCALE 16384.0   // This value we will have to change for the experiments (+-2)
#define GYRO_SENSITIVITY 131.0 // This value we will have to change for the experiments (+-250)
#define IMU_ADDRESS 0x68
#define ACCEL_XOUT_H 0x3B
#define SLEEP_MODE_ADDRESS 0x00
#define POWER_MAN_ADDRESS 0x6B
#define ACC_CONFIG_ADDRESS 0x1C
#define GYRO_CONFIG_ADDRESS 0x1B
#define GYRO_SAMPLE_ADDRESS 0x00
#define ACC_SAMPLE_ADDRESS 0x00
#define F_I2C 400000
#define I2C_READ 1
#define I2C_WRITE 0

// LED pins definition
#define LED_ACC_PIN PB3
#define LED_YAW_PIN PB5

// All the variables needed for code
int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
float ax, ay, az, gx, gy, gz;
float yaw = 0, roll = 0, pitch = 0;
float dt;
float gz_bias = 0.0, ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;
unsigned long previous_time = 0;
unsigned long ms_count = 0;
float vx = 0, prev_vx = 0, x_pos = 0;
float ax_f = 0, ax_hp = 0, ax_lp = 0, ax_p = 0;
uint8_t bright = 0;
const float ALPHA_LP = 0.2, ALPHA_HP = 0.98;

//printing function prototypes
void printAccel();
void printDistanceX();
void printRoll();
void printPitch();
void printYaw();

/********************************************************/
/**************** Current Time Functions ****************/
/********************************************************/
/*
Interupt service routine, gets called every 1ms and increments ms_count
*/
ISR(TIMER0_COMPA_vect){
  ms_count++;
}

/*
Custom delay function (ms)
*/
void myDelay(uint32_t msTime) {
    uint32_t startTime = ms_count;
    while ((ms_count - startTime) < msTime);
}

/*
  Timer 0 will be used for our own delay function
  Works the same way as we did in assignment 1, so we have ticks of 4us, then we count from 0 to 249 (250 ticks)
  On compare, we call the ISR and we clear the register (set it to 0).
  This calls the ISR every 1ms, so we can count the ms.
*/
void timer0Setup(){
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
    sei();
}
/********************************************************/
/******************* Current Time End *******************/
/********************************************************/


/*****************************************************/
/**************** Start I2C Functions ****************/
/*****************************************************/
// WE enable pull up resistor on both SDA and SCL
void I2CSetup(){
  TWSR = 0;
  TWBR = ((F_CPU/F_I2C) - 16)/2;
}

void I2CStart(){
  uint8_t twst;
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));
  twst = TW_STATUS & 0xF8;
}

void I2CStop(){
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
  while(TWCR & (1<<TWSTO));
}

unsigned char I2CWrite(unsigned char data)
{
  uint8_t twst;
  TWDR = data;
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));
  twst = TW_STATUS & 0xF8;
  if (twst != TW_MT_DATA_ACK && twst != TW_MR_SLA_ACK) return 1;
  return 0;
}

unsigned char I2CReadACK(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
  while(!(TWCR & (1<<TWINT)));
  return TWDR;
}

unsigned char I2CReadNACK(void)
{
  TWCR = (1<<TWINT) | (1<<TWEN);
  while(!(TWCR & (1<<TWINT)));
  return TWDR;
}

/********************************************************/
/******************* End I2C Function *******************/
/********************************************************/

/*********************************************************/
/**************** Start MPU6050 Functions ****************/
/*********************************************************/

/*
We start the I2C
Write the address oif the IMU (0x68)
Write the address of the power management
Write in power management address to disable sleep mode (wake up)
Then we configure the ADC of both the gyroscope and acceleration (look at datasheet page 12 and 13)
*/
/*********************************************************/
/**************** Start MPU6050 Functions ****************/
/*********************************************************/

void MPU6050Setup(){
  I2CStart();
  I2CWrite(IMU_ADDRESS << 1);
  I2CWrite(POWER_MAN_ADDRESS);
  I2CWrite(SLEEP_MODE_ADDRESS);
  I2CStop();
// Set accelerometer configuration (full scale ±2g)
  I2CStart();
  I2CWrite(IMU_ADDRESS << 1);
  I2CWrite(0x1C);
  I2CWrite(ACC_SAMPLE_ADDRESS);
  I2CStop();

  I2CStart();
  I2CWrite(IMU_ADDRESS << 1);
  I2CWrite(0x1B);
  I2CWrite(GYRO_SAMPLE_ADDRESS);
  I2CStop(); 
}

void MPU6050ReadRaw(){
  I2CStart();
  I2CWrite(IMU_ADDRESS<<1);
  I2CWrite(ACCEL_XOUT_H);
  I2CStart();
  I2CWrite((IMU_ADDRESS<<1) | 1);

  ax_r = (I2CReadACK()<<8) | I2CReadACK();
  ay_r = (I2CReadACK()<<8) | I2CReadACK();
  az_r = (I2CReadACK()<<8) | I2CReadACK();
  uint16_t trash = (I2CReadACK()<<8) | I2CReadACK();
  gx_r = (I2CReadACK()<<8) | I2CReadACK();
  gy_r = (I2CReadACK()<<8) | I2CReadACK();
  gz_r = (I2CReadACK()<<8) | I2CReadNACK();
  I2CStop();
}

void calibrateMPU6050(void) {
    int num_samples = 2000;
    long gz_sum = 0, ax_sum = 0, ay_sum = 0, az_sum = 0;

    for (int i = 0; i < num_samples; i++) {
        MPU6050ReadRaw();
        gz_sum += gz_r;
        ax_sum += ax_r;
        ay_sum += ay_r;
        az_sum += az_r;
        _delay_ms(1);
    }

    gz_bias = gz_sum / num_samples;
    ax_bias = (ax_sum / num_samples) / ACCEL_SCALE;
    ay_bias = (ay_sum / num_samples) / ACCEL_SCALE;
    az_bias = (az_sum / num_samples) / ACCEL_SCALE - 1.0;
}

void calculateAllAngles() {
    ax = ((float)ax_r / ACCEL_SCALE) - ax_bias;
    ay = ((float)ay_r / ACCEL_SCALE) - ay_bias;
    az = ((float)az_r / ACCEL_SCALE) - az_bias;


    roll = atan2(ay, az) * 180.0 / M_PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;


    dt = (ms_count - previous_time) / 1000.0;
    previous_time = ms_count;


    float gz = ((float)gz_r - gz_bias) / GYRO_SENSITIVITY;
    yaw += gz * dt;
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
}

void calculateXDistance(){
  ax_f = ALPHA_LP * ax + (1-ALPHA_LP) * ax_f;
  ax_hp = ALPHA_HP * (ax_hp + ax_f - ax_p);
  ax_p = ax_f;
  
  if(fabs(ax_hp) < 0.01){
    ax_hp = 0.0;
  }

  float ax_ms = ax_hp * 9.81;
  vx = prev_vx + ax_ms* dt;
  vx *= 0.99;

  x_pos += ((prev_vx + vx)/2) * dt;
  prev_vx = vx;
  ax = ax_hp;
}

/*
Function that prints the acceleration values of x,y and z (in g)
*/
void printAccel(){
  char arrayAccelX[10];
  char arrayAccelY[10];
  char arrayAccelZ[10];
  dtostrf(ax, 6, 2, arrayAccelX);
  dtostrf(ay,6,2, arrayAccelY);
  dtostrf(az,6,2, arrayAccelZ);
  UART_TransmitArray("ax: ");
  UART_TransmitArray((uint8_t*)arrayAccelX); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
  UART_TransmitArray("ay: ");
  UART_TransmitArray((uint8_t*)arrayAccelY); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
  UART_TransmitArray("az: ");
  UART_TransmitArray((uint8_t*)arrayAccelZ); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
}

/*
Function that prints the distance in X of the IMU
*/
void printDistanceX(){
  char arrayDistanceX[10];
  dtostrf(x_pos, 6, 2, arrayDistanceX);
  UART_TransmitArray("Distance along x: ");
  UART_TransmitArray((uint8_t*)arrayDistanceX); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
}

/*
Function that prints the roll of the IMU.
*/
void printRoll(){
  char arrayRoll[10];
  dtostrf(roll, 6, 1, arrayRoll);
  UART_TransmitArray("Roll: ");
  UART_TransmitArray((uint8_t*)arrayRoll); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
}

/*
Function that prints the pitch of the IMU.
*/
void printPitch(){
  char arrayPitch[10];
  dtostrf(pitch, 6, 1, arrayPitch);
  UART_TransmitArray("Pitch: ");
  UART_TransmitArray((uint8_t*)arrayPitch); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
}

/*
Function that pritns the yaw of the IMU.
*/
void printYaw(){
  char arrayYaw[10];
  dtostrf(yaw, 6, 1, arrayYaw);
  UART_TransmitArray("Yaw: ");
  UART_TransmitArray((uint8_t*)arrayYaw); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
}

/*
Function that prints all values.
*/
void printAll(){
  printDistanceX();
  printAccel();
  printRoll();
  printPitch();
  printYaw();
}
/*
Simple pin setup for the LEDs
*/
void angleLEDPinSetup(){
  DDRB |= (1 << LED_YAW_PIN) | (1<< LED_ACC_PIN);
}

/*
Works the same way as Assignment 1
*/
void timer2Setup(){
  TCCR2A |= (1 << WGM20) | (1 << WGM21) | (1<<COM2A0) | (1<<COM2A1);    
  TCCR2B |= (1 << CS21);
}

void imuSetup(){
  angleLEDPinSetup();
  timer0Setup();
  timer2Setup();
  I2CSetup();
  MPU6050Setup();
}

/** Change Brightness **/
// If |ax| >= 1.12, brightness is set to 100% = (255/255).
// If |ax| <= 0.12, brightness is set to 0% = (0/255).
// Otherwise, the brightness is calculated as a linear scale between 0 and 255 based on |ax|.
// The calculated brightness is applied by setting OCR2A to the brightness value, which controls the PWM duty cycle.
// The timer will then adjust the LED's brightness based on the PWM duty cycle and repeat every cycle.
/**void ledBrightness(){
  if (fabs(ax) >= 1.12){ bright = 255;}
  else if (fabs(ax) <= 0.12){ bright = 0;}
  else{ bright = (uint8_t)255*(fabs(ax) - 0.12);}
  OCR2A = bright;
}
**/

/*************************************************************/
/******************* End MPU6050 Functions *******************/
/*************************************************************/
