#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

// Stupid things for IMU calibration
#define ACCEL_SCALE 16384.0   // This value we will have to change for the experiments (depends on ACC_ADC_ADDRESS)
#define GYRO_SENSITIVITY 131.0 // This value we will have to change for the experiments (depends on GYRO_ADC_ADDRESS)

// Stupid IMU addresses
#define IMU_ADDRESS 0x68
#define ACCEL_XOUT_H 0x3B
#define SLEEP_MODE_ADDRESS 0x00
#define POWER_MAN_ADDRESS 0x6B
#define ACC_CONFIG_ADDRESS 0x1C
#define GYRO_CONFIG_ADDRESS 0x1B
#define ACC_ADC_ADDRESS 0x00  // This value we will have to change for the experiments
#define GYRO_ADC_ADDRESS 0x00  // This value we will have to change for the experiments
// Stupid I2C delay
#define I2C_DELAY_US 5

// Stupid Pins
#define SDA_PIN PC4
#define SCL_PIN PC5

// Stupid variables
int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
float ax, ay, az, gx, gy, gz;
float yaw = 0, roll = 0, pitch = 0;
float dt;
float gz_bias = 0.0, ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;
unsigned long previous_time = 0;
unsigned long ms_count = 0;
float vx = 0, x_pos = 0;


/********************************************************/
/**************** Current Time Functions ****************/
/********************************************************/
ISR(TIMER0_COMPA_vect){
  ms_count++;
}

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
void I2CSetup(void) {
    PORTC |= (1 << SDA_PIN) | (1 << SCL_PIN);
}

// These functions are basically to set the pins to high and low 
void SDAHigh(void) {
    PORTC |= (1 << SDA_PIN);
    DDRC &= ~(1 << SDA_PIN); // Input mode
}

void SDALow(void) {
    DDRC |= (1 << SDA_PIN);  // Output mode
    PORTC &= ~(1 << SDA_PIN); // Low
}

void SCLHigh(void) {
    PORTC |= (1 << SCL_PIN);
    DDRC &= ~(1 << SCL_PIN); // Input mode
}

void SCLLow(void) {
    DDRC |= (1 << SCL_PIN);  // Output mode
    PORTC &= ~(1 << SCL_PIN); // Low
}

// This function just reads the state of SDA
uint8_t SDARead(void) {
    return (PINC & (1 << SDA_PIN)) != 0;
}

// For the Start and Stop conditions, look at page 33 and 34 of the MPU6050 datasheet
void I2CStart(void) {
    SDAHigh();
    SCLHigh();
    _delay_us(I2C_DELAY_US);
    SDALow();
    _delay_us(I2C_DELAY_US);
    SCLLow();
}

void I2CStop(void) {
    SDALow();
    _delay_us(I2C_DELAY_US);
    SCLHigh();
    _delay_us(I2C_DELAY_US);
    SDAHigh();
    _delay_us(I2C_DELAY_US);
}

// For the data communicatio (write and read), look at page 34-36 in the MPU6050 datasheet
uint8_t I2CWrite(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) {
            SDAHigh();
        } else {
            SDALow();
        }
        data <<= 1;
        SCLHigh();
        _delay_us(I2C_DELAY_US);
        SCLLow();
        _delay_us(I2C_DELAY_US);
    }
    SDAHigh();
    SCLHigh();
    _delay_us(I2C_DELAY_US);
    uint8_t ack = !(SDARead());
    SCLLow();
    return ack;
}

uint8_t I2CRead(uint8_t ack) {
    uint8_t data = 0;
    SDAHigh();
    for (uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        SCLHigh();
        _delay_us(I2C_DELAY_US);
        if (SDARead()) {
            data |= 1;
        }
        SCLLow();
        _delay_us(I2C_DELAY_US);
    }
    if (ack) {
        SDALow();
    } else {
        SDAHigh();
    }
    SCLHigh();
    _delay_us(I2C_DELAY_US);
    SCLLow();
    return data;
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
void MPU6050Setup(){
  I2CStart();
  I2CWrite(IMU_ADDRESS << 1);
  I2CWrite(POWER_MAN_ADDRESS);
  I2CWrite(SLEEP_MODE_ADDRESS);
  I2CStop();

  // Acceleration
  I2CStart();
  I2CWrite(IMU_ADDRESS << 1);
  I2CWrite(ACC_CONFIG_ADDRESS);           
  I2CWrite(ACC_ADC_ADDRESS);            
  I2CStop();

  // Gyroscope
  I2CStart();
  I2CWrite(IMU_ADDRESS << 1);
  I2CWrite(GYRO_CONFIG_ADDRESS);            
  I2CWrite(GYRO_ADC_ADDRESS);             
  I2CStop(); 
}


void MPU6050ReadRaw(){
  I2CStart();
  I2CWrite(IMU_ADDRESS<<1);
  I2CWrite(ACCEL_XOUT_H);
  I2CStart();
  I2CWrite((IMU_ADDRESS<<1) | 1);

  // Basically read the data, then move it 8 bits, because each hex is 4 bits, and we are skipping 2 (3B to 3D)
  ax_r = (I2CRead(1)<<8) | I2CRead(1);
  ay_r = (I2CRead(1)<<8) | I2CRead(1);
  az_r = (I2CRead(1)<<8) | I2CRead(1);

  // Things between acceleration and gyroscope
  uint16_t trash = (I2CRead(1)<<8) | I2CRead(1);

  gx_r = (I2CRead(1)<<8) | I2CRead(1);
  gy_r = (I2CRead(1)<<8) | I2CRead(1);
  gz_r = (I2CRead(1)<<8) | I2CRead(0);
  I2CStop();
}

// Stupid ass IMU is so sensitive, so you have to like basically create a bias of movement, so like try to see when its technically not really moving, what the average is of movement, if you dont do this, shit goes haywire.
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

void calculateAccGyro(){
  ax = ((float)ax_r / ACCEL_SCALE) - ax_bias;
  ay = ((float)ay_r / ACCEL_SCALE) - ay_bias;
  az = ((float)az_r / ACCEL_SCALE) - az_bias;
  gz = ((float)gz_r - gz_bias) / GYRO_SENSITIVITY;
}

//Calculate delta t for integration
void deltaT(){
  dt = (ms_count - previous_time)/1000.0;
  previous_time = ms_count;
}

// Calculate the position in x, basically have to work with which values oh acceleration we should take care of
void distanceX(){
  if(fabs(ax) > fabs(ax_bias)){
    vx += ax * dt;
    x_pos += vx * dt;
  }
}

// Basically functions to find angles
void calculateRoll(float ay, float az){
  roll = atan2(ay,az) * 180.0 / M_PI;
}

void calculatePitch(float ax, float ay, float az){
  pitch = atan2(-ax, sqrt(ay*ay + az*az))*180.0 / M_PI;
}

void calculateYaw(float gz){
  gz = ((float)gz_r - gz_bias)/GYRO_SENSITIVITY;
  yaw += gz*dt;
  if(yaw > 180){yaw -= 360;}
  if(yaw < -180){yaw+=360;}          
}

// Function to calculate all values
void calculateAllValues(){
  deltaT();
  calculateAccGyro();
  distanceX();
  calculateRoll();
  calculatePitch();
  calculateYaw();
}

/*************************************************************/
/******************* End MPU6050 Functions *******************/
/*************************************************************/

































































