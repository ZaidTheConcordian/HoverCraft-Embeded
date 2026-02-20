/****all libraries****/
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <math.h>
#include <inttypes.h>
#include <compat/twi.h>
/*********************/


/****Definitions for the Servo****/
#define USS_SERVO_PIN PB1 // Will be controlled with OCR1A
#define RUDDER_SERVO_PIN PB2 // Will be controlled with OCR1B
#define MIN_PULSE 1000  // Min pulse of this servo is 600us, but each tick of our timer is 0.5us
#define MAX_PULSE 5000  // Max pulse of this servo is 2400us, but each tick of our timer is 0.5us
void servoSetup(void);
void timer1Setup(void);
void setPosUSS(long angle);
void setPosRudder(long angle);
/*********************************

/****Definitions for the USS****/
#define TRIGGER_PIN PB5
#define ECHO_PIN PD3
const long min_dist = 65;
const long max_dist = 235;
const uint8_t min_speed = 230;
const uint8_t max_speed = 255;
volatile uint32_t us_count = 0;
void timer2Setup(void);
unsigned long myMicros(void);
void USSSetup(void);
long calculateDistanceUS(void);
/*******************************/

/****Definitions for the fans****/
#define RUDDER_FAN_PIN PD6 // OC0A
#define LIFT_FAN_PIN PD5 // OC0B
void fanSetup(void);
void timer0Setup(void);
void setThrustFanSpeed(uint8_t speed);
void setLiftFanSpeed(uint8_t speed);
/********************************/

/****Definitions for the IRS****/
#define IR_PIN PC0
const float VREF = 2.6;
void IRSetup(void);
uint16_t readADC(void);
float calculateDistanceIR(uint16_t adcValue);
/*******************************/

/****Definitions for the IMU****/
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
#define F_CPU 16000000UL
#define WHO_AM_I 0x75
int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
float ax, ay, az, gx, gy, gz;
float yaw = 0, roll = 0, pitch = 0;
float dt;
float gz_bias = 0.0, ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;
uint32_t previous_time = 0;
float vx = 0, prev_vx = 0, x_pos = 0;
float ax_f = 0, ax_hp = 0, ax_lp = 0, ax_p = 0;
bool IMU_On = false;
const float ALPHA_LP = 0.2, ALPHA_HP = 0.98;
void I2CSetup(void);
void I2CStart(void);
void I2CStop(void);
unsigned char I2CWrite(unsigned char data);
unsigned char I2CReadACK(void);
unsigned char I2CReadNACK(void);
bool checkMPU6050Connection(void);
void MPU6050Setup(void);
void MPU6050ReadRaw(void);
void calibrateMPU6050(void);
void calculateAllAngles(void);
void calculateXDistance(void);
/*******************************/

/****Definitions of main program****/
int turnCounter = 0;
float IRDistance = 0;
boolean IRCheck = false;
float baseDistance = 65;
float scale = 250;
bool justTurned = false;
uint32_t turnCD = 0;
long distance_f, distance_r, distance_l;
uint32_t prev = 0;
long distanceDiff = 15;
void setUpAll(void);
int stop(void);
bool checkFinishLine(long distance_u);
void moveForward(void);
void navigation(long distance_f);
void adjustRudders(void);
void turnRight(void);
void turnLeft(void);
/***********************************/

/****Main Function****/
int main() {
    start();
    while (1) {
      adjustRudders();
      adjustUSS();
      navigation(averageDistance());
      IRDistance = calculateDistanceIR(readADC());
      //Serial.println(IRDistance);
      if(checkFinishLine(IRDistance)){
        stop();
        return 0;
      }
      moveForward();
      _delay_ms(10);
    }
    return 0;
}
/*************************************************************/
/******************** End Main Functions *********************/
/*************************************************************/

/*******************************************************************************************************************************************************************************************************************************/
/************************************************** This section is reserved to place all the code of all the different functions used for the logic of out hovercraft *********************************************************/
/*******************************************************************************************************************************************************************************************************************************/
void setUpAll(){
  //Serial.begin(9600);
    timer0Setup();
    timer1Setup();
    timer2Setup();
    fanSetup();
    servoSetup();
    USSSetup();
    IRSetup();
    MPU6050Setup();
    sei();
}
   
// Function to Navigate, when we want to turn
void navigation(long distance_f){
    uint32_t curr_time = myMicros();

    if(distance_f < calculateThresholdDistance()){
        setLiftFanSpeed(130);
        _delay_ms(500);
        setThrustFanSpeed(80);
        _delay_ms(500);
        
        setPosUSS(175);
        _delay_ms(1000);
        distance_l = averageDistance();
        _delay_ms(100);

        setPosUSS(5);
        _delay_ms(1000);
        distance_r = averageDistance();
        _delay_ms(100);

        setPosUSS(90);
        _delay_ms(1000);

        if(distance_l < distance_r){
            turnRight();
            _delay_ms(500);
        } else {
            turnLeft();
            _delay_ms(500);
        }

        justTurned = true;
        turnCD = myMicros();
    }
}

float calculateThresholdDistance(){
    MPU6050ReadRaw();
    calculateAllAngles();

    // Exponential scaling with vx
    float threshold = baseDistance + scale * (1 - exp(-fabs(vx)));
    if(threshold > 180) threshold = 180;
    return threshold;
}

void resetIMU(){
  turnCounter++;
  baseDistance -= 5;
  if(turnCounter == 2){
    _delay_ms(500);
    baseDistance -= 5;
  }
  _delay_ms(250);
    setThrustFanSpeed(100);
    setLiftFanSpeed(100);
    _delay_ms(1000);
    yaw = 0;
    roll = 0;
    pitch = 0;
    vx = 0;
    prev_vx = 0;
    x_pos = 0;
    ax = 0;
    ay = 0;
    az = 0;
    ax_f = 0;
    ax_hp = 0;
    ax_lp = 0;
    ax_p = 0;
    previous_time = myMicros();
    _delay_ms(100);
    MPU6050ReadRaw();
    calculateAllAngles();
    yaw = 0;
    previous_time = myMicros();
    setLiftFanSpeed(255);
    if(turnCounter > 2) baseDistance = 45;
}


int stop(){
    setLiftFanSpeed(0);
    setThrustFanSpeed(0);
    return 1;
}

bool checkFinishLine(long distance_u){
    //Serial.println(distance_u);
    if(myMicros() > 30000000){
      if(distance_u < 30){
        if(IRCheck){
          return true;
        }
        IRCheck = true;
        return false;
      }
    }
    IRCheck = false;
    return false;
}

void moveForward(){
   setThrustFanSpeed(255);
}

void adjustUSS(){
  MPU6050ReadRaw();
  calculateAllAngles();
  setPosUSS(90-yaw);
}

void start(){
  I2CSetup();
  bool IMU_On = checkMPU6050Connection();
  setUpAll();
  setPosUSS(90);
  setPosRudder(90);
  calibrateMPU6050();
  _delay_ms(250);
  setLiftFanSpeed(255);
  _delay_ms(1500);
  setThrustFanSpeed(255);
}

void adjustRudders(){
  MPU6050ReadRaw();
  calculateAllAngles();
  setPosRudder(85+(yaw/0.90)-(roll/2));
}

void turnRight(){
  setPosUSS(78);
  setLiftFanSpeed(255);
  setThrustFanSpeed(200);
  setPosRudder(135);
  float currentDistance = averageDistance();
  int validCount = 0;
  while (validCount < 2) {
      if (currentDistance >= 150 && currentDistance <= 190) {
          validCount++;
      } else {
          validCount = 0;
      }
      currentDistance = averageDistance();
  }
  setPosRudder(90);
  resetIMU();
}

void turnLeft(){
  setPosUSS(102);
  setLiftFanSpeed(255);
  setThrustFanSpeed(200);
  setPosRudder(45);
  float currentDistance = averageDistance();
  int validCount = 0;
  while (validCount < 2) {
      if (currentDistance >= 150 && currentDistance <= 190) {
          validCount++;
      } else {
          validCount = 0;
      }
      currentDistance = averageDistance();
  }
  setPosRudder(90);
  resetIMU();
}

long averageDistance(){
  long sum = 0;
  for(int i =0; i<2; i++){
    sum+= calculateDistanceUS();
  }
  return sum/2.0;
}





/*******************************************************************************************************************************************************************************************************************************/
/************************************ This section is reserved to place all the code of all the different components, it aslo includes the deinitions and the variables for each component *************************************/
/*******************************************************************************************************************************************************************************************************************************/

/*************************************************************/
/************************** Servos ***************************/
/*************************************************************/

void timer1Setup() {
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 40000;
}

void servoSetup() {
    DDRB |= (1 << USS_SERVO_PIN) | (1 << RUDDER_SERVO_PIN);
}

void setPosUSS(long angle){
    long pulse = 0;
    if(angle <= 0){
        pulse = MIN_PULSE;
    }
    else if(angle >= 180){
        pulse = MAX_PULSE;
    }
    else{
        pulse = MIN_PULSE + ((MAX_PULSE - MIN_PULSE) * angle) / 180;
    }
    OCR1A = pulse;
}

void setPosRudder(long angle){
    long pulse = 0;
    if(angle <= 25){
        pulse = 2389;
    }
    else if(angle >= 155){
        pulse = 4444;
    }
    else{
        pulse = MIN_PULSE + ((MAX_PULSE - MIN_PULSE) * angle) / 180;
    }
    OCR1B = pulse;
}

/*************************************************************/
/*************************************************************/
/*************************************************************/

/*************************************************************/
/**************************** USS ****************************/
/*************************************************************/
ISR(TIMER2_OVF_vect){
  us_count += 256;
}

void timer2Setup(){
  TCCR2A = 0;
  TCCR2B = (1 << CS21);
  TIMSK2 = (1<<TOIE2);
  TCNT2 = 0;
}

unsigned long myMicros(){
  uint8_t t;
  cli();
  t = TCNT2;
  uint32_t time = us_count + t;  
  sei();
  return time / 2;
}

void USSSetup(){
    DDRB |= (1<<TRIGGER_PIN);
    DDRB &= ~(1 << ECHO_PIN);
}

long calculateDistanceUS() {
    unsigned long duration = 0;
    long distance = 0;
    PORTB &= ~(1 << TRIGGER_PIN);
    _delay_us(2);
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);
    while (!(PIND & (1 << ECHO_PIN)));
    uint32_t startTime = myMicros();
    while (PIND & (1 << ECHO_PIN));
    uint32_t endTime = myMicros();
    duration = endTime - startTime;
    distance = (duration * 0.0343)/2;
    _delay_ms(10);
    return distance;
}

/*************************************************************/
/*************************************************************/
/*************************************************************/

/*************************************************************/
/**************************** Fans ***************************/
/*************************************************************/

void fanSetup(){
    DDRD |= (1<<RUDDER_FAN_PIN) | (1<<LIFT_FAN_PIN);
}

void timer0Setup(){
    TCCR0A = (1<<WGM00) | (1<<WGM01) | (1 << COM0A1) | (1<<COM0B1);
    TCCR0B = (1<<CS00);
}

void setThrustFanSpeed(uint8_t speed){
    OCR0A = speed;
}

void setLiftFanSpeed(uint8_t speed){
  OCR0B = speed;
}

/*************************************************************/
/*************************************************************/
/*************************************************************/

/*************************************************************/
/**************************** IRS ****************************/
/*************************************************************/

void IRSetup(){
    DDRC &= ~(1 << IR_PIN);
    ADMUX = 0;
    ADCSRA = ( 1 << ADEN) | ( 1 << ADPS2) | ( 1 << ADPS1) | ( 1 << ADPS0);
}

uint16_t readADC() {
    ADCSRA |= (1 << ADSC);  
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

float calculateDistanceIR(uint16_t adcValue) {
    float voltage = ((float)adcValue / 1024.0) * VREF;
    return 29.988*pow(voltage,-1.173);
}

/*************************************************************/
/*************************************************************/
/*************************************************************/

/*************************************************************/
/**************************** IMU ****************************/
/*************************************************************/

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

bool checkMPU6050Connection() {
    uint8_t id = 0;
    I2CStart();
    if (I2CWrite((IMU_ADDRESS << 1) | I2C_WRITE)) return false;
    if (I2CWrite(WHO_AM_I)) return false;
    I2CStart();
    if (I2CWrite((IMU_ADDRESS << 1) | I2C_READ)) return false;
    id = I2CReadNACK();
    I2CStop();
    return (id == 0x68);
}

void MPU6050Setup(){
  I2CStart();
  I2CWrite(IMU_ADDRESS << 1);
  I2CWrite(POWER_MAN_ADDRESS);
  I2CWrite(SLEEP_MODE_ADDRESS);
  I2CStop();

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

void calibrateMPU6050() {
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

    uint32_t current_time = myMicros();
    dt = (current_time - previous_time) / 1000000.0;
    previous_time = current_time;


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
/*************************************************************/
/*************************************************************/
/*************************************************************/
