/**all libraries**/
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include <compat/twi.h>


/** Main Function **/

/*************************************************************/
/******************** End Main Functions *********************/
/*************************************************************/

















/*******************************************************************************************************************************************************************************************************************************/
/************************************ This section is reserved to place all the code of all the different components, it aslo includes the deinitions and the variables for each component *************************************/
/*******************************************************************************************************************************************************************************************************************************/

/*************************************************************/
/************************** Servos ***************************/
/*************************************************************/
// Definitions for the Servo
#define USS_SERVO_PIN PB1 // Will be controlled with OCR1A
#define RUDDER_SERVO_PIN PB2 // Will be controlled with OCR1B
#define MIN_PULSE 1000  // Min pulse of this servo is 600us, but each tick of our timer is 0.5us
#define MAX_PULSE 5000  // Max pulse of this servo is 2400us, but each tick of our timer is 0.5us

void timer1Setup() {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 40000;
}

void servoSetup() {
    DDRB |= (1 << RUDDER_SERVO_PIN) | (1 << USS_SERVO_PIN);
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
    Serial.print("I have set the pulse to"); Serial.println(pulse);
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

#define TRIGGER_PIN PB5
#define ECHO_PIN PD3
volatile uint32_t us_count = 0;

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
    while (PIND & (1 << ECHO_PIN)){
      if((myMicros() - startTime) > 25000){break;}
    }
    uint32_t endTime = myMicros();
    duration = endTime - startTime;
    distance = (duration * 0.0343)/2;
    _delay_ms(10);
    return distance;
}

void navigation(long distance_front){
  long distance_left = 0, distance_right = 0;
  if(distance_front < 55){
    // (2) Rotate servo ccw
    setPosUSS(170); // Rotate servo ccw so the US sensor looks in the right direction
    _delay_ms(500); // Wait for the servo to reach the position

    distance_left = calculateDistanceUS();
    Serial.print("Distance left"); Serial.println(distance_left);
    setPosUSS(10); // Rotate servo cw so the US sensor looks in the left direction
    _delay_ms(500); // Wait for the servo to reach the position

    distance_right = calculateDistanceUS();
    Serial.print("Distance right"); Serial.println(distance_right);
    if(distance_left > distance_right){
      Serial.println("turning left");
    }
    else if(distance_left < distance_right){
      Serial.println("turning right");
    }
    // Rotate servo back to its neutral position
    setPosUSS(90);
    _delay_ms(500); // Wait for the servo to reach the position
  }
}


// MAIN
int main() {
    timer1Setup();
    timer2Setup();
    servoSetup();
    USSSetup();
    Serial.begin(9600);
    setPosUSS(90);
    sei();
    while (1) {
      _delay_ms(1000);
      navigation(43);
      _delay_ms(100);
    }

  return 0;
}




































