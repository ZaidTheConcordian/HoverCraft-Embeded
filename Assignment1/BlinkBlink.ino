#include <avr/io.h>
#include <avr/interrupt.h>

//Using the following calculator in the normal mode
//https://www.ee-diary.com/p/atmega-microcontroller-timercounter.html#
//where The Cpu Fred is 8MH
//Time delay = 0.0325s,32500us
//Prescaler = 1024
//We get TCNT0 = 2
//yo my blinkblink has freq of two so he show up two times a sec for his protection tax
//0.25/0.0325 = 7.6923 Bro it is like 8


#define BLINKBLINKMF PB5 // THIS IS BLINK BLINK TOWN



void pinSetup();
void blinkblink(long distance);
void checkdablink();
//For the Mode0 we need to set all the WGM0 bits to 0 (1,2,0) because all zero we dont need to actually play with them
//for the prescaler we need to set the CS00 and CS02 to 1 and CS01 to 0

volatile int overflow_count;
volatile int light_flag;

int main() {
  pinSetup();
  sei();
  long distance = 50;
  overflow_count =0 ; // need to 8 before changing period again
  light_flag = 0; // check if light is on
  
  while (1) {
    
    checkdablink();
    blinkblink(distance);

  }
}


void pinSetup(){
  PORTB |= (1 <<BLINKBLINKMF); 
  DDRB |= (1<<BLINKBLINKMF);  //Where the BEGGERS BEG FOR BLINK BLINK MONEY
  TCNT0 = 2;  //as Calculted need 2 for a time delay of 0.0325 sec not as the sign but as seconds // we start at 2 and count the full 254 cycles to overflow
  TCCR0B |= (0<<CS01) | (1<<CS00) |(1<<CS02);//Sniffing that good prescaler thing 1024 times
  TIMSK0 |= (1 << TOIE0); // Enable Timer0 overflow interrupt
  return;
  
}




void checkdablink(){
  if (overflow_count>=8){ // see if that period has passed
    PORTB ^= (1 <<BLINKBLINKMF); //toggle that blinkblink
    overflow_count = 0; // reseting the period
  }

}


void blinkblink(long distance){
  if (distance > 46) {
    if(light_flag == 0){//if light is off need to turn on
      TCNT0 = 2; //reseting counter
      TIMSK0 |= (1<<TOIE0);  //enabling the interupt
      light_flag = 1; //Telling that blinkblink men that he is bright
      PORTB |= (1 <<BLINKBLINKMF); 
    }
  }
  else if (distance < 12) {
    if(light_flag == 0){ //if light is off need to turn on
      TCNT0 = 2; //reseting counter
      TIMSK0 |= (1<<TOIE0);  //  enabling the interupt
      light_flag = 1;//Telling that blinkblink men that he is bright
      PORTB |= (1 <<BLINKBLINKMF); 
      
    }
  }
  else {
    if(light_flag == 1) // if light is on need to trun of
    TIMSK0 &= ~(1<<TOIE0);//Set it that thing to set interpt to 0
    light_flag = 0; //setting the flag to 0
    PORTB &= ~(1 <<BLINKBLINKMF); 
  }
  

}
ISR(TIMER0_OVF_vect){
  overflow_count++;
}
