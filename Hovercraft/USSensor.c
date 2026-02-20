
/** Pin Number Definitions **/
// The pin for the LED which brightness will vary on the distance is called LED_DISTANCE_PIN, assigned to PB3
// The trigger pin for the ultrasonic sensor is called TRIGGER_PIN, assigned to PB5.
// The echo pin for the ultrasonic sensor is called ECHO_PIN, assigned to PD3
// The blinking LED is sharing the PB5 pin with TRIGGER_PIN
#define LED_DISTANCE_PIN PB3
#define TRIGGER_PIN PB5
#define ECHO_PIN PD3
#define BLINK_LED_PIN PB5

/** Constant Values **/
// PWM_MAX defines the maximum value for the PWM (8 bits), which is 255 (0 to 255)
#define PWM_MAX 255
uint16_t ms_count = 0;

/** Functions **/
// pinSetup() => Sets the pins to be either an input or output. Additionally, it makes changes to the timers used.
// changeBrightness(long distance) => Adjust the brightness of the LED depending on the distance given.
// calculateDistanceUS() => Calculates the distance from the ultrasonic sensor.
// ledBlink -> just blinks the LED... 💯
void usPinSetup();
//void changeBrightness(long distance);
long calculateDistanceUS();
//void ledBlink(long distance);

/*********************************************************/
/***********Start HC-SR04 US SENSOR Functions ************/
/*********************************************************/ 

// Interrupt Service Routine, when an interrupt from timer0 is called (the timer hits OCR0A), the ISR is called and it increments ms_count by one
ISR(TIMER0_COMPA_vect){
  ms_count++;
}

/** Pin Setup **/
// We set the TRIGGER_PIN and LED_DISTANCE_PIN as outputs.
// We set ECHO_PIN as input
// Since the port for the distance LED is PB3 (OC2A), we use Timer2 (TCCR2A)
// WGM20 and WGM21 are set for fast PWM mode and COM2A0 and COM2A1 are set for inverting mode
// We set CS21 for a prescaler of 8. This is done to have the proper frequency for the distance LED.
// For dimming LED, the frequency should be above 2kHz "https://www.ti.com/document-viewer/lit/html/SSZTAY6#:~:text=In%20short%2C%20to%20use%20PWM,have%20a%20PWM%20dimming%20input"
// A prescale of 8 gives a frequency of around 8kHz, no prescaler would be too fast and a lower one would be too slow.
// We are also going to be using timer 1, we set it as 0 and set the prescale at 8 (0.5us per cycle)
// Timer0 will be used for the blinking LED
// WGM01 is to set it to CTC mode, basically set it to 0 when it reaches the OCR0A value
// CS01 and CS00 are set for a prescaler of 64, this emans everytick of the timer will be 4us
// Then we set OCR0A at 249, this is done because 250 (0-249) * 4us is 1ms, meaning everytime timer0 reaches 249, it took 1ms
// Then we set OCIE0A so that we can call an interrupt when reaching the 249 (every 1ms)
void usPinSetup(){
  DDRB |= (1 << TRIGGER_PIN) | (1<< LED_DISTANCE_PIN);
  DDRD &= ~(1 << ECHO_PIN);
  TCCR2A |= (1 << WGM20) | (1 << WGM21) | (1<<COM2A0) | (1<<COM2A1);    
  TCCR2B |= (1 << CS21);  
  TCCR1A = 0;
  TCCR1B = (1<<CS11);
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);
  sei();
  return;
  // Everything from 1 to 7 should be in the PDR
}

/** Change Brightness **/
// Taking in the distance calculated, if distance is >= 46cm, then distance LED -> 10% ~ (26/255)
// If the distance is <= 12cm, then distance LED -> 100% = (255/255)
// Otherwise, we calculate the percentage of brightness and put it on a scale of 255
// The timer will count from 0 to 255, with inverting mode, it will set the distance LED pin as HIGH while its counting from 0 to bright,
// then it will set the distance LED pin as LOW while counting from bright to 255, it knows to first count to bright because of OCR2A.
// With fast PWM mode, when it reaches 255, the timer will get set back to 0.
void changeBrightness(long distance) {
    uint8_t bright = 0; // 8 bit unsigned int for the brightness
    if (distance >= 46) { bright = 26; }
    else if (distance <= 12) { bright = PWM_MAX; }
    else { bright = PWM_MAX - (distance - 12.0) * (229.0/34.0); }
    OCR2A = bright;
}

/** Calculate Distance US **/
// The first part of the function is based off the datasheet of the HC-SR04.
// You first set the trigger pin to 0 for 2us, then you set it to 1 for 10us.
// During that time the sensor will send 8 cycles burst that travel at the speed of sound.
// We wait for the bursts to come back to the sensor so that echo becomes 1.
// Then we calculate how long echo is 1 for, we then multiply the time by the speed of sound and divide it by two (round trip, we want one way)
// The maximum distance is 4m (23200us)
// Delay of 10ms at the end (look at USS datasheet)
// For the timer, we basically just set it to 0, then we wait until we have a HIGH from echo (max waiting time of 38000us / 0.5 (0.5us per cycle))
// Then when the ECHO pin is 1, we set the timer to 0 and then loop while the pin is 1
// At the end we set the time as the value counted by the timer and then we use the formula given for the sensor (*0.5 because 0.5us for one cycle)
long calculateDistanceUS() {
    unsigned long duration = 0;
    long distance = 0;
    PORTB &= ~(1 << TRIGGER_PIN);
    _delay_us(2);
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);
    TCNT1 = 0;
    while (!(PIND & (1 << ECHO_PIN))) {
        if (TCNT1 > 72000) return -1;
    }
    TCNT1 = 0;
    while (PIND & (1 << ECHO_PIN)) {
        if (TCNT1 > 46400) break;
    }
    duration = TCNT1;
    distance = (duration * 0.0343) / 4;
    printDuration(duration *0.0005);
    _delay_ms(10);
    return distance;
}



// The LedBlink function is made to run in parallel with the rest of the code
// It frist checks if the distance is out of range, then it take the previously save time value and finds the interval between the current time and the previous one.
// If that interval is more than 250ms, then we negate the LED (if on then off, if off then on), then we save the current time ad the previous time
// If its in range, then we keep the LED on.
/**void ledBlink(long distance){
  if(distance < 12 || distance > 46){
    if(ms_count >= 250){
      PORTB ^= (1 << BLINK_LED_PIN);
      ms_count = 0;
    }
  }
  else{
    PORTB &= ~(1 << BLINK_LED_PIN);
    ms_count = 0;
  }
}
**/
void usPrintDistance(long distance){
  char arrayDistance[10];
  dtostrf(distance, 6, 1, arrayDistance);
  UART_TransmitArray("Distance: ");
  UART_TransmitArray((uint8_t*)arrayDistance); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
}

void printDuration(long duration){
  char arrayDuration[10];
  dtostrf(duration, 6, 1, arrayDuration);
  UART_TransmitArray("Duration: ");
  UART_TransmitArray((uint8_t*)arrayDuration); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
}

/*********************************************************/
/*************End HC-SR04 US SENSOR Functions ************/
/*********************************************************/
