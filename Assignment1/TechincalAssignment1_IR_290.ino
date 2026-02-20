// Read Pages 205-220 in The ATmega328P Datasheet for More Information about the 
// ADC Conversion and the Pins/Registers Used. 

/** Libraries **/
#include <avr/io.h>
#include <util/delay.h>

/** Pin Number Definitions **/
// The pin for the LED which brightness will vary on the distance and is called LED_DISTANCE_PIN, assigned to PB3.
// The pin for the infrared sensors which will read the voltage output is called ADC_PIN, assigned to PC0.
// The pin for the LED which brightness will flash when the distance is outside of the range is called BLINK_LED_PIN, assigned to PB5.
#define ADC_PIN PC0
#define LED_DISTANCE_PIN PB3 
#define BLINK_LED_PIN PB5

/** Constant Values **/
// PWM_MAX defines the maximum value for the PWM (8 bits), which is 255 (0 to 255).
#define PWM_MAX 255
const float VREF = 2.6;
uint16_t ms_count = 0;

/** Functions **/
// pinSetup() => Sets the pins to be either an input or output. Additionally, it sets the ADC input settings 
// and controls the ADC operations. Also, it makes changes to the timers used.
// readADC() => Reads the ADC value from the IR sensor (ADC0).
// calculateDistance(uint16_t adcValue) => Calculates the distance from the infrared sensor based on the ADC value, 
// basically converting the value into a voltage.
// changeBrightness(long distance) => Adjust the brightness of the LED depending on the distance given.
// ledBlink(long distance) => Adjust the brightness of the LED so it flashes when the obstacle is outside the required ranged.
void pinSetup();
uint16_t readADC();
float calculateDistance(uint16_t adcValue);
void changeBrightness(long distance);
void ledBlink(long distance);

/** Main Function **/
// This is the main function that will setup the pins (pinSetup()) and run an infinite while loop
// Inside the loop, it will call the functions necessary to read the ADC value, calculate the distance 
// with the sensor (by converting first the ADC value to a voltage) and adjust the brightness of the LED.
// Additionally it will have the blinking LED while the distance is out of range.
int main(){
    pinSetup();

    while (1) {
        uint16_t adcValue = readADC(); // 16 bit unsigned int for the brightness
        printADC(adcValue);
        
        float distance = calculateDistance(adcValue);
        printDistance(distance);
        
        changeBrightness(distance);
        ledBlink(distance);
        _delay_ms(50);
    }
}

// Interrupt Service Routine, when an interrupt from timer0 is called (the timer hits OCR0A), the ISR is called and it increments ms_count by one
ISR(TIMER0_COMPA_vect){
  ms_count++;
}

/** Pin Setup **/
// We set LED_DISTANCE_PIN as an output and ADC_PIN as an analog input.
// The ADC Multiplexer Select (ADMUX) is used to select the input pin for the ADC. 
// Set to use the AREF for ADC conversions, as the internal reference voltage (Vref) is off.
// The ADC CTRL and Status Register (ADCSRA) is used to control the operations. Writing the ADEN bit to one enables the ADC, and setting 
// the ADC prescaler select bits APDS2, APDS1 and APDS0 to one, configures the prescaler to 128 for a stable ADC clock between 50kHz and 200kHz.  
// Since the port for the distance LED is PB3 (OC2A), we use Timer2 (TCCR2A)
// WGM20 and WGM21 are set for fast PWM mode and COM2A0 and COM2A1 are set for inverting mode
// We set CS21 for a prescaler of 8. This is done to have the proper frequency for the distance LED. 
// For dimming LED, the frequency should be above 2kHz "https://www.ti.com/document-viewer/lit/html/SSZTAY6#:~:text=In%20short%2C%20to%20use%20PWM,have%20a%20PWM%20dimming%20input"
void pinSetup() {
  uartTX_init();
  DDRB |= (1 << LED_DISTANCE_PIN);
  DDRC &= ~(1 << ADC_PIN);
  ADMUX = 0;
  ADCSRA = ( 1 << ADEN) | ( 1 << ADPS2) | ( 1 << ADPS1) | ( 1 << ADPS0);
  TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2A1) | (1 << COM2A0);  
  TCCR2B = (1 << CS21);
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);
  sei();
}

/** Read ADC (Analog) Value **/ 
// Setting the ADSC (ADC Start Conversion) bit in ADCSRA will start the ADC conversion.
// Afterwards, we need to wait for the conversion to complete by checking if ADSC is still 
// set (it is cleared when the conversion finishes).
// Finally we return the 10-bit ADC result from the ADC Data Register.
uint16_t readADC() {
    ADCSRA |= (1 << ADSC);  
    while (ADCSRA & (1 << ADSC)); 
    return ADC;
}

/** Calculate Distance IR **/
// Convert the ADC value to voltage using the formula: (adcValue / 1024) * reference_voltage (1.1 V).
// ADC value is 10 bits (0-1023), and we are using 2.6 V as the reference voltage.
// Used SharpIR formula to find the distance based on the voltage.
float calculateDistance(uint16_t adcValue) {
    float voltage = ((float)adcValue / 1024.0) * VREF;
    return 29.988*pow(voltage,-1.173);
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
    _delay_us(100);
}

void ledBlink(long distance){
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

void printDistance(long distance){
  char arrayDistance[10];
  dtostrf(distance, 6, 1, arrayDistance);
  UART_TransmitArray("Distance: ");
  UART_TransmitArray((uint8_t*)arrayDistance); //cast a pointer to char to pointer to uint8_t which the function expects. Each character in the string is converted to a byte in the range of 0 to 255 which corresponds to an ASCII value. 
  UART_TransmitArray("\n");
}

void printADC(uint16_t adcValue){
  char arrayADC[10];
  dtostrf(adcValue, 4,0, arrayADC);
  UART_TransmitArray("ADC: ");
  UART_TransmitArray((uint8_t*)arrayADC);
  UART_TransmitArray("\n");
}
