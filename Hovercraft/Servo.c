// Definitions for the Servo
#define SERVO_PIN PB1  
#define MIN_PULSE 1200  // Min pulse of this servo is 600us, but each tick of our timer is 0.5us
#define MAX_PULSE 4800  // Max pulse of this servo is 2400us, but each tick of our timer is 0.5us

/*************************************************************/
/******************* Start Servo Functions *******************/
/*************************************************************/
/*
 Timer 1 will be used for the Servo
 Start by setting the timer to non inverting mode (low before match anf high after match (COM1A1)) and fast pwm mode (goes back to 0 when reaching the top (WGM11)).
 We then set the rest WGM13 and WGM12 for Fast PWM mode, and CS11 for prescaler of 8, this makes ticks of 0.5us.
 Then we set ICR1 to 40000, the servo is 50mHz (20ms), and since each tick is 0.5us, we need 40000 ticks.
*/
void timer1Setup() {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 40000;
}

void servoSetup() {
    DDRB |= (1 << SERVO_PIN);
    timer1Setup(); //Set timer 1 to control servo with PWM
    setPos(90); // Set servos to neutral position (90 degrees)
    _delay_ms(500); // Delay for adjustement in the position of servo
    TCCR1A &= ~(1 << COM1A1); // Clear COM1A1 to disable servo
}

/*
  Basically converting the angle to a pWM, so if its less than 0 degrees, its gonna be the min value of PWM
  If its more than 180 degrees, its gonna be the max value of PWM
  Anything else and we use a formula to convert the angle to PWM
*/
int angleToPulse(int angle) {
    if (angle <= 0) return MIN_PULSE;
    if (angle >= 180) return MAX_PULSE;
    return MIN_PULSE + ((MAX_PULSE - MIN_PULSE) * (long)angle) / 180;
}

// Just set the position depending on an angle, we call the angleToPulse function (OCR1A is the compare value register)
void setPos(int angle) {
    OCR1A = angleToPulse(angle);
}

// Function to set the Servo angle and LED L
void setServoWithYaw(){
// Control servo position based on yaw
    if (yaw > 81) {
        setPos(171);
        PORTB |= (1 << LED_YAW_PIN); // Turn on LED if yaw exceeds ±81 degrees
    }
    else if (yaw < -81) {
        setPos(9);
        PORTB |= (1 << LED_YAW_PIN); // Turn on LED if yaw exceeds ±81 degrees
    }
    else{
      setPos(90-yaw);
      PORTB &= ~(1 << LED_YAW_PIN); // Turn off LED if yaw is within ±81 degrees
    }      
}

/*************************************************************/
/******************** End Servo Functions ********************/
/*************************************************************/
