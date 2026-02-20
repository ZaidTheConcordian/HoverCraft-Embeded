//define 16MHz CPU speed from external crystal on Nano
#define F_CPU 16000000UL
//define baudrate
#define BAUD 9600UL
//define UBRR from baudrate based on equations in data sheet and speed of 16
#define UBRR (F_CPU/(16 * BAUD)-1)




/*************************************************************/
/******************** Start UART Functions *******************/
/*************************************************************/

//initialization function
void uartTX_init() {
    //set baud rate
    UBRR0H = (uint8_t)(UBRR >> 8);
    UBRR0L = (uint8_t)(UBRR);

    //enable tx
    UCSR0B = (1 << TXEN0);

    //set asynchronous, no parity, 1 stop bit, 8 data bits
    //datasheet table 19-7 UCSZn Bits settings
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
}


//Uart transmit function 

void UART_Transmit(uint8_t data){

//wait for transmit buffer to be empty - check UDREn flag (data register empty) in UCSRnA register
while (!(UCSR0A & (1<<UDRE0)));

//load into register for transmission
UDR0 = data;

}

//this only takes a string so data will need to be converted to string to send to this function
//data is dealt with byte by byte so the type and sign of the data does not matter
//Hence, an unsigned 8 bits value with a range of 0-255 is best suited for this job
void UART_TransmitArray(uint8_t * msg){
 
  int i = 0;

  while (msg[i] != '\0'){

    UART_Transmit(msg[i]);
    i++;

  }
}

/*************************************************************/
/******************** End UART Functions *********************/
/*************************************************************/
