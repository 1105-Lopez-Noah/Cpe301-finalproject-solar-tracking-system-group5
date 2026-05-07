//Group 5
//Noah Lopez
//Christopher Thrasher
//Ryan Byorum
//Elliot Locker

#include <Servo.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//servos
Servo horizontal;
Servo vertical;

//States
enum State { OFF, IDLE, ACTIVE, ERROR };
volatile State currentState = OFF;
volatile bool startPressed = false;

//servo values
int servohori = 90;
int servovert = 90;

const int servohoriLimitHigh = 120;
const int servohoriLimitLow = 60;

const int servovertLimitHigh = 120;
const int servovertLimitLow = 60;

// tolerance
const int tol = 100;

//timing
unsigned long previousMillis = 0;
const unsigned long interval = 60000; // 1 minute logging


//UART FUNCTIONS 
void UART_init() {
  unsigned int ubrr = 103; // 9600 baud for 16MHz

  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;

  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_sendChar(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void UART_sendString(const char* str) {
  while (*str) {
    UART_sendChar(*str++);
  }
}

void UART_sendNumber(int num) {
  char buffer[10];
  itoa(num, buffer, 10);
  UART_sendString(buffer);
}

// ADC FUNCTIONs 
void ADC_init() {
  ADMUX = (1 << REFS0);

  ADCSRA = (1 << ADEN) |
           (1 << ADPS2) |
           (1 << ADPS1) |
           (1 << ADPS0);
}

uint16_t ADC_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | channel;

  ADCSRA |= (1 << ADSC);

  while (ADCSRA & (1 << ADSC));

  return ADC;
}

// ISR 
void startButtonISR() {
  startPressed = true;
}

//  LED UPDATE 
void updateLEDs() {

  // Turn all OFF first
  PORTA &= ~(1 << PA0);
  PORTA &= ~(1 << PA1);
  PORTA &= ~(1 << PA2);
  PORTA &= ~(1 << PA3);

  // OFF = BLUE
  if (currentState == OFF) {
    PORTA |= (1 << PA3);
  }

  // IDLE = YELLOW
  else if (currentState == IDLE) {
    PORTA |= (1 << PA1);
  }

  // ACTIVE = GREEN
  else if (currentState == ACTIVE) {
    PORTA |= (1 << PA2);
  }

  // ERROR = RED
  else if (currentState == ERROR) {
    PORTA |= (1 << PA0);
  }
}

//  PRINT STATE 
void printState() {

  if (currentState == OFF)
    UART_sendString("OFF");

  else if (currentState == IDLE)
    UART_sendString("IDLE");

  else if (currentState == ACTIVE)
    UART_sendString("ACTIVE");

  else if (currentState == ERROR)
    UART_sendString("ERROR");
}

//  SETUP 
void setup() {

  cli();

  UART_init();
  ADC_init();

  // LED PINS 
  // PA0 = RED
  // PA1 = YELLOW
  // PA2 = GREEN
  // PA3 = BLUE

  DDRA |= (1 << PA0);
  DDRA |= (1 << PA1);
  DDRA |= (1 << PA2);
  DDRA |= (1 << PA3);

  //  BUTTONS 
  // PD3 = START BUTTON
  // PG5 = OFF BUTTON
  // PE3 = RESET BUTTON

  DDRD &= ~(1 << PD3);
  PORTD |= (1 << PD3);

  DDRG &= ~(1 << PG5);
  PORTG |= (1 << PG5);

  DDRE &= ~(1 << PE3);
  PORTE |= (1 << PE3);

  //  SERVOS 
  horizontal.attach(2);
  vertical.attach(13);

  horizontal.write(servohori);
  vertical.write(servovert);

  // INTERRUPT 
  attachInterrupt(digitalPinToInterrupt(3), startButtonISR, FALLING);

  sei();

  updateLEDs();

  UART_sendString("System Started\r\n");
}

// LOOP 
void loop() {

  // START BUTTON 
  if (startPressed && currentState == OFF) {

    currentState = IDLE;

    UART_sendString("START BUTTON\r\n");

    startPressed = false;
  }

  // OFF BUTTON 
  if (!(PING & (1 << PG5))) {

    currentState = OFF;

    UART_sendString("OFF BUTTON\r\n");
  }

  // RESET BUTTON 
  if (!(PINE & (1 << PE3))) {

    if (currentState == ERROR) {
      currentState = IDLE;
    }

    servohori = 90;
    servovert = 90;

    horizontal.write(servohori);
    vertical.write(servovert);

    UART_sendString("RESET BUTTON\r\n");
  }

  updateLEDs();

  // OFF STATE 
  if (currentState == OFF) {

    horizontal.write(90);
    vertical.write(90);

    return;
  }

  // ADC READS 
  int lt = ADC_read(0);
  int ld = ADC_read(1);
  int rd = ADC_read(2);
  int rt = ADC_read(3);

  //  ERROR CHECK 
  if ((lt < 5 && rt < 5 && ld < 5 && rd < 5) ||
      (lt > 1018 && rt > 1018 && ld > 1018 && rd > 1018)) {

    currentState = ERROR;

    updateLEDs();

    UART_sendString("ERROR STATE\r\n");

    return;
  }

  // AVERAGES 
  int avt = (lt + rt) / 2;
  int avd = (ld + rd) / 2;
  int avl = (lt + ld) / 2;
  int avr = (rt + rd) / 2;

  int dvert = avt - avd;
  int dhoriz = avl - avr;

  //  STATE LOGIC 
  if (abs(dvert) <= tol && abs(dhoriz) <= tol) {

    currentState = IDLE;
  }

  else {

    currentState = ACTIVE;

    // VERTICAL 
    if (abs(dvert) > tol) {

      if (avt > avd)
        servovert++;

      else
        servovert--;

      if (servovert > servovertLimitHigh)
        servovert = servovertLimitHigh;

      if (servovert < servovertLimitLow)
        servovert = servovertLimitLow;

      vertical.write(servovert);
    }

    // HORIZONTAL 
    if (abs(dhoriz) > tol) {

      if (avl > avr)
        servohori--;

      else
        servohori++;

      if (servohori > servohoriLimitHigh)
        servohori = servohoriLimitHigh;

      if (servohori < servohoriLimitLow)
        servohori = servohoriLimitLow;

      horizontal.write(servohori);
    }
  }

  updateLEDs();

  //  1 MINUTE LOGGING 
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;

    UART_sendString("STATE: ");
    printState();

    UART_sendString(" | LT: ");
    UART_sendNumber(lt);

    UART_sendString(" | RT: ");
    UART_sendNumber(rt);

    UART_sendString(" | LD: ");
    UART_sendNumber(ld);

    UART_sendString(" | RD: ");
    UART_sendNumber(rd);

    UART_sendString(" | ServoH: ");
    UART_sendNumber(servohori);

    UART_sendString(" | ServoV: ");
    UART_sendNumber(servovert);

    UART_sendString("\r\n");
  }
}
