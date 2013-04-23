#include < avr/io.h >
#include < avr/interrupt.h >
#include < Arduino.h >

// Shift Register pin
int DS = 13;
int INCLK = 11;
int OUTCLK = 10;

// RPM counter
// Connect hall effect sensor to pin 2 (interrupt 0)
volatile byte rotationCount = 0;
unsigned int rps = 0;
unsigned long oldTime = 0;

// Interrupt pin
volatile unsigned long count = 0;
volatile unsigned long interruptCount = 25;
volatile unsigned long ledArrayIndex = 0;

// LED Array

#define TIMER_FREQUENCY 15625
#define LED_COLUMNS 20
#define REFRESH_COUNT LED_COLUMNS

// each element of this array is a bitfield where each bit represents an
// LED where the LSB is the top (highest) LED on the sphere.
unsigned long ledArray[LED_COLUMNS] = {0x00001, 0x00002, 0x00004, 0x00008, 
                                       0x00010, 0x00020, 0x00040, 0x00080, 
                                       0x00100, 0x00200, 0x00400, 0x00800,
                                       0x01000, 0x02000, 0x04000, 0x08000,
                                       0x10000, 0x20000, 0x40000, 0x80000};

/***************************************************************
* Timer class
* Description: For clock division, enable timer interrupt type
****************************************************************/

class CTimer {
  int clock_divider, enableCMPA, enableCMPB, enableOVF;
    
  // to set clock:
  // flush interrupts (flushInterrupts)
  // set initial cycle count (set_time)
  // set divider (set_divider)
  // enable interrupt type (compare or overflow)
  // debug: printConfig -- dumps all registers
  
  public:
    void set_time(int); // set initial cycle count
                        // Refers to Timer 1, which is 16-bit 
    int get_time();     // get preloaded time
    void set_divider(int);    // divider definitions -- step wise; any other values will be sent to the next highest step
                              // a = 1, clk/1
                              // a = 2, clk/8
                              // a = 3, clk/6
                              // a = 4, clk/256
                              // a = 5, clk/1024
    int get_divider();  // get divider value
    void set_cmp(int);  // set interrupt cycle count
                        // CMP interrupt fires when the timer is equal to the CMP reference
    int get_cmp();      // get interrupt cycle count 
    void set_cmpA(int); // enable CMPA interrupt
                        // CMP is 16-bit
    int get_cmpA();     // get CMPA interrupt state
    void set_cmpB(int); // enable CMPB interrupt
    int get_cmpB();     // get CMPB interrupt state
    void set_ovf(int);  // enable OVF interrupt
                        // OVF interrupt fires when timer overflows and resets to 0 (when 16 registers are set)
    int get_ovf();      // get OVF interrupt state
    void printConfig();
    void flushInterrupts();
    
    CTimer(int user_a){
      TCNT1 = user_a;
    }
    
    CTimer(){
      TCNT1 = 0;
    }
};

/***************************************************************
* ISR class
* Description: Set interrupt and signal response type
****************************************************************/

class CISR {
  int enableIsr, intPin, setSignal;
  public:
    void set_Isr(int);  // enable Isr
    void set_IsrPin(int);  // set Isr pin
    void set_signal(int);  // set signal respone
    int get_enableIsr();  // get enable boolean
    int get_enableIsrPin();  // get Isr pin
    int get_signal();  // get signal response
};

ISR(TIMER1_COMPA_vect) {
  // Expect 16 MHz/x
  count++;
  if (count > interruptCount) {
    count = 0;
    updateLedArray();
  }  
}

void updateLedArray ()

/*++

Routine Description:

    This routine updates the LED array by writing out to the shift registers.

Arguments:

    None.
    
Return Value:

    None.

--*/

{
 
    digitalWrite(OUTCLK, LOW);        
    shiftOut(DS, INCLK, LSBFIRST, ledArray[ledArrayIndex]);
    shiftOut(DS, INCLK, LSBFIRST, ledArray[ledArrayIndex] >> 8);
    shiftOut(DS, INCLK, LSBFIRST, ledArray[ledArrayIndex] >> 16);
    digitalWrite(OUTCLK, HIGH);
    Serial.println(ledArray[ledArrayIndex], HEX);
    ledArrayIndex += 1;
    ledArrayIndex %= LED_COLUMNS;
    return;
}

void updateRotationSpeed () 

/*++

Routine Description:

    This routine updates the interrupt period based on the rotation speed.
    
Arguments:

    None.

Return Value:

    None.

--*/

{
    
    rotationCount += 1;
    if (rotationCount >= 5) {
        rps = rotationCount * 1000 / (millis() - oldTime);
        interruptCount = TIMER_FREQUENCY / (rps * REFRESH_COUNT);
        oldTime = millis();
        rotationCount = 0;
    }
    
    return;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Serial.println("Setting pins");
  pinMode(DS, OUTPUT);
  pinMode(OUTCLK, OUTPUT);
  pinMode(INCLK, OUTPUT);
  
  // Set up hall effect sensor interrupt. Write to pin 2 so no resisters need
  // to be attached to the sensor. http://playground.arduino.cc/Main/ReadingRPM
  digitalWrite(2, HIGH);
  attachInterrupt(0, updateRotationSpeed, RISING);
  //Serial.println("Setting TIMER");
  noInterrupts();
  
  CTimer timer;
  
  timer.set_time(0);
  timer.set_cmp(TCNT1/2);
  timer.flushInterrupts();
  timer.set_cmpA(1);
  timer.set_divider(5);
  timer.printConfig();
  
  /*
  // preloading timer
  TCNT1 = 255;
  Serial.print("TCNT1: ");
  Serial.println(TCNT1, HEX);
  
  // compare to a preloaded value
  OCR1A |= TCNT1/2;//65536;
  //Serial.print("OCR1A: ");
  //Serial.println(OCR1A, HEX);

  // Enable overflow and compare A interrupts
  TIMSK1 |= ( 1 << OCIE1A );
  Serial.print("TIMSK1: ");
  Serial.println(TIMSK1, HEX);
  
  // timer 2 modifications
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= ( 1 << CS12 );
  TCCR1B |= ( 0 << CS11 );
  TCCR1B |= ( 1 << CS10 );

  Serial.print("TCCR1B: ");
  Serial.println(TCCR1B, HEX);
  */
  // Enable interrupts  
  interrupts();
  //Serial.println("Finished init");
}

void loop() {
  // put your main code here, to run repeatedly: 
}

// set methods
void CTimer::flushInterrupts(){
  TCCR1B = 0;
}
void CTimer::set_time (int a) {
  // preload time
  TCNT1 = a;
}
void CTimer::set_cmp (int a) {
  // preload comparator
  OCR1A = a;
}
void CTimer::set_cmpA (int a){
  // enable compareA interrupt
  TIMSK1 |= ( 1 << OCIE1A );
  enableCMPA = a;
}
void CTimer::set_cmpB (int a){
  // enable compareB interrupt
  TIMSK1 |= ( 1 << OCIE1B );
  enableCMPB = a;
}
void CTimer::set_ovf (int a){
  // enable OVF interrupt
  TIMSK1 |= ( 1 << TOIE1 );
  enableOVF = a;
}
void CTimer::set_divider(int a){
  TCCR1B = a;
  clock_divider = a;
}

//get methods
int CTimer::get_time () {
  return TCNT1;
}
int CTimer::get_cmp () {
  return OCR1A;
}
int CTimer::get_cmpA (){
  return enableCMPA;
}
int CTimer::get_cmpB (){
  return enableCMPB;
}
int CTimer::get_ovf (){
  return enableOVF;
}
int CTimer::get_divider (){
  return clock_divider;
}
void CTimer::printConfig(){
  Serial.print("TCNT1: ");  //time
  Serial.println(TCNT1, HEX);
  Serial.print("OCR1A: ");  //compare interrupt value
  Serial.println(OCR1A, HEX);
  Serial.print("TIMSK1: ");  //interrupt enable registers
  Serial.println(TIMSK1, HEX);
  Serial.print("TCCR1B: ");  // divider value
  Serial.println(TCCR1B, HEX);
}

/***************
* ISR class routines
***************/
//set methods
void CISR::set_Isr(int a) {
  if (a == 2) {
    EIMSK |= (a << INT0 ); //pin 2
    intPin = 2;
  
  } else {
    EIMSK |= ( a << INT1 ); //pin 3
    intPin = 3;
  }
} // enable Isr

void CISR::set_signal(int a){
  EICRA = a;
  setSignal = a;
}  // set signal respone

//get methods
int CISR::get_enableIsr(){
  return enableIsr, intPin;
}  // get enable boolean

int CISR::get_signal(){
  return setSignal;
}  // get signal response
