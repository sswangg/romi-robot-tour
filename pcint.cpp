#include "pcint.h"

//define an ISR function pointer; ISRs are of type void fxn(void)
typedef void(*PCISR)(void); 

//array to hold all of the indivicual ISRs; initialize to null
PCISR pcISR[] = {0, 0, 0, 0, 0, 0, 0, 0}; 

//store the previous state -- needed to find which have changed
static volatile uint8_t lastB = PINB; //these are likely all 0 at the start

void attachPCInt(uint8_t pcInt, void (*pcisr)(void))
{
    cli();
    PCICR = (1 << PCIE0);   //enable PC interrupts (not a problem to set repeatedly)

    PCMSK0 |= (1 << pcInt); //enable PCInt on the specific pin
    pcISR[pcInt] = pcisr;   //register the ISR

    PCIFR = (1 << PCIF0);   //clear any pending interrupt before we get started

    //make sure we start with the current state, but don't clobber other pins
    lastB &= ~(1 << pcInt);             //clear the affected pin in lastB
    lastB |= PINB & (1 << pcInt);      //then set it to the current state
    sei();
}

ISR(PCINT0_vect)
{
    //read the current state of the PCINT pins
    volatile uint8_t pinsB = PINB;

    //find the pins that have changed
    volatile uint8_t deltaB = pinsB ^ lastB;                

    //we're only interested in pins that have changed AND are set for interrupts
    volatile uint8_t maskedDeltaB = deltaB & PCMSK0;        
    
    //each of these checks if each ISR should run
    if((maskedDeltaB & (1 << PCINT0))) {pcISR[PCINT0]();}   
    if((maskedDeltaB & (1 << PCINT1))) {pcISR[PCINT1]();}
    if((maskedDeltaB & (1 << PCINT2))) {pcISR[PCINT2]();}
    if((maskedDeltaB & (1 << PCINT3))) {pcISR[PCINT3]();}
    if((maskedDeltaB & (1 << PCINT4))) {pcISR[PCINT4]();}
    if((maskedDeltaB & (1 << PCINT5))) {pcISR[PCINT5]();}
    if((maskedDeltaB & (1 << PCINT6))) {pcISR[PCINT6]();}
    if((maskedDeltaB & (1 << PCINT7))) {pcISR[PCINT7]();}

    //update last pin states
    lastB = pinsB;                                          
}

uint8_t digitalPinToPCInterrupt(uint8_t pin)
{
  uint8_t pcInt = NOT_AN_INTERRUPT;

#if defined(__AVR_ATmega32U4__)
  switch(pin)
  {
    case 17: pcInt = PCINT0; break;
    case 15: pcInt = PCINT1; break;
    case 16: pcInt = PCINT2; break;
    case 14: pcInt = PCINT3; break;
    case  8: pcInt = PCINT4; break;
    case  9: pcInt = PCINT5; break;
    case 10: pcInt = PCINT6; break;
    case 11: pcInt = PCINT7; break;
    default: break;
  }
#endif

  return pcInt;
}