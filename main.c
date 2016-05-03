
// MassMind.org BOB P.I.D. Servo controller
// Board, kit, instructions at:
// http://techref.massmind.org/techref/io/SERVO/BOBPID.htm
// Using MPLAB IDE v8.84 C18 v3.31
// Starting from:
// http://www.hobbytronics.co.uk/rotary-encoder-tutorial
// Modified by James Newton at MassMind org
// [X] Modify to use RC3, P1D as the output.
// [X] Modify to use RA0 / RA5
// [X] Test / debug at low speed (throttle encoder)
// [X] Modify to interrupt on RA0 / RA5
// [X] Increase resolution of encoder (A&B, Rise&Fall)
// [X] Add PID code
// [X] Add serial user interface
// [X] Add step direction input
// [ ] Test / debug (run to position at speed)
// [X] Add PID term non-volatile storage
// [X] Add Parameter for polarity of direction
// [ ] Add LCD user interface?
//     Problem with LCD interface: We don't control X DIR input
//      which is also used by LCD as a data line.
// [ ] use enable from step direction input.
// [ ] Add demo mode
// [ ] Add status reply to "?" command
// [ ] Add autotune mode with status output

#define MOTOR_ENABLE LATCbits.LATC7  	//Enables Motor Driver
#define MOTOR_DIRECTION LATAbits.LATA4	//Sets Motor Direction
#define MOTOR_SPEED LATCbits.LATC2  	//Set Motor PWM output (must be P1x)

#define ENCODER_A PORTAbits.RA0        // Encoder A Pin (must be IOC)
#define ENCODER_B PORTAbits.RA5        // Encoder B Pin (must be IOC)
// from ZSTEP on Z-Y LED pins via JP5 and
// (slant) jumper from ASTEP to ZSTEP over RP2. A AXIS not connected

#define INPUT_STEP PORTAbits.RA1		//Step input (must be INT1)
#define INPUT_DIRECTION PORTCbits.RC0	//which direction do we step?
#define INPUT_ENABLE PORTBbits.RB6		//
#define INPUT_STEP_SIZE 2				//how much do we step?
#define ERROR_STATUS LATAbits.LATA2		//A2 jumper to Xerr/LED on J12
// J12 pin 3 anode, pin 2 cathode bridge to pin 1 330R to GND J11 pin 5
// C5 is spindle/LED or LCD /E
#define ERROR_ON 1
#define ERROR_OFF 0

#define	PID_KP_START 20
#define	PID_KI_START 0.2
#define	PID_KD_START 120

#define SAMPLE_TIME .005	//Seconds
#define SAMPLE_FREQ (1/SAMPLE_TIME) //Hz

#define PWM_MAX 253
//#define PWM_MIN 0

#define OUT_RES_DIV 16	//Amount to divide the output by for PWM.
//Too low a value her make the units for the PID impossibly small
//and makes resolution two high. Too high reduces resolution.

#define OSC_FREQ 16000000L	
//Max 16 MHz in standard modes.
//Up to 64M available with extra PLL setup.
#define BAUD_RATE 9600

//#define THROTTLE
/* Use THROTTLE for simple (working tested) throttle control
   The encoder is read, and limited to 0 to 25 which produces
   a PWM output between 0 and 250. */
#define MAX_COUNT 25

#define PID_POS
/* Use PID_POS for simple (NOT tested) PID position controller
   The encoder is read, and compared to Setpoint. PID is used to
   produce a PWM output between 0 and 250 to drive the motor to
   the desired position. */

// Frequency will be (FOSC/4)/((256-X)*Prescale) in 8 bit mode and
// (FOSC/4)/((65536-X)*Prescale) in 16 bit mode. Solving for X we get
// X = 256 - Hz * Prescale * FOSC/4
#define TIMER0_PRESCALE 256
#define TIMER0_COUNT (256 - (OSC_FREQ / (4UL*SAMPLE_FREQ*TIMER0_PRESCALE)))
#if TIMER0_COUNT < 0 || TIMER0_COUNT > 255
#error TIMER0 count out of range
#endif

#include <p18f14k22.h>
#define EE_BLOCK_SIZE 64

#pragma config FOSC = IRC
#pragma config WDTEN = OFF
#pragma config BOREN = OFF
#pragma config PWRTEN = ON
#pragma config MCLRE = OFF
#pragma config LVP = OFF
#pragma config HFOFST = OFF
#pragma config PLLEN = OFF

//Define Interrupt Locations
void hi_interrupt(void);
void lo_interrupt(void);

#pragma code high_vector_section=0x8
void high_vector (void){
	_asm GOTO hi_interrupt _endasm
	}
#pragma code low_vector_section=0x18
void low_vector(void){
	_asm GOTO lo_interrupt _endasm
	}
#pragma code

#ifdef THROTTLE
unsigned char encoder_counter=0; //where we are
#endif

#ifdef PID_POS
double Setpoint=0; //where we want to be
double encoder_counter=0; //where we are

//Data to support the PID
double PIDkp;	//proportional constant
double PIDki;	//integral constant
double PIDkd; 	//derivative constant
double PIDerror;	//error between Setpoint and encoder count
double PIDlastInput;//last encoder reading (for derivative)
double PIDdInput;	//change in input (for derivative)
double PIDITerm;	//integral term (calculated per unit time)
double PIDOutput;	//output

typedef union {
 struct {
  unsigned dir:1;
  };
 unsigned char byte; 
 } T_Flags;
T_Flags Flags;

#endif


#pragma udata access withports //trying to get the vars in the same bank as ports

typedef union {
 struct  {
  unsigned int b0: 1;
  unsigned int b1: 1;
  unsigned int b2: 1;
  unsigned int b3: 1;
  unsigned int b4: 1;
  unsigned int b5: 1;
  unsigned int b6: 1;
  unsigned int b7: 1;
  };
 unsigned char byte; 
 } T_Bytevar;
near T_Bytevar  temp;

typedef union {
 struct {
  unsigned A:1;
  unsigned B:1;
  unsigned dA:1;
  unsigned dB:1;
  unsigned set_pwm:1;
  };
 unsigned char byte; 
 } T_Encoder;
near T_Encoder Enc;

near unsigned char cmd; //recieved commands.
near double val; //recieved numbers.
near unsigned char decimals;

near unsigned int timer;

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(amt) ((amt)<0?0-(amt):(amt))

#pragma interrupt hi_interrupt
void hi_interrupt(void) {
	// ------------------------
	// High Priority Interrupts
	// ------------------------    

/**** STEP INPUT ****/
#ifdef PID_POS
	if (INTCON3bits.INT1IF) { //did we get a step? INT1 is RA1
		if (INPUT_DIRECTION) {
			Setpoint+=INPUT_STEP_SIZE;
			}
		else {
			Setpoint-=INPUT_STEP_SIZE;
			}
		//only need to update the error if the setpoing or encoder change
		PIDerror = Setpoint - encoder_counter;
		INTCON3bits.INT1IF = 0; //clear the interrupt
		}
#endif

/**** READ ENCODER ****/
	if (INTCONbits.RABIF) { //did Port A or B bits change?
		temp.byte = 0;
		temp.b0 = ENCODER_A; 
		temp.b1 = ENCODER_B; 

		Enc.byte ^= temp.byte; //XOR last reading with this
		Enc.byte <<= 2;        //and save that 
		Enc.byte |= temp.byte; //and save this reading.

		ERROR_STATUS = ERROR_OFF;
		if(Enc.dA && Enc.dB) { //overrun?
			ERROR_STATUS = ERROR_ON;
			TXREG = '!';
			return;
			}

		if(Enc.dA) { // A changed
			if(Enc.A==Enc.B)
				encoder_counter++;
			else
				encoder_counter--;
			}    
		else { //if(Enc.dB) { // B changed
			if(Enc.A==Enc.B)
				encoder_counter--;
			else
				encoder_counter++;
			}

#ifdef THROTTLE
		Enc.set_pwm = 1; // Set flag to indicate change to PWM needed
		if (encoder_counter>254) encoder_counter=0;
		if (encoder_counter>MAX_COUNT) encoder_counter=MAX_COUNT;
#endif

#ifdef PID_POS
		//only need to update the error if the setpoing or encoder change
		PIDerror = Setpoint - encoder_counter;
#endif

		INTCONbits.RABIF = 0; //reset port a and b interrupt flag
		//reading the port should have done this already, but...
		}
	}


#pragma interruptlow lo_interrupt
void lo_interrupt(void){
	// -----------------------
	// Low Priority Interrupts
	// -----------------------    

/**** READ SERIAL ****/
	if (PIR1bits.RCIF) { //did we recieve a byte?
		temp.byte = RCREG;
		TXREG = temp.byte; //echo it
		if('0'<=temp.byte && temp.byte<='9') {	//recieved a digit
			temp.byte-='0';
			if (decimals>0) {
				decimals++;
				}
			val *= 10;
			val += temp.byte;
			}
		else switch (temp.byte) {
			case '.': decimals=1; break;
			case '-': val = 0-val; break; //'-' must come after number
			default: cmd = temp.byte;  //it's a command
			}
		}


/**** TIMER TICK ****/
	if (INTCONbits.TMR0IF) {
		TMR0L = TIMER0_COUNT;

#ifdef PID_POS
		if (timer>0) timer--;
      /*Compute derivative and integral variables only once per unit time*/
		PIDdInput = (encoder_counter - PIDlastInput);
		PIDlastInput = encoder_counter;

		PIDITerm = constrain(
	      	PIDITerm + (PIDki * PIDerror), 
	      	0-PWM_MAX,
	      	PWM_MAX
	      	);
#endif
		INTCONbits.TMR0IF = 0;          // Clear interrupt flag
		}

	}


#ifdef PID_POS

/**** COMPUTE OUTPUT VIA PID ****/
//http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/
double ComputePID() {
	
      /*Compute PID Output*/
      PIDOutput = constrain(
      	( PIDkp * PIDerror 		//Proportional
      	+ PIDITerm 			//Integral ki already in ITerm
      	- PIDkd * PIDdInput	//Differential
      	  ), 
      	0-PWM_MAX, //minimum is negative max because direction
      	PWM_MAX
      	);
 
      return PIDOutput;
	}

#endif


/**** OUTPUT PWM ****/
void SetPWM(unsigned char pwm_width) {
	// set pwm values
	// input of 0 to 25
	// PWM output is on P1A (pin 5)

	unsigned char pwm_lsb;

	pwm_width*=10;                         // change value from 0-25 to 0-250
	//10 Bits - 2 LSB's go in CCP1CON 5:4, 8 MSB's go in CCPR1L
	pwm_lsb = pwm_width & 0b00000011;      // Save 2 LSB
	CCPR1L = pwm_width >> 2;               // Remove 2 LSB and store 8 MSB in CCPR1L (only 6 bits as max duty value = 250)
	pwm_lsb = pwm_lsb << 4;                // Move 2 LSB into correct position
	CCP1CON = pwm_lsb + 0b00001100;        // duty lowest bits (5:4) + PWM mode
	}
	
unsigned char Read_EEPROM(unsigned char address) {
	EEADR = address; // Data Memory Address to read
	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.RD = 1;
	return EEDATA;
	}

double Read_Double_EEPROM(unsigned char address) {
	unsigned char i;
	double data;
	for (i = 0; i < sizeof(double); i++) {
		*((unsigned char*)&data + i) = Read_EEPROM(address + i);
		}
	return data;
	}

void Write_EEPROM(unsigned char address, unsigned char data) {
	EEADR = address; // Data Memory Address to read
	EEDATA = data; // Data Memory Value to write
	EECON1bits.EEPGD = 0; // Point to DATA memory
	EECON1bits.CFGS = 0; // Access 1=program FLASH 0=Data EEPROM memory
	EECON1bits.WREN = 1; // Enable writes
	// required sequence
	INTCONbits.GIE = 0; // Disable interrupts
	EECON2 = 0x55; // Write 55h
	EECON2 = 0xaa; // Write AAh
	EECON1bits.WR = 1; // Set WR bit to begin write
	INTCONbits.GIE = 1; // Enable interrupts
	while (!PIR2bits.EEIF); // wait for interrupt to signal write complete
	PIR2bits.EEIF = 0; // clear EEPROM write operation interrupt flag bit
	EECON1bits.WREN = 0; // disable writes on write complete (EEIF set)
	}

unsigned char Write_Double_EEPROM(unsigned char address, double data) {
	unsigned char i;
	for (i = 0; i < sizeof(double); i++) {
		Write_EEPROM(address + i, *((unsigned char*)&data + i));
		}
	return sizeof(double);
	}

void main(void) {
#if OSC_FREQ == 16000000L
	OSCCON = 0b01110010;		// Int osc at 16 MHz
#else
#error "Update OSCON bits to match FOSC"
#endif
	TRISA = 0b11111111;			// Set Ports to all inputs.
	TRISB = 0b11111111;			// ...should be already but
	TRISC = 0b11111111;			// ...let's not take chances?

/**** SETUP UART ****/
	TRISBbits.TRISB5 = 1; // Set Port B5 for UART RX
	TRISBbits.TRISB7 = 1; // Set Port B7 for UART TX
/* Datasheet, 15.0 page 180
"For all modes of EUSART operation, the TRIS control
bits corresponding to the RX/DT and TX/CK pins should
be set to 1. The EUSART control will automatically
reconfigure the pin from input to output, as needed." */

// Baud Rate formula for SYNC=0 (Async), BRG16=1 (16-bit), BRGH=1 (high speed)
// BAUD_RATE = FOSC / (4 x (SPBRG + 1)) 
#define SPBR (OSC_FREQ/(4UL * BAUD_RATE) + 1)
	SPBRGH = SPBR/256;
	SPBRG = SPBR%256;
	TXSTA = 0  // Transmit Status and Control Register
		| 0b10000000	// CSRC Clock Source, 1=Master, 0=Slave
	//	| 0b01000000	// TX9 9-bit xmit. 1=9, 0=8 bits transmitted
	//	| 0b00000001	// TX9D Ninth bit of data in 9 bit mode.
		| 0b00100000	// TXEN Transmit enabled
	//	| 0b00010000	// SYNC 1=Synchronous mode, 0=Asynchronous mode
	//	| 0b00001000	// SENDB Send break
		| 0b00000100	// BRGH High baud rate
		;
	RCSTA = 0  // Recieve Status and Control Register
		| 0b10000000	// SPEN Serial Port Enabled (sets RB7 as output)
	//	| 0b01000000	// RX9 9-bit recieve
	//	| 0b00100000	// SREN in Synchronous mode, enables single RX
		| 0b00010000	// CREN Continuous RX Enable
	//	| 0b00001000	// ADDEN Address detect enable
		;
	BAUDCON=0  // Baud Rate Control Register
	//	| 0b00100000	// DTRXP RX Polarity. 1=Active low (inverted) 
	//	| 0b00010000	// XKTXP TX/Clock Polarity 1=idle low/falling edge
		| 0b00001000	// BRG16 1=SPBRGH:SPBRG baud rate, 0=SPBRG only
	//	| 0b00000010	// WUE Wake up enabled (async only)
	//	| 0b00000001	// ABDEN Auto-Baud detect enabled (async only)
		;


/**** SETUP INTERRUPTS ****/
	T0CON=0			// Timer 0 Control
		| 0b10000000	// Enable timer 0=disabled
		| 0b01000000	// 8 bit counter 0=16 bit
	//	| 0b00100000	// External clock 0=internal 1=external
	//	| 0b00010000	// Increment on falling edge 0=rising
	//	| 0b00001000	// Disable Prescaling 0=Enabled 1=disabled
#if 256 == TIMER0_PRESCALE
		|      0b111	// 1:256 prescale value
#elif 128 == TIMER0_PRESCALE
	    |      0b110	// 1:128 prescale value
#elif 64 == TIMER0_PRESCALE
	    |      0b101	// 1:64 prescale value
#elif 32 == TIMER0_PRESCALE
	    |      0b100	// 1:32 prescale value
#elif 16 == TIMER0_PRESCALE
	    |      0b011	// 1:16 prescale value
#elif 8 == TIMER0_PRESCALE
	    |      0b010	// 1:8 prescale value
#elif 4 == TIMER0_PRESCALE
	    |      0b001	// 1:4 prescale value
#elif 2 == TIMER0_PRESCALE
	    |      0b000	// 1:2 prescale value
#else
#error Invalid Timer0 Prescale
#endif
		;

	TMR0L = TIMER0_COUNT;		// 

//	INTCON2bits.RABPU = 1;		// Port A and B pull-up 1=DISable
//	INTCON2bits.INTEDG0 = 1;	// INT0 Edge 1=Rising / 0=Falling
	INTCON2bits.INTEDG1 = 1;	// INT1 Edge 1=Rising / 0=Falling
//	INTCON2bits.INTEDG2 = 1;	// INT2 Edge 1=Rising / 0=Falling
	INTCON2bits.TMR0IP = 0;		// Timer 0 interrupt priority 1=High / 0=Low
	INTCON2bits.RABIP = 1;		// Port A and B interrupt priority 1=High / 0=Low

#define INT_LEVELS
#ifdef INT_LEVELS
	RCONbits.IPEN = 1;			// Enable interrupt priority levels
	INTCONbits.GIEH = 1;		// High priority interrupt enable
	INTCONbits.GIEL = 1;		// Low priority interrupt enable
#else
	RCONbits.IPEN = 0;			// Disable interrupt priority levels (like '690)
	INTCONbits.GIE = 1;			//  Enable all unmasked ints
	INTCONbits.PEIE = 1;		//  Enable unmasked peripheral ints 
#endif
	INTCONbits.TMR0IE = 1;		//  Enable Time 0 Overflow interrupt
//	INTCONbits.INT0IE = 1;		//  Enable external INT0 interrupt RA0
	INTCONbits.RABIE = 1;		//  Port A and B change interrupt see  IOCA/IOCB enable


//	INTCON3bits.INT2IP = 1;		// INT2 interrupt priority 1=High / 0=Low
	INTCON3bits.INT1IP = 1;		// INT1 interrupt priority 1=High / 0=Low
//	INTCON3bits.INT2IE = 1;		// Enable external INT2 interrupt RA2
	INTCON3bits.INT1IE = 1;		// Enable external INT1 interrupt RA1


// Clear the peripheral interrupt flags
	PIR1 = 0;	//Perhipheral Interrupt Flag Register 1
	PIR2 = 0;	//Perhipheral Interrupt Flag Register 2

	PIE1bits.RCIE = 1;			// Enable Interrupt on RX
	IPR1bits.RCIP = 0;			// RX interrupt priority 1=high 0=low

//	PIE1bits.TXIE = 1;			// Enable Interrupt on TX
//	IPR1bits.TXIP = 0;			// TX interrupt priority 1=high 0=low

	ANSEL=0;                      // Digital
	ANSELH=0;                     // Digital
	ADCON0=0;                     // A2D Off
	CM1CON0=0;                    // Comparators off
	CM2CON0=0;                    // Comparators off

//ports are input by default, but just to clarify our intention:
	TRISAbits.TRISA0 = 1;
	TRISAbits.TRISA5 = 1;

	TRISAbits.TRISA1 = 1;
	WPUAbits.WPUA1 = 1;		// Enable weak pullup on step
	TRISCbits.TRISC0 = 1;
	//WPUCbits.WPUC0 = 1;		// Enable weak pullup on direction 
	//no pull up available on port C


	IOCAbits.IOCA0 = 1;			// Interrupt on Change. See INTCONbits.RABIE
	IOCAbits.IOCA5 = 1;			// Interrupt on Change. See INTCONbits.RABIE

	/*
	* PWM Register Values
	* Oscillator Frequency Fosc = 16000000
	* Clock Frequency Fclk = 4000000
	* PWM Freq = 250 - allows us to use a duty value of 0 to 250
	* Prescaler Value = 16
	* Postscaler Value = 16    
	* PR2 = 62
	* Maximum duty value = 250 
	*/
#if OSC_FREQ == 16000000L
	T2CON = 0b01111111;           // prescaler postscaler to give 250Hz + turn on TMR2;
	PR2 = 62;                     // gives 250Hz
#else
#error "Update T2CON bits and PR2 to match FOSC"
#endif
    
	CCPR1L = 0b00000000;          // set duty MSB - initially 0 - off
	CCP1CON = //ENHANCED CAPTURE/COMPARE/PWM CONTROL REGISTER
	 0b00 + 	// unused in PWM mode 
	   0b00 +	// duty lowest bits if PWM mode  
		 0b1100 // PWM mode; P1A, P1C active-high; P1B, P1D active-high
	//	 0b1101 // PWM mode; P1A, P1C active-high; P1B, P1D active-low
	//	 0b1110 // PWM mode; P1A, P1C active-low; P1B, P1D active-high
	//	 0b1111 // PWM mode; P1A, P1C active-low; P1B, P1D active-low
		;

	PSTRCONbits.STRD = 1;	//steer the output to P1D, RC2
	PSTRCONbits.STRA = 0;	// ... not P1A RC5
	TRISCbits.TRISC2 = 0;	// Set pin C2 1=input / 0=output motor PWM
	TRISAbits.TRISA4 = 0;	// Set pin A4 0=output for motor direction

	timer=SAMPLE_FREQ/2;	// Dividing by 2 gives 1/2 second blink
	ERROR_STATUS=ERROR_ON;	// Let the user know we are coming on.
	TRISAbits.TRISA2 = 0;	// Set A2 output for XErr / LED on J12
	while(timer>0) ;
	ERROR_STATUS=ERROR_OFF;	//

	MOTOR_ENABLE = 0; 	// Set Motor disabled
	TRISCbits.TRISC7 = 0;	// Set pin C7 0=output for motor enable


#ifdef PID_POS
	/**** INITIALIZE PID ****/

	//try EEPROM
	if ('p'==Read_EEPROM(0) ) {
		Flags.byte = Read_EEPROM(1);
		PIDkp = Read_Double_EEPROM(2 + (0*sizeof(double)));
		PIDki = Read_Double_EEPROM(2 + (1*sizeof(double)));
		PIDkd = Read_Double_EEPROM(2 + (2*sizeof(double)));
		}
	else {
		Flags.dir = 1;
		PIDkp = PID_KP_START;
		PIDki = PID_KI_START;
		PIDkd = PID_KD_START;
		}
	PIDlastInput = encoder_counter; //avoid startup bump
	PIDITerm = 0;
	//PIDITerm = constrain(PIDOutput, 0-PWM_MAX, PWM_MAX);	
	//Even if previously running (can't be in this case)
	Setpoint = encoder_counter + 1; //just for testing.
	while(1) {
		if (cmd) { //we got a new value and command.
			while (1 < decimals--) 
				val/=10;
			if (13==cmd && 0!=val) { //cmd was just enter
				Setpoint = val;
				PIDerror = Setpoint - encoder_counter;
				}
			else if ('e'==cmd) { //enable motor
				PIDlastInput = encoder_counter; //avoid startup bump
				MOTOR_ENABLE = 1;
				}
			else if (' '==cmd) { //disable motor
				MOTOR_ENABLE = 0;
				ERROR_STATUS = 0; //clear errors.
				}
			else if ('p'==cmd && 0<=val) { //set new kp
				PIDkp = val;
				}
			else if ('i'==cmd && 0<=val) { //set new ki
				PIDki = val;// *(double)PID.SampleTime/1000;
				}
			else if ('d'==cmd && 0<=val) { //set new kd
				PIDkd = val;// /(double)PID.SampleTime/1000;
				}
//			else if ('t'==cmd && 0<val) { //set a new sample time
//				double ratio  = (double)val
//					/ (double)PID.SampleTime;
//				PID.ki *= ratio;
//				PID.kd /= ratio;
//				PID.SampleTime = (unsigned long)val;
//				}
			else if ('w'==cmd && 321==val) { //save parms
				//TODO: let the user know they need to enter 321w
				Write_EEPROM(0,Flags.byte); //Save flags
				Write_Double_EEPROM(2 + (0*sizeof(double)),PIDkp);
				Write_Double_EEPROM(2 + (1*sizeof(double)),PIDki);
				Write_Double_EEPROM(2 + (2*sizeof(double)),PIDkd);
				Write_EEPROM(0,'p'); //flag the EEPROM
				}
			else if ('w'==cmd && 216==val) { //direction high
				Flags.dir = 1;
				}
			else if ('w'==cmd && 286==val) { //direction low
				Flags.dir = 0;
				}
			cmd=0; //done with command
			val=0; //reset value for next reception
			decimals = 0;
			}

		ComputePID();

		if (PIDOutput>0) {
			MOTOR_DIRECTION = Flags.dir; //set direction forward
			}
		else {
			MOTOR_DIRECTION = !Flags.dir; //set direction backward
			}
		SetPWM(abs(PIDOutput)/OUT_RES_DIV);
		}
#endif

#ifdef THROTTLE

	while(1) {
		if(Enc.set_pwm) {
			//Set PWM values
			SetPWM(encoder_counter);
			Enc.set_pwm = 0;
			}
		}
#endif

	}


