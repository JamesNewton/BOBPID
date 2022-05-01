
// MassMind.org BOB P.I.D. Servo controller
// Board, kit, instructions at:
// http://techref.massmind.org/techref/io/SERVO/BOBPID.htm
// Using MPLAB IDE v8.84 C18 v3.31
// Starting from:
// http://www.hobbytronics.co.uk/rotary-encoder-tutorial
// Modified by James Newton at MassMind org
// Testing by Jim Fongjm_fong@yahoo.com
// Possible DRO testing by robertgeo4@charter.net
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
// [X] Support bootloader for firmware upgrades.
//     http://picprog.strongedge.net/bootloader/bootloader.html
// [X] use enable from step direction input.
// [X] 0.92 Why was it using double? Switch to long 
// [X] 0.92 Add config parameter for step size
// [X] 0.93 Fixed OUT_RES_DIV 16 vs 10 bug
// [X] 0.93 Faster! Change to 64MHz clock 
// [X] 0.94 Faster!! Just track error, vs Setpoint - encoder_counter 
//	Note: Step/Direction input is now relative. Calculation of 
//	Derivative and Integral terms changed. May need to re-tune
//  EEPROM Version '3'
// [X] 0.94 Fix: Actually save the step size parameter
// [X] 0.95 Just one "!" per encoder overrun; avoid buffer overrun
// [X] 0.95 Add version / status / settings as "?" command
// [x] 0.95 Let the user know they need to enter 321w to save
// [X] 0.95 Past MAX_ERROR, disable and suggest other direction
// [X] 0.95 Movements via serial command have a max velocity 'v'
//	Setpoint is now relative, always returns to 0, no wraparaound
//	Smaller datasizes (short long, 24 bits) used for most variables
//  EEPROM Version '4'
// [X] 0.96 BugFix: 10 display digits exceeded precision. 9 works.
// [X] 0.96 BugFix: Avoid PIDerror changing during calculations.
// [x] 0.96 Periodic error output in reverse HEX. E.g. 15 is +F00000.
//  "TEll" 7311w turns it on, "lESS" 1355w off.
// [X] 0.96 Add 'r' command to report error
// [ ] 0.96 Error output in standard hex.
// [ ] RC PWM: 100Hz PWM with zero at 1.5mS, reverse at 1mS
//  and forward at 2mS) so we can support ESC's for BLDC motors. 
// [ ] RC Servo signal input
// [ ] Faster!! Switch to assembly jump table in decoder?
// [ ] Recover position via index pulse if available??
// [ ] Support digital modes with AS brand encoders???
// [ ] Set velocity and use PID to control that instead of position
// [ ] Add LCD user interface?
//     Problem with LCD interface: We don't control X DIR input
//      which is also used by LCD as a data line.
// [ ] Add demo mode
// [ ] Add autotune mode 

#define VERSION_STRING "0.96"
#define veloc

#define MOTOR_ENABLE LATCbits.LATC7  	//Enables Motor Driver
#define MOTOR_DIRECTION LATAbits.LATA4	//Sets Motor Direction
#define MOTOR_SPEED LATCbits.LATC2  	//Set Motor PWM output (must be P1x)

#define ENCODER_A PORTAbits.RA0        // Encoder A Pin (must be IOC)
#define ENCODER_B PORTAbits.RA5        // Encoder B Pin (must be IOC)
// from ZSTEP on Z-Y LED pins via JP5 and
// (slant) jumper from ASTEP to ZSTEP over RP2. A AXIS not connected

#define INPUT_STEP PORTAbits.RA1		//Step input (must be INT1)
#define INPUT_DIRECTION PORTCbits.RC0	//which direction do we step?
#define INPUT_ENABLE (!PORTBbits.RB6)	//! assumes active low enable
#define INPUT_STEP_SIZE 2				//how much do we step?
#define ERROR_STATUS LATAbits.LATA2		//A2 jumper to Xerr/LED on J12
// J12 pin 3 anode, pin 2 cathode bridge to pin 1 330R to GND J11 pin 5
// C5 is spindle/LED or LCD /E
#define ERROR_ON 1
#define ERROR_OFF 0

#define	PID_KP_START 0.2
#define	PID_KI_START 0.01
#define	PID_KD_START 0.1
#define EEPROM_VERSION '4'
//change when layout of EEPROM data changes so that new code updates
//don't cause garbled data.

#define SAMPLE_TIME .005	//Seconds
#define SAMPLE_FREQ (1/SAMPLE_TIME) //Hz

#define PWM_MAX 253
//#define PWM_MIN 0

#define MAX_ERROR 100000

#define OUT_RES_DIV 2	//Amount to divide the output by for PWM.
//Too low a value her make the units for the PID impossibly small
//and makes resolution two high. Too high reduces resolution.

//#define OSC_FREQ 16000000L	
//Max 16 MHz in standard modes.
#define OSC_FREQ 64000000L	
//Up to 64M available with extra PLL setup.
//#define BAUD_RATE 9600
#define BAUD_RATE 38400


#define DISPLAY_DIGITS 7
#define DISPLAY_PRECISION 3
#define EOL 13
#define SET_CMD 'w'



#if OSC_FREQ == 16000000L
// Frequency will be (FOSC/4)/((256-X)*Prescale) in 8 bit mode 
// Solving for X we get X = 256 - Hz * Prescale * FOSC/4
 #define TIMER0_PRESCALE 256
 #define TIMER0_COUNT (256 - (OSC_FREQ / (4UL*SAMPLE_FREQ*TIMER0_PRESCALE)))
 #if TIMER0_COUNT < 0 || TIMER0_COUNT > 255
  #error TIMER0 count out of range
  #endif
 #endif

#if OSC_FREQ == 64000000L
// Frequency will be (FOSC/4)/((65536-X)*Prescale) in 16 bit mode. 
// Solving for X we get  X = 655536 - Hz * Prescale * FOSC/4
 #define TIMER0_PRESCALE 256
 #define TIMER0_COUNT (65536 - (OSC_FREQ / (4UL*SAMPLE_FREQ*TIMER0_PRESCALE)))
 #if TIMER0_COUNT < 0 || TIMER0_COUNT > 65535
  #error TIMER0 count out of range
  #endif
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
#if OSC_FREQ == 64000000L
#pragma config PLLEN = ON
#else
#pragma config PLLEN = OFF
#endif

//Define Interrupt Locations
void hi_interrupt(void);
void lo_interrupt(void);

#define BOOTLOADER
/*To enable the bootloader, with output in <project>.HEX file
Find \src\traditional\startup\c018i.c, make local copy, add to project
Edit as follows:

//#pragma code _entry_scn=0x000000
//void
//_entry (void)
//{
//_asm goto _startup _endasm
//}
//removed inorder to support.
//http://picprog.strongedge.net/bootloader/bootloader.html

Find \bin\LKR\18f14k22_g.lkr, make local copy, add to project
Edit as follows:

#IFDEF _CRUNTIME
  #IFDEF _EXTENDEDMODE
#error Make custom c018i_e.c file as per c018i.c below.
    FILES c018i_e.o
    FILES clib_e.lib
    FILES p18f14k22_e.lib

  #ELSE
//    FILES c018i.o
//Removed because we have a local custom ver in order to support 
//http://picprog.strongedge.net/bootloader/bootloader.html
    FILES clib.lib
    FILES p18f14k22.lib
  #FI

#FI


*/

#ifdef BOOTLOADER
extern void _startup (void);
#pragma code _entry=0x00
void
_entry (void)
{
_asm 
	goto _startup 
	goto _startup 
high_vector:
	GOTO hi_interrupt
_endasm
	}

#else
#pragma code high_vector_section=0x8
void high_vector (void){
	_asm GOTO hi_interrupt _endasm
	}
#endif


#pragma code low_vector_section=0x18
void low_vector(void){
	_asm GOTO lo_interrupt _endasm
	}


#pragma code

void ComputePID();

//short long is 24 bits, was long, 32 bits
#define loctype short long

#pragma udata access withports //trying to get the vars in the same bank as ports
//for this code, it doesn't make any difference. If included, add "near" to vars below.

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

near loctype Setpoint; //distance to the goal
near loctype Slowpoint; //where we want to start slowing down
near unsigned int move; //amount to move this tick.
near unsigned int vmax; //max Setpoint change per sample time
near unsigned int velocity; //current change per sample time
near char amax; //max change in change per sample time TODO: Save this
near char accel; //current change in change per sample time

//Data to support the PID
near loctype PIDerror;	//error between Setpoint and encoder count

near double PIDkp;	//proportional constant
near double PIDki;	//integral constant
near double PIDkd; 	//derivative constant

near loctype PIDlastInput;//last encoder reading (for derivative)
near loctype PIDdInput;	//change in input (for derivative)
near int PIDITerm;	//integral term (calculated per unit time)
near int PIDOutput;	//output

typedef union {
 struct {
  unsigned dir:1;
  unsigned inputEnable:1;
  unsigned neg:1;
  unsigned saved:1;
  unsigned errout:1;
  };
 unsigned char byte; 
 } T_Flags;
near T_Flags Flags;

near unsigned char step_size;

near unsigned char cmd; //recieved commands.
near double val; //recieved numbers.
near unsigned char decimals;




near unsigned int timer;

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define abs(amt) ((amt)<0?0-(amt):(amt))
#define min(v1,v2) ((v1)<(v2)?(v1):(v2))
#define smaxval(t) (((0x1ULL << ((sizeof(t) * 8ULL) - 1ULL)) - 1ULL) | \
                    (0x7ULL << ((sizeof(t) * 8ULL) - 4ULL)))
                    
#pragma interrupt hi_interrupt
void hi_interrupt(void) {
	// ------------------------
	// High Priority Interrupts
	// ------------------------    

/**** STEP INPUT ****/
	if (INTCON3bits.INT1IF) { //did we get a step? INT1 is RA1
		if (INPUT_DIRECTION) {
			PIDerror+=step_size;
			}
		else {
			PIDerror-=step_size;
			}
		INTCON3bits.INT1IF = 0; //clear the interrupt
		}

/**** READ ENCODER ****/
/* Yes, this code looks stupid but it's actually pretty fast. 
It's an attempt to get a basic C compiler to generate good asm code.
Yes, those are goto's in a C program. They were cheaper than a call
to a function, saving several clock cycles and allowing faster encoders.
I probably should just re-write all this in asm, because then I could 
use a jump table, but at least this way it's still C and mostly readable.
The switch / case is even worse... C18 doesn't make a jump table.
*/
	if (INTCONbits.RABIF) { //did Port A or B bits change?
		temp.byte = 0;
		temp.b0 = ENCODER_A; 
		temp.b1 = ENCODER_B; 

		Enc.byte ^= temp.byte; //XOR last reading with this
		
		Enc.byte <<= 2;        //and save that 
//_asm	//C compiler literally multiplies by 4, but this is no faster
//	BCF STATUS,0,ACCESS //clear carry otherwise can rotate in 1's
//	RLCF Enc, 1, 0 //just "Enc" because ASM doesn't know structures
//	BCF STATUS,0,ACCESS //clear carry otherwise can rotate in 1's
//	RLCF Enc, 1, 0 //1=put result back in file register, 0=in access bank
//_endasm
// And this won't work because high bits get rotated back in.
//		Rlncf(Enc, 1, ACCESS); //Not Enc.byte, because the assembler has no union
//		Rlncf(Enc, 1, ACCESS); //1=save in reg, ACCESS because Enc is near
		
		Enc.byte |= temp.byte; //and save this reading.

		if(Enc.dA && Enc.dB) { //overrun?
			if (ERROR_ON != ERROR_STATUS) TXREG = '!'; //just one per warning.
			ERROR_STATUS = ERROR_ON;
			timer=SAMPLE_FREQ/4;	// Dividing by 4 gives 1/4 second blink
			return;
			}

		if(Enc.dA) { // A changed
			if(Enc.A) { // if A==B inc else dec
				if (Enc.B)
					 goto encoderinc; //B and now A
				else
					 goto encoderdec; //Not B and now A
				}
			else { //!Enc.A
				if (Enc.B)
					goto encoderdec; //B and now not A
				else
					goto encoderinc; //Not B and now not A
				}
			}
		else { //if(Enc.dB) { // B changed
			if(Enc.A) { // If A==B dec else inc
				if (Enc.B)
					goto encoderdec; //A and now B
				else
					goto encoderinc; //A and now not B
				}
			else { //!Enc.A
				if (Enc.B)
					goto encoderinc; //Not A and now B
				else
					goto encoderdec; //Not A and now not B
				}
			}

encoderinc:
		PIDerror--; //positive motion is negative error
		goto encoderdone;
encoderdec:
		PIDerror++; //negative motion is positive error
		goto encoderdone; //just so there isn't any speed difference.
encoderdone:
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
			case '-': Flags.neg=1; break; //'-' must come after number
			default: cmd = temp.byte;  //it's a command
			}
		}


/**** TIMER TICK ****/
	if (INTCONbits.TMR0IF) {

		TMR0H = TIMER0_COUNT / 256;
		TMR0L = TIMER0_COUNT;

		move=min(velocity,abs(Setpoint));
		if (Setpoint>0) {
			PIDerror+=move;
			Setpoint-=move;
			if (accel>=0 && Setpoint < Slowpoint) accel = -amax;
			if (velocity < vmax) Slowpoint += move;
			} 
		else if (Setpoint<0) {
			PIDerror-=move; 
			Setpoint+=move;
			if (accel>=0 && Setpoint > Slowpoint) accel = -amax;
			if (velocity < vmax) Slowpoint -= move;
			}

		velocity += accel;
		if (velocity > vmax) {
			velocity = vmax;
			accel = 0;
			}
			
		if (timer>0) 
			timer--;

      /*Compute derivative and integral variables only once per unit time*/
		//PIDdInput = PIDlastInput - PIDerror;
		//PIDlastInput = PIDerror;
		PIDdInput = PIDlastInput;
      	INTCONbits.RABIE = 0; //keep PIDerror from changing
		PIDlastInput = PIDerror;
		INTCONbits.RABIE = 1; //allow PIDerror to change
		PIDdInput -= PIDlastInput;

		//PIDITerm = PIDITerm + (PIDki * PIDerror);
		PIDITerm = PIDITerm + (PIDki * PIDlastInput); //avoid PIDerror changing
		
		PIDITerm = constrain(PIDITerm, 0-PWM_MAX, PWM_MAX);
		//if you try to do this all at once, C18 runs out of temp variable space.

		INTCONbits.TMR0IF = 0;          // Clear interrupt flag
		}

	}



/**** COMPUTE OUTPUT VIA PID ****/
//http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/
void ComputePID() {
	
    /*Compute PID Output*/
    //PIDOutput = 
    //	+ PIDkp * PIDerror 	//Proportional
    //	+ PIDITerm 			//Integral ki already in ITerm
    //	- PIDkd * PIDdInput	//Differential
	//	;

	INTCONbits.RABIE = 0; //keep PIDerror from changing
	PIDOutput = PIDerror;
	INTCONbits.RABIE = 1; //allow PIDerror to change
	PIDOutput *= PIDkp;				//Proportional
	PIDOutput += PIDITerm; 			//Integral ki already in ITerm
	PIDOutput -= PIDkd * PIDdInput;	//Differential

	//must do the contrain separately or C18 runs out of temp variable space.
	PIDOutput = constrain( PIDOutput,
    	0-PWM_MAX, //minimum is negative max because direction
    	PWM_MAX
    	);

 	}



/**** OUTPUT PWM ****/
void SetPWM(unsigned char pwm_width) {
	// set pwm values
	// input of 0 to 255 ??? no it isn't... 
	// PWM output is on P1A (pin 5)

	unsigned char pwm_lsb;

	pwm_width*=OUT_RES_DIV;                // 
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
	
void putchar(char c) {
	while (!PIR1bits.TXIF)
   	    ;
   	TXREG = c;
	}

void puts(char *s, unsigned char len) {
    while (s && *s && len--)
        putchar (*s++);
	}

void puts_lit(const rom char *zstr) {
	while(*zstr) {
		putchar(*zstr);
    	zstr++;
    	}
	}


const long pow10[10] = {1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000};

void puts_int( //send positive integers to STDOUT via putchar(char)
   unsigned int num	//positive integer to display
  ,char digits	//count of digits, negative and '0' pad for trailing zeros
  ,char pad		//character to pad with, or null 
  ) {
unsigned int i;
char c;
unsigned char d;
  d = abs(digits);
  while (0 < d) {
    i = num/pow10[--d]; //pre-increment
    if (d && !i) { //no digits found yet
      if (pad) putchar(pad); 
      }
    else { //found something
      c = i%10; //only want the lowest digit
      //Optional: Don't continue after fraction done
      if ('0'==pad && 0<digits && 0==c) return; 
      putchar(c+'0'); //convert to ASCII and send
      }
    }
  return;
  }

void puts_double( //send floating point numbers to STDOUT
    float f  	//number to display
  , char precision	//precision, negative to keep trailing zeros
  ) {
unsigned int a;
 
// check for negative float
  if(f<0.0) { //is it negative?
    putchar('-'); //indicate
    f*=-1; //make it positive
    }
 
  a=(int)f;	// extract whole number
  puts_int(a,DISPLAY_DIGITS,0);
  if (precision) {
    putchar('.');
    f-=(float)a; //remove whole part
    f*=pow10[abs(precision)]; // promote to precision
    f+=0.5; // round
    a=(int)f; // extract whole number
    puts_int(a,precision,'0');
    }
  return; 
  }


//char hexnibble (unsigned char b) {
//	if (b>9) b+=('A'-10); else b+='0';
//	return b;
//	}
//Nah.. Memory is cheap
#define hexnibble(b) "0123456789ABCDEF"[b]

void puts_hex(near loctype h) {
	near char buf[sizeof(loctype)*2+1];
	near unsigned char i;
	buf[sizeof(loctype)*2+1]=0; //need a trailing zero to output via puts Thanks Aram!
	if (0 > h) {
		putchar('-');
		h = -h;
		}
	else {
		putchar('+');
		}
	for (i = sizeof(loctype)*2-1; i>=0; i--) { //down count is faster
		buf[i] = hexnibble((unsigned char)h&0xF);
		h/=16; // easily optimized by most compilers
		}
	puts((char*)buf, sizeof(buf)); 
	//stupid cast to avoid warning from compiler
	//http://www.microchip.com/forums/m53856.aspx
	}

void main(void) {
#if OSC_FREQ == 16000000L
	OSCCON = 0b01110010;		// Int osc at 16 MHz
#define OSCON_OK
#endif
#if OSC_FREQ == 64000000L
	OSCCON = 0b01110000;		// Int osc at 16 MHz, System clock primary
	OSCTUNE = 0b01000000;		// PLL enabled, 
#define OSCON_OK
#endif

#ifndef OSCON_OK
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
#if OSC_FREQ == 16000000L
		| 0b01000000	// 8 bit counter 0=16 bit
#endif
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

	TMR0H = TIMER0_COUNT/256;		// 
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
	T2CON = 0b01111111;           // prescaler postscaler max + turn on TMR2;
	PR2 = 62;                     // gives 250Hz
#elif OSC_FREQ == 64000000L
	T2CON = 0b01111111;           // prescaler postscaler max + turn on TMR2;
	PR2 = 62;                     // should give 250Hz?

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

	MOTOR_ENABLE = 0; 	// Set Motor disabled
	TRISCbits.TRISC7 = 0;	// Set pin C7 0=output for motor enable


	/**** INITIALIZE PID ****/

	amax = 1; //TODO: actually save and restore this
	
	//try EEPROM
	if (EEPROM_VERSION==Read_EEPROM(0) ) {
		Flags.byte = Read_EEPROM(1);
		PIDkp = Read_Double_EEPROM(2 + (0*sizeof(double)));
		PIDki = Read_Double_EEPROM(2 + (1*sizeof(double)));
		PIDkd = Read_Double_EEPROM(2 + (2*sizeof(double)));
		step_size = Read_EEPROM(2 + (3*sizeof(double)));
		vmax = Read_EEPROM(3 + (3*sizeof(double)))
				+ (Read_EEPROM(4 + (3*sizeof(double)))<<8);
		}
	else { //load defaults
		Flags.dir = 0;
		Flags.inputEnable = 0; //don't track input enable
		Flags.saved=0;
		PIDkp = PID_KP_START;
		PIDki = PID_KI_START;
		PIDkd = PID_KD_START;
		step_size = INPUT_STEP_SIZE;
		vmax = 10;
		puts_lit("NO SETTINGS!");
		}

	PIDerror = 0; //avoid startup bump
	PIDITerm = 0;
	//PIDITerm = constrain(PIDOutput, 0-PWM_MAX, PWM_MAX);	
	//Even if previously running (can't be in this case)
	Setpoint = 0;
	cmd = '?';
	while(1) {
		if (cmd) { //we got a new value and command.
			while (1 < decimals--) //convert to fraction
				val/=10; //via repeated decimal shift
			if (Flags.neg) { val = -val; Flags.neg=0;} //apply sign
			//value is now ready
			if (13==cmd && 0!=val) { //cmd was just enter
				Setpoint = val - Setpoint; //Setpoint is really position error
				velocity = Slowpoint = 0;
				accel = amax;
				//PIDerror += val - Setpoint;
				//Setpoint = val;
				}
			else if ('q'==cmd) { //Question. Returns error
				puts_hex(PIDerror); //pass by value so copy unchanged
				}	
//			else if ('t'==cmd) { //Tune (reserved)
//				}	
			else if ('z'==cmd) { //zero error
				PIDerror=0; //We want to be where we are
				}	
			else if ('e'==cmd) { //enable motor
				PIDerror=0; //We want to be where we start
				Setpoint=vmax; //give a little kick to start
				SetPWM(0);
				MOTOR_ENABLE = 1;
				ERROR_STATUS = 0; //clear errors.
				}
			else if (' '==cmd || 27==cmd || '!'==cmd) { //disable motor
				Flags.inputEnable = 0; //don't track input enable
				Flags.errout = 0; //stop reporting error
				MOTOR_ENABLE = 0; //Disable the motor
				ERROR_STATUS = 0; //clear errors.
				}
			else if ('p'==cmd && 0<=val) { //set new kp
				PIDkp = val;
				Flags.saved=0;
				}
			else if ('i'==cmd && 0<=val) { //set new ki
				PIDki = val;// *(double)PID.SampleTime/1000;
				Flags.saved=0;
				}
			else if ('d'==cmd && 0<=val) { //set new kd
				PIDkd = val;// /(double)PID.SampleTime/1000;
				Flags.saved=0;
				}
			else if ('s'==cmd && 0<val && 2^sizeof(step_size)>val) { //set a new step size
				step_size=(char)val;
				Flags.saved=0;
				}
			else if ('v'==cmd && 0<val && 2^sizeof(vmax)>val) { //set a new maximum velocity
				vmax=(unsigned int)val;
				Flags.saved=0;
				}
			else if ('a'==cmd && 0<val && 2^sizeof(amax)>val) { //set a new maximum acceleration
				amax=(unsigned int)val;
				Flags.saved=0;
				}
/* The SET_CMD commands
Current:
ZIG  	216 	Set direction
ZAG   	286 	
GO  	60   	Enable
SO  	50  	Disable
BOOT	8007	Reboot
TELL	7311	Error send
LESS	1355	No error

Possible:

HELP	4319
STAT	5747
TEST	7357
GAGE	6463
GAGS	6465
LOG 	106
LOT 	107
*/
			else if (SET_CMD==cmd && 321==val) { //save parms
				Flags.saved=1;
				Write_EEPROM(1,Flags.byte); //Save flags
				Write_Double_EEPROM(2 + (0*sizeof(double)),PIDkp);
				Write_Double_EEPROM(2 + (1*sizeof(double)),PIDki);
				Write_Double_EEPROM(2 + (2*sizeof(double)),PIDkd);
				Write_EEPROM(2 + (3*sizeof(double)),step_size);
				Write_EEPROM(3 + (3*sizeof(double)),vmax&0xFF);
				Write_EEPROM(4 + (3*sizeof(double)),vmax>>8);
				Write_EEPROM(0,EEPROM_VERSION); //flag the EEPROM
				}
			else if (SET_CMD==cmd && 8007==val) { //reset to bootloader
_asm
	RESET
_endasm
				}
			else if (SET_CMD==cmd && 216==val) { //direction high
				Flags.dir = 1;
				Flags.saved=0;
				}
			else if (SET_CMD==cmd && 7311==val) { //start continuous error report
				Flags.errout = 1;
				Flags.saved=0;
				}
			else if (SET_CMD==cmd && 1355==val) { //start continuous error report
				Flags.errout = 0;
				Flags.saved=0;
				}
			else if (SET_CMD==cmd && 286==val) { //direction low
				Flags.dir = 0;
				Flags.saved=0;
				}
			else if (SET_CMD==cmd && 50==val) { //So carefull
				Flags.inputEnable = 0; //don't track input enable
				Flags.saved=0;
				}
			else if (SET_CMD==cmd && 60==val) { //Go!
				Flags.inputEnable = 1; //we are enabled if input is
				Flags.saved=0;
				}
			else if ('r'==cmd ) { //Read error
				puts_hex(PIDerror); //pass by value so copy unchanged
				}
			else if ('?'==cmd ) { //Help
				puts_lit("\r\nMassMind.org BOB P.I.D. v");
				puts_lit(VERSION_STRING);
				//puts_lit("\r\n");
				puts_lit(" err:");
				puts_double(PIDerror,0);
				puts_lit(" ");
				if (Setpoint) puts_double(Setpoint,0);
				puts_lit("\r\n");
				puts_double(PIDkp,DISPLAY_PRECISION);
				puts_lit("p ");
				puts_double(PIDki,DISPLAY_PRECISION);
				puts_lit("i ");
				puts_double(PIDkd,DISPLAY_PRECISION);
				puts_lit("d ");
				puts_int(step_size,DISPLAY_PRECISION,0);
				puts_lit("s ");
				puts_int(vmax,DISPLAY_PRECISION,0);
				puts_lit("v ");
				puts_lit(Flags.dir?"216w":"286w");
				puts_lit(Flags.inputEnable?"60w":"50w");
				puts_lit(Flags.errout?"7311w":"1355w");
				puts_lit("\r\n");
				if (!Flags.saved) {
					puts_lit("321w 2Save\r\n");
					}
				}
			cmd=0; //done with command
			val=0; //reset value for next reception
			decimals = 0;
			}
		
		if (Flags.inputEnable) {
			if (INPUT_ENABLE) {
				MOTOR_ENABLE = 1;
				}
			else {
				MOTOR_ENABLE = 0;
				ERROR_STATUS = 0; //clear errors.
				}
			}
			
		ComputePID();
		if (PIDOutput>0) {
			MOTOR_DIRECTION = Flags.dir; //set direction forward
			}
		else {
			MOTOR_DIRECTION = !Flags.dir; //set direction backward
			}
		SetPWM(abs(PIDOutput)/OUT_RES_DIV);

		if (MOTOR_ENABLE && MAX_ERROR<abs(PIDerror)) { //error out of control
			MOTOR_ENABLE=0;
			ERROR_STATUS = ERROR_ON;
			puts_lit("\r\nMAX ERROR! Try ");
			puts_lit(Flags.dir?"286w":"216w"); //try the opposite
			PIDerror = 0; //clear the error.
			}
		if (!timer) {
			ERROR_STATUS=ERROR_OFF;	
// This method often ends up sending data when the controller isn't ready for it. 
// better to poll with the 'r' command. 
			if (Flags.errout) { //need to send error and reset timer
				puts_hex(PIDerror); //pass by value so copy unchanged
				timer = SAMPLE_FREQ/5; //(BAUD_RATE/10); 3840 is way to fast.
				}
			}

		}

	}


