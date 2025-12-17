# BOBPID
The goal here was to provide a basic but useful servo driver at very low cost (blank PIC 18F14k22's are under $2, the BOB PCB is $8, couple bucks misc parts) which could replace stepper motors in any standard application, such as 3D printers or CNC machines. 

It accepts both serial and step / direction inputs. The serial interface allows for feedback of status / position and easy tuning of the P.I.D. constants and setting options like the polarity of the output direction signal. Simple position goal setting and trap-plan motion is also available from the serial interface. The step / direction input allows it's use in any application where a standard stepper driver was intended. 

The output is PWM speed and direction which will drive most any standard H-Bridge or other motor driver.

Instructions, PCB, kit, etc... all at:

http://techref.massmind.org/techref/io/servo/BOBPID.htm

Quick video of it running from a 3D printer RAMPS/Marlin controller:
- https://www.youtube.com/watch?v=rnwYqGB3bXw Using a home made driver ~1hp.
- https://www.youtube.com/watch?v=Wxd9-VgvRpI Using a SyRen 25 to 50 amp 
- https://www.youtube.com/watch?v=TFkugHdCh3Y Another smaller DC motor, still pretty strong!
- https://www.youtube.com/watch?v=AVgdFyZaA00 AmpFlow E30 150
- https://www.youtube.com/watch?v=TjflUkI--Sw Motor with built in optical encoder. 

## The Monster Servo!

Demonstration of the sort of power you can get with used DC gear motors:
- https://www.youtube.com/watch?v=EHmwiAFREVE Raw power
- https://www.youtube.com/shorts/ekbzLTYLiSY Smoother motion

PID Tuning:
- https://www.youtube.com/watch?v=qii1SxnRzJw

