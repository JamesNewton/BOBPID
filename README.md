# BOBPID
The goal here was to provide a basic but useful servo driver at very low cost (blank PIC 18F14k22's are under $2, the BOB PCB is $8, couple bucks misc parts) which could replace stepper motors in any standard application, such as 3D printers or CNC machines. 

It accepts both serial and step / direction inputs. The serial interface allows for easy tuning of the P.I.D. constants and setting options like the polarity of the output direction signal. The step / direction input allows it's use in any application where a standard stepper driver was intended. 

The output is PWM speed and direction which will drive most any standard H-Bridge or other motor driver.

Instructions, PCB, kit, etc... all at:

http://techref.massmind.org/techref/io/SERVO/BOBPID.htm

Quick video of it running from a 3D printer RAMPS/Marlin controller:

https://www.youtube.com/watch?v=rnwYqGB3bXw
