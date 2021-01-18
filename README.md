# MotorSpeedController  
yaya.

# Hardware  
  

# Software  
Contains code for the controller itself and code for a master device to interface with the controller.  
 * **controller** - Code for the speed controller itself.  
 * **controllerInterfaceLib** - Library files for use on a master device to interface with the controller.  
 
  ## Wiring Connections and Interface  
 This controller uses standard SPI communications.  
| Arduino | Speed Controller |
|---------|------------------|
| 5V      | 5V               |
| GND     | GND              |
| MOSI    | MOSI             |
| MISO    | MISO             |
| SCK     | SCK              |
| SS*     | SS               |  

*The SS pin must be used for *at least one* of the connections. Additional speed controllers may use other digital IO pins.  
 
 
 ## Interface Lib Usage  
**Create:**  
Create an instance of the class.  

    Nidec24hController motor(SPI, ssPin);  

*SPI* - the SPI singleton object from the SPI.h library.  
*ssPin* - pin to use as chip select. Should simply be SS for first controller.  

**Initialize:**  
Create an instance of the class.  

    setup() {
        ...
        // Init SPI
        SPI.begin ();
        SPI.setClockDivider(SPI_CLOCK_DIV4);
        
        //Init motor controller
        motor.init();  
        ...
    }


**Set Speed:**  
Set the speed for the motor in RPM. Negative values are allowed. This will switch the motor to speed control mode if in power control mode or braking.  

    motor.setSpeed(speed);  

*speed* - the speed for the motor in RPM.  

**Set Power:**  
Set the power for the motor (-1000 to 1000). This will switch the motor to power control mode if in speed control mode or braking.  

    motor.setPower(power);  

*power* - the power for the motor (-1000 to 1000).  

**Brake:**  
Engage the electronic braking and stop the motor..  

    motor.brake();  

**Set Direction Inverted:**  
Set whether the motor control should be inverted.  

    motor.invertDirection(invertDir);  

*invertDir* - boolean to set if direction should be inverted.  

**Get Current Speed:**  
Get the current speed of the motor in RPM.   

    motor.getSpeed();  

**Get Current Power:**  
Get the current power the motor is running at. If in speed control mode, this will be varied automatically by the PID algorithm.  

    motor.getPower();  

**Get the Target Speed:**  
Get the target speed the controller is attempting to regulate the motor at.  

    motor.getTargetSpeed();  
