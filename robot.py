#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import rev
import wpilib
import wpilib.drive

kY = 4 #Y Button
kX = 3 #X Button
kB = 2 #B Button
kA = 1 #A Button
kLB = 5 #LB Button
kRB = 6 #RB Button
kBack = 7 #Back Button
kStart = 8 #Start Button

class MyRobot(wpilib.TimedRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""

        self.joystick = wpilib.XboxController(0)

        self.lf_motor = rev.CANSparkMax(2, rev.CANSparkLowLevel.MotorType.kBrushed)
        self.lr_motor = rev.CANSparkMax(9, rev.CANSparkLowLevel.MotorType.kBrushed)
        self.rf_motor = rev.CANSparkMax(3, rev.CANSparkLowLevel.MotorType.kBrushed)
        self.rr_motor = rev.CANSparkMax(4, rev.CANSparkLowLevel.MotorType.kBrushed)

        l_motor = wpilib.MotorControllerGroup(self.lf_motor, self.lr_motor)
        r_motor = wpilib.MotorControllerGroup(self.rf_motor, self.rr_motor)

        l_motor.setInverted(True)

        self.drive = wpilib.drive.DifferentialDrive(l_motor, r_motor)

        self.climber_motor = rev.CANSparkMax(7, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.claw_motor = rev.CANSparkMax(8, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.feed_motor = rev.CANSparkMax(5, rev.CANSparkLowLevel.MotorType.kBrushed) #bottom wheel
        self.launch_motor = rev.CANSparkMax(6, rev.CANSparkLowLevel.MotorType.kBrushed) #top wheel

        # If launch and feeder wheels are spinning wrong direction
        self.feed_motor.setInverted(True)
        self.launch_motor.setInverted(True)

        # Apply current to launching mechanism
        self.feed_motor.setSmartCurrentLimit(60)
        self.launch_motor.setSmartCurrentLimit(60)

        # Inverting and applying current limit to roller claw and climber
        self.claw_motor.setInverted(False)
        self.climber_motor.setInverted(False)

        self.claw_motor.setSmartCurrentLimit(60)
        self.climber_motor.setSmartCurrentLimit(60)

        # Brake mode best for these motors and their use
        self.claw_motor.setIdleMode(1)      # kBrake
        self.climber_motor.setIdleMode(1)   # kBrake

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)


    def autonomousInit(self):
        """Called when autonomous mode is enabled"""

        self.timer = wpilib.Timer()
        self.timer.start()

    def autonomousPeriodic(self):
        """Movement during autonomous mode"""
        #if self.timer.get() < 1.0:
        #    self.drive.arcadeDrive(1.0, 1.0)
        #else:
        #    self.drive.arcadeDrive(0, 0)
        ''''''

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""
        ''' #TEST THIS LATER (MOVEMENT ADJUSTMENTS)
        #drive motors
        RightY = self.joystick.getRightY()
        LeftY = self.joystick.getLeftY()

        #exponential movement
        if(RightY < 0):
            RightY = (RightY**4)*-1
        else:
            RightY = (RightY**4)
        
        if(LeftY < 0):
            LeftY = (LeftY**4)*-1
        else:
            LeftY = (LeftY**4)

        #this makes it turn slower
        if((RightY < 0.1 and RightY > -0.1) and (LeftY > 0.5 or LeftY < -0.5)):
            LeftY = LeftY * 0.66
        elif((LeftY < 0.1 and LeftY > -0.1) and (RightY > 0.5 or RightY < -0.5)):
            RightY = RightY * 0.66
        
        self.drive.tankDrive(RightY, LeftY)
        '''

        # LAUNCHER WHEEL CONTROL
        # Spins up the launcher wheel
        if (self.joystick.getRawButton(kRB)):
            self.launch_motor.set(1)
        elif (self.joystick.getRawButtonReleased(kRB)):
            self.launch_motor.set(0)

        # FEEDER WHEEL CONTROL
        # Spins feeder wheel, wait for launch wheel to spin up to full speed for best results
        if (self.joystick.getRawButton(kLB)):
            self.feed_motor.set(1)
        elif (self.joystick.getRawButtonReleased(kLB)):
            self.feed_motor.set(0)

        # INTAKE NOTE
        # While the button is being held spin both motors to intake note
        if (self.joystick.getRawButton(kY)):
            self.launch_motor.set(-1)
            self.feed_motor.set(1)
        elif (self.joystick.getRawButtonReleased(kY)):
            self.launch_motor.set(0)
            self.feed_motor.set(0)

        # amp button?
        # While amp button is being held, spin both motors to "spit" the note
        # out at a lower speed into the amp
        if (self.joystick.getRawButton(kX)):
            self.launch_motor.set(.4)
            self.feed_motor.set(.17)
        elif (self.joystick.getRawButtonReleased(kX)):
            self.launch_motor.set(0)
            self.feed_motor.set(0)

        # ROLLER CLAW CONTROL
        # Hold one of the two buttons to injest or expel note from roller claw
        # One button is positive claw power the other negative
        if (self.joystick.getRawButton(kA)):
            self.claw_motor.set(.5)
        elif (self.joystick.getRawButton(kB)):
            self.claw_motor.set(-.5)
        else:
            self.claw_motor.set(0)

        # CLIMBER MOTOR CONTROL
        # POV is D-Pad on controller, 0 == UP   180 == DOWN
        if (self.joystick.getPOV() == 0):
            self.climber_motor.set(1)
        elif (self.joystick.getPOV() == 180):
            self.climber_motor.set(-1)
        else:
            self.climber_motor.set(0)

    def disabledInit(self) -> None:
        # This just makes sure that our simulation code knows that the motor is off
        self.lf_motor.set(0)
        self.lr_motor.set(0)
        self.rf_motor.set(0)
        self.rr_motor.set(0)
        self.climber_motor.set(0)
        self.claw_motor.set(0)
        self.feed_motor.set(0)
        self.launch_motor.set(0)