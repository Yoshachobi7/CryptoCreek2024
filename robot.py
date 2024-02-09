#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.drive


class MyRobot(wpilib.TimedRobot):
    """Main robot class"""

    kUpButton = 4 #Y Button
    kLeftButton = 3 #X Button
    kRightButton = 2 #B Button
    kDownButton = 1 #A Button

    def robotInit(self):
        """Robot-wide initialization code should go here"""

        self.stick = wpilib.XboxController(0)

        self.lf_motor = wpilib.PWMSparkMax(1)
        self.lr_motor = wpilib.PWMSparkMax(2)
        self.rf_motor = wpilib.PWMSparkMax(3)
        self.rr_motor = wpilib.PWMSparkMax(4)

        self.hook_motor = wpilib.PWMSparkMax(5)

        l_motor = wpilib.MotorControllerGroup(self.lf_motor, self.lr_motor)
        r_motor = wpilib.MotorControllerGroup(self.rf_motor, self.rr_motor)

        l_motor.setInverted(True)

        self.drive = wpilib.drive.DifferentialDrive(l_motor, r_motor)

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""

        self.timer = wpilib.Timer()
        self.timer.start()

    def autonomousPeriodic(self):
        if self.timer.get() < 2.0:
            self.drive.arcadeDrive(-1.0, -0.3)
        else:
            self.drive.arcadeDrive(0, 0)

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""
        #drive motors
        RightY = self.stick.getRightY()
        LeftY = self.stick.getLeftY()

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

        #hook motor
        hookup = self.stick.getRawButton(self.kUpButton)
        hookdown = self.stick.getRawButton(self.kDownButton)
        hookmotor = 0
        if(hookup == True):
            hookmotor = 1
        elif(hookdown == True):
            hookmotor = -1
        else:
            hookmotor = 0
        self.hook_motor.set(hookmotor)

