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

        # change to rev.cansparkmax
        # self.launch_motor = wpilib.PWMSparkMax(5)
        # self.feed_motor = wpilib.PWMSparkMax(6)

        # self.climber_motor = wpilib.PWMSparkMax(7)
        # self.claw_motor = wpilib.PWMSparkMax(8)

        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""

        self.timer = wpilib.Timer()
        self.timer.start()

    def autonomousPeriodic(self):
        """auto"""

    def teleopPeriodic(self):
        """Called when operation control mode is enabled"""
        #drive motors
        RightY = self.joystick.getRightY()
        LeftY = self.joystick.getLeftY()

        # # exponential movement
        # if(RightY < 0):
        #     RightY = (RightY**4)*-1
        # else:
        #     RightY = (RightY**4)
        
        # if(LeftY < 0):
        #     LeftY = (LeftY**4)*-1
        # else:
        #     LeftY = (LeftY**4)

        # # this makes it turn slower
        # if((RightY < 0.1 and RightY > -0.1) and (LeftY > 0.5 or LeftY < -0.5)):
        #     LeftY = LeftY * 0.66
        # elif((LeftY < 0.1 and LeftY > -0.1) and (RightY > 0.5 or RightY < -0.5)):
        #     RightY = RightY * 0.66


        self.drive.tankDrive(RightY, LeftY)

        # # launcher wheel
        # if (self.joystick.getRawButton(kRB)):
        #     self.launch_motor.set(1)
        # elif (self.joystick.getRawButtonReleased(kRB)):
        #     self.launch_motor.set(0)

        # # feeder wheel
        # if (self.joystick.getRawButton(kLB)):
        #     self.feed_motor.set(1)
        # elif (self.joystick.getRawButtonReleased(kLB)):
        #     self.feed_motor.set(0)

        # # intake note
        # if (self.joystick.getRawButton(kY)):
        #     self.launch_motor.set(-1)
        #     self.feed_motor.set(1)
        # elif (self.joystick.getRawButtonReleased(kY)):
        #     self.launch_motor.set(0)
        #     self.feed_motor.set(0)

        # # amp button?
        # if (self.joystick.getRawButton(kX)):
        #     self.launch_motor.set(.4)
        #     self.feed_motor.set(.17)
        # elif (self.joystick.getRawButtonReleased(kX)):
        #     self.launch_motor.set(0)
        #     self.feed_motor.set(0)

        # # claw
        # if (self.joystick.getRawButton(kA)):
        #     self.claw_motor.set(.5)
        # elif (self.joystick.getRawButton(kB)):
        #     self.claw_motor.set(-.5)
        # else:
        #     self.claw_motor.set(0)

        # # hook motor
        # if (self.joystick.getPOV() == 0):
        #     self.climber_motor.set(1)
        # elif (self.joystick.getPOV() == 180):
        #     self.climber_motor.set(-1)
        # else:
        #     self.climber_motor.set(0)