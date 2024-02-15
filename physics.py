#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# See the notes for the other physics sample
#
import wpilib
import wpilib.simulation
import wpimath.system.plant

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        # Motors
        self.lf_motor = wpilib.simulation.PWMSim(robot.lf_motor.getChannel())
        # self.lr_motor = wpilib.simulation.PWMSim(2)
        self.rf_motor = wpilib.simulation.PWMSim(robot.rf_motor.getChannel())
        # self.rr_motor = wpilib.simulation.PWMSim(4)

        # Gyro
        self.gyro = wpilib.simulation.AnalogGyroSim(robot.gyro)

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )
        # fmt: on

        # This gearbox represents a gearbox containing 4 Vex 775pro motors.
        self.elevatorGearbox = wpimath.system.plant.DCMotor.vex775Pro(4)

        # Simulation classes help us simulate what's going on, including gravity.
        self.elevatorSim = wpilib.simulation.ElevatorSim(
            self.elevatorGearbox,
            robot.kElevatorGearing,
            robot.kCarriageMass,
            robot.kElevatorDrumRadius,
            robot.kMinElevatorHeight,
            robot.kMaxElevatorHeight,
            True,
            0,
            [0.01],
        )
        self.encoderSim = wpilib.simulation.EncoderSim(robot.encoder)
        self.motorSim = wpilib.simulation.PWMSim(robot.motor.getChannel())

        # Create a Mechanism2d display of an elevator
        self.mech2d = wpilib.Mechanism2d(20, 50)
        self.elevatorRoot = self.mech2d.getRoot("Elevator Root", 10, 0)
        self.elevatorMech2d = self.elevatorRoot.appendLigament(
            "Elevator", self.elevatorSim.getPositionInches(), 90
        )

        # Put Mechanism to SmartDashboard
        wpilib.SmartDashboard.putData("Elevator Sim", self.mech2d)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain (only front motors used because read should be in sync)
        lf_motor = self.lf_motor.getSpeed()
        rf_motor = self.rf_motor.getSpeed()

        transform = self.drivetrain.calculate(lf_motor, rf_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        self.gyro.setAngle(-pose.rotation().degrees())

        # First, we set our "inputs" (voltages)
        self.elevatorSim.setInput(
            0, self.motorSim.getSpeed() * wpilib.RobotController.getInputVoltage()
        )

        # Next, we update it
        self.elevatorSim.update(tm_diff)

        # Finally, we set our simulated encoder's readings and simulated battery
        # voltage
        self.encoderSim.setDistance(self.elevatorSim.getPosition())
        # SimBattery estimates loaded battery voltage
        # wpilib.simulation.RoboRioSim.setVInVoltage(
        #     wpilib.simulation.BatterySim
        # )

        # Update the Elevator length based on the simulated elevator height
        self.elevatorMech2d.setLength(self.elevatorSim.getPositionInches())
