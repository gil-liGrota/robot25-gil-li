// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.commands.ledsCommands;
import frc.robot.subsystems.leds.leds;
import frc.robot.subsystems.leds.ledsIOReal;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import frc.robot.subsystems.arm.arm;
import frc.robot.subsystems.arm.armIOReal;
import frc.robot.commands.armcommands;
import frc.robot.POM_lib.sensors.POMDigitalInput;
import frc.robot.commands.elevatorCommands;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.elevator.elevatorIOReal;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems

        // Controller
        private final PomXboxController driverController = new PomXboxController(0);
        private arm arm;
        private POMDigitalInput brakeSwitch = new POMDigitalInput(4);

        private leds leds;
        private elevator elevator;

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private SwerveDriveSimulation driveSimulation = null;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                leds = new leds(new ledsIOReal());
                                arm = new arm(new armIOReal(brakeSwitch));
                                elevator = new elevator(new elevatorIOReal(brakeSwitch));
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                break;
                }

                SendableChooser<Command> c = new SendableChooser<>();

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", c); // TODO use auto builder

                // Set up SysId routines
                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                driverController.leftTrigger().whileTrue(elevatorCommands.closeElevatorManual(elevator));
                driverController.rightTrigger().whileTrue(elevatorCommands.openElevatorManual(elevator));
                driverController.b().onTrue(elevatorCommands.goToPosition(elevator, 10));
                driverController.x().onTrue(elevatorCommands.goToPosition(elevator, 20));
                driverController.y().onTrue(elevatorCommands.goToPosition(elevator, 30));
                driverController.a().onTrue(elevatorCommands.closeElevator(elevator));

        }

        // public void displaSimFieldToAdvantageScope() {
        // if (Constants.currentMode != Constants.Mode.SIM)
        // return;

        // Logger.recordOutput(
        // "FieldSimulation/RobotPosition",
        // driveSimulation.getSimulatedDriveTrainPose());
        // Logger.recordOutput(
        // "FieldSimulation/Notes",
        // SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
        // }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
