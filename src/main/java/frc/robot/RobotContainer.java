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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOPOM;
import frc.robot.subsystems.drive.ModuleIOSim;

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
        private final Drive drive;

        // Controller
        private final PomXboxController driverController = new PomXboxController(0);

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
                                drive = new Drive(
                                                new GyroIOPigeon(),
                                                new ModuleIOPOM(0),
                                                new ModuleIOPOM(1),
                                                new ModuleIOPOM(2),
                                                new ModuleIOPOM(3));
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations

                                driveSimulation = new SwerveDriveSimulation(Drive.maplesimConfig,
                                                new Pose2d(3, 3, new Rotation2d()));
                                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                                drive = new Drive(
                                                new GyroIOSim(this.driveSimulation.getGyroSimulation()),
                                                new ModuleIOSim(this.driveSimulation.getModules()[0]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[1]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[2]),
                                                new ModuleIOSim(this.driveSimulation.getModules()[3]));
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                break;
                }

                SendableChooser<Command> c = new SendableChooser<>();

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", c); // TODO use auto builder

                // Set up SysId routines
                autoChooser.addOption(
                                "Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                autoChooser.addOption(
                                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Steer Forward)",
                                drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Steer Reverse)",
                                drive.sysIdSteerQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Steer Forward)",
                                drive.sysIdSteerDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Steer Reverse)",
                                drive.sysIdSteerDynamic(SysIdRoutine.Direction.kReverse));

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
                // Default command, normal field-relative drive
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> driverController.getLeftY() * 0.27,
                                                () -> driverController.getLeftX() * 0.27,
                                                () -> driverController.getRightX() * 0.23));
                // driverController.x().onTrue(Commands.runOnce(() ->
                // moduleFL.setTurnPosition(new Rotation2d(Math.PI))));
                // driverController.b().onTrue(
                // Commands.runOnce(() -> moduleFL.setTurnPosition(new Rotation2d(1.5 *
                // Math.PI))));
                // driverController.y().whileTrue(Commands.run(() -> moduleFL.setTurnPosition(
                // new Rotation2d(driverController.getLeftX(), driverController.getLeftY()))));

                // drive.setDefaultCommand(drive.testSteeringCommand(driverController::getLeftX,
                // driverController::getLeftY));
                // driverController.PovUp().whileTrue(drive.testSteeringCommand(() -> 0, () ->
                // 1));
                // driverController.PovLeft().whileTrue(drive.testSteeringCommand(() -> 1, () ->
                // 0));

                // drive.setDefaultCommand(drive.testSteeringAngleCommand(Rotation2d.fromDegrees(30)));

                // ])));

                // Lock to 0° when A button is held
                driverController
                                .a()
                                .whileTrue(
                                                DriveCommands.joystickDriveAtAngle(
                                                                drive,
                                                                () -> -driverController.getLeftY(),
                                                                () -> -driverController.getLeftX(),
                                                                () -> new Rotation2d()));

                // Switch to X pattern when X button is pressed
                driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

                // Reset gyro to 0° when Y button is pressed
                driverController.y().onTrue(drive.resetGyroCommand());
        }

        public void displaSimFieldToAdvantageScope() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                Logger.recordOutput(
                                "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesArrayByType("Note"));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
