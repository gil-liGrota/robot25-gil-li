package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.ALGAE_OUTTAKE_DRIVE_BACK_SPEED;
import static frc.robot.subsystems.drive.DriveConstants.KD_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.KD_XY;
import static frc.robot.subsystems.drive.DriveConstants.KI_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.KI_XY;
import static frc.robot.subsystems.drive.DriveConstants.KP_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.KP_XY;
import static frc.robot.subsystems.drive.DriveConstants.MAX_ACCELERATION_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.MAX_ACCELERATION_XY;
import static frc.robot.subsystems.drive.DriveConstants.MAX_VELOCETY_OMEGA;
import static frc.robot.subsystems.drive.DriveConstants.MAX_VELOCETY_XY;
import static frc.robot.subsystems.drive.DriveConstants.OMEGA_TOLERANCE;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_TOLERANCE;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.VisionConstants;
// import frc.robot.Constants.VisionConstants;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.FieldConstants;
import frc.robot.subsystems.drive.Swerve;

public class SwerveCommands {
        private static final double DEADBAND = 0.15;
        private static final double ANGLE_KP = 5.0;
        private static final double ANGLE_KD = 0.4;
        private static final double ANGLE_MAX_VELOCITY = 8.0;
        private static final double ANGLE_MAX_ACCELERATION = 20.0;
        private static final double TOLERANCE = 0.08;
        private static final double FF_START_DELAY = 2.0; // Secs
        private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
        private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
        private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

        private SwerveCommands() {
        }

        private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
                // Apply deadband
                double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
                Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

                // Square magnitude for more precise control
                linearMagnitude = linearMagnitude * linearMagnitude;

                // Return new linear velocity
                return new Pose2d(new Translation2d(), linearDirection)
                                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                .getTranslation();
        }

        /**
         * Rotates the robot to a desired angle using PID control.
         *
         * @param drive       The drive subsystem.
         * @param targetAngle The target angle.
         * @return The command that will rotate the robot to the target angle.
         */
        public static Command rotateToAngle(Swerve drive, Supplier<Rotation2d> targetAngle) {
                return joystickDriveAtAngle(drive, () -> 0, () -> 0, targetAngle);
        }

        public static Command rotateByAngle(Swerve drive, Rotation2d byAngle) {
                ProfiledPIDController angleController = new ProfiledPIDController(
                                ANGLE_KP,
                                0.0,
                                ANGLE_KD,
                                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
                angleController.enableContinuousInput(-Math.PI, Math.PI);
                angleController.setTolerance(TOLERANCE);
                return new Command() {
                        Rotation2d target = new Rotation2d();

                        @Override
                        public void initialize() {
                                target = drive.getRotation().plus(byAngle);
                                angleController.reset(drive.getRotation().getRadians());
                        }

                        @Override
                        public void execute() {
                                // Calculate angular speed
                                double omega = angleController.calculate(drive.getRotation().getRadians(),
                                                target.getRadians());

                                // Convert to field relative speeds & send command
                                ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);
                                drive.runVelocity(speeds, true);
                        }

                        @Override
                        public void end(boolean interrupted) {
                                drive.runVelocity(new ChassisSpeeds(0, 0, 0), true);
                        }

                        @Override
                        public boolean isFinished() {
                                return angleController.atSetpoint();
                        }
                };
        }

        /**
         * Field relative drive command using two joysticks (controlling linear and
         * angular velocities).
         */
        public static Command joystickDrive(
                        Swerve drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier) {
                return Commands.run(
                                () -> {

                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Apply rotation deadband
                                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                                        // if (linearVelocity.getNorm() > 0.3) {
                                        // omega *= 1.3;
                                        // }

                                        // Square rotation value for more precise control
                                        omega = Math.copySign(omega * omega, omega);

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega * drive.getMaxAngularSpeedRadPerSec());
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                                        speeds,
                                                        isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                                        : drive.getRotation());

                                        // speeds = new ChassisSpeeds(0, 0.2, 0);
                                        drive.runVelocity(speeds, true);
                                },
                                drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
        }

        // public static Command resetPosition(Swerve drive) {
        // return Commands.runOnce(() -> drive.resetPosition(), drive);
        // }

        public static Command joystickDriveRobotRelative(
                        Swerve drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier) {
                return Commands.runEnd(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Apply rotation deadband
                                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                                        // Square rotation value for more precise control
                                        omega = Math.copySign(omega * omega, omega);

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega * drive.getMaxAngularSpeedRadPerSec());
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(speeds, true);
                                },
                                () -> drive.runVelocity(new ChassisSpeeds(), true),
                                drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
        }

        public static Command joystickDriveClosedLoopVel(
                        Swerve drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
                        DoubleSupplier omegaSupplier) {
                return Commands.run(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Apply rotation deadband
                                        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                                        // Square rotation value for more precise control
                                        // omega = Math.copySign(omega * omega, omega);

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega * drive.getMaxAngularSpeedRadPerSec());
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                                        speeds,
                                                        isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                                        : drive.getRotation());
                                        drive.runVelocity(speeds, false);
                                },
                                drive).beforeStarting(Commands.runOnce(drive::resetKinematics, drive));
        }

        /**
         * Field relative drive command using joystick for linear control and PID for
         * angular control.
         * Possible use cases include snapping to an angle, aiming at a vision target,
         * or controlling
         * absolute rotation with a joystick.
         */
        public static Command joystickDriveAtAngle(
                        Swerve drive,
                        DoubleSupplier xSupplier,
                        DoubleSupplier ySupplier,
                        Supplier<Rotation2d> rotationSupplier) {

                // Create PID controller
                ProfiledPIDController angleController = new ProfiledPIDController(
                                ANGLE_KP,
                                0.0,
                                ANGLE_KD,
                                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
                angleController.enableContinuousInput(-Math.PI, Math.PI);
                angleController.setTolerance(TOLERANCE);

                // Construct command
                return Commands.run(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Calculate angular speed
                                        double omega = angleController.calculate(
                                                        drive.getRotation().getRadians(),
                                                        rotationSupplier.get().getRadians());

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega);
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
                                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                        : drive.getRotation()), true);
                                },
                                drive)

                                // Reset PID controller when command starts
                                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
        }

        public static Command driveBackSlow(Swerve drive) {
                ChassisSpeeds speeds = new ChassisSpeeds(-ALGAE_OUTTAKE_DRIVE_BACK_SPEED, 0, 0.2);
                return Commands.runEnd(() -> drive.runVelocity(speeds, true), () -> drive.stop(), drive);
        }

        public static Command driveForwardSlowRight(Swerve drive) {
                ChassisSpeeds speeds = new ChassisSpeeds(0.75, 0, 0);
                return Commands.runEnd(() -> drive.runVelocity(speeds, true), () -> drive.stop(), drive);

        }

        public static Command driveForwardSlowLeft(Swerve drive) {
                ChassisSpeeds speeds = new ChassisSpeeds(0.9, 0.4, 0.2);
                return Commands.runEnd(() -> drive.runVelocity(speeds, true), () -> drive.stop(), drive);
        }

        /**
         * Field relative drive command using joystick for linear control and PID for
         * angular control.
         * Possible use cases include snapping to an angle, aiming at a vision target,
         * or controlling
         * absolute rotation with a joystick. pass 4 booleans to set the robot angle to
         * the front, back,
         * left, or right
         */
        public static Command joystickDriveFixedAngles(
                        Swerve drive,
                        DoubleSupplier xSupplier,
                        DoubleSupplier ySupplier,
                        BooleanSupplier front,
                        BooleanSupplier back,
                        BooleanSupplier left,
                        BooleanSupplier right) {

                // Create PID controller
                ProfiledPIDController angleController = new ProfiledPIDController(
                                ANGLE_KP,
                                0.0,
                                ANGLE_KD,
                                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
                angleController.enableContinuousInput(-Math.PI, Math.PI);
                angleController.setTolerance(TOLERANCE);

                Rotation2d[] rot = new Rotation2d[] { new Rotation2d(0) }; // Array for accessing in lambda
                // Construct command
                return Commands.run(
                                () -> { // TODO: verify angles, maybe need to rotate half pi
                                        if (front.getAsBoolean())
                                                rot[0] = new Rotation2d(0.0);
                                        else if (back.getAsBoolean())
                                                rot[0] = new Rotation2d(Math.PI);
                                        else if (left.getAsBoolean())
                                                rot[0] = new Rotation2d(Math.PI / 2.0);
                                        else if (right.getAsBoolean())
                                                rot[0] = new Rotation2d(-Math.PI / 2.0);
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        // Calculate angular speed
                                        double omega = angleController.calculate(drive.getRotation().getRadians(),
                                                        rot[0].getRadians());

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega);
                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
                                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                        : drive.getRotation()), true);
                                },
                                drive)
                                // Reset PID controller when command starts
                                .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
        }

        /**
         * Field relative drive command using joystick for linear control and PID for
         * angular control.
         * Possible use cases include snapping to an angle, aiming at a vision target,
         * or controlling
         * absolute rotation with a joystick.
         */
        public static Command joystickDrivePidAngle(
                        Swerve drive,
                        DoubleSupplier xSupplier,
                        DoubleSupplier ySupplier,
                        DoubleSupplier directionX,
                        DoubleSupplier directionY) {

                // Create PID controller
                ProfiledPIDController angleController = new ProfiledPIDController(
                                ANGLE_KP,
                                0.0,
                                ANGLE_KD,
                                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
                angleController.enableContinuousInput(-Math.PI, Math.PI);
                angleController.setTolerance(TOLERANCE);

                Rotation2d[] lastAngle = new Rotation2d[] { drive.getRotation() };

                // Construct command
                return Commands.run(
                                () -> {
                                        // Get linear velocity
                                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(
                                                        xSupplier.getAsDouble(),
                                                        ySupplier.getAsDouble());

                                        double x = MathUtil.applyDeadband(directionX.getAsDouble(), DEADBAND);
                                        double y = MathUtil.applyDeadband(directionX.getAsDouble(), DEADBAND);
                                        Rotation2d direction = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND) > 0
                                                        ? new Rotation2d(x, y)
                                                        : lastAngle[0];

                                        // Calculate angular speed
                                        double omega = angleController.calculate(
                                                        drive.getRotation().getRadians(), direction.getRadians());

                                        // Convert to field relative speeds & send command
                                        ChassisSpeeds speeds = new ChassisSpeeds(
                                                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                                        omega);

                                        boolean isFlipped = DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get() == Alliance.Red;
                                        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped
                                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                                        : drive.getRotation()), true);
                                        lastAngle[0] = direction;

                                },
                                drive)

                                // Reset PID controller when command starts
                                .beforeStarting(() -> {
                                        angleController.reset(drive.getRotation().getRadians());
                                        lastAngle[0] = drive.getRotation();
                                });
        }

        /**
         * Measures the velocity feedforward constants for the drive motors.
         *
         * <p>
         * This command should only be used in voltage control mode.
         */
        public static Command feedforwardCharacterization(Swerve drive) {
                List<Double> velocitySamples = new LinkedList<>();
                List<Double> voltageSamples = new LinkedList<>();
                Timer timer = new Timer();

                return Commands.sequence(
                                // Reset data
                                Commands.runOnce(
                                                () -> {
                                                        velocitySamples.clear();
                                                        voltageSamples.clear();
                                                }),

                                // Allow modules to orient
                                Commands.run(
                                                () -> {
                                                        drive.runCharacterization(0.0);
                                                },
                                                drive)
                                                .withTimeout(FF_START_DELAY),

                                // Start timer
                                Commands.runOnce(timer::restart),

                                // Accelerate and gather data
                                Commands.run(
                                                () -> {
                                                        double voltage = timer.get() * FF_RAMP_RATE;
                                                        drive.runCharacterization(voltage);
                                                        velocitySamples.add(drive.getFFCharacterizationVelocity());
                                                        voltageSamples.add(voltage);
                                                },
                                                drive)

                                                // When cancelled, calculate and print results
                                                .finallyDo(
                                                                () -> {
                                                                        int n = velocitySamples.size();
                                                                        double sumX = 0.0;
                                                                        double sumY = 0.0;
                                                                        double sumXY = 0.0;
                                                                        double sumX2 = 0.0;
                                                                        for (int i = 0; i < n; i++) {
                                                                                sumX += velocitySamples.get(i);
                                                                                sumY += voltageSamples.get(i);
                                                                                sumXY += velocitySamples.get(i)
                                                                                                * voltageSamples.get(i);
                                                                                sumX2 += velocitySamples.get(i)
                                                                                                * velocitySamples
                                                                                                                .get(i);
                                                                        }
                                                                        double kS = (sumY * sumX2 - sumX * sumXY)
                                                                                        / (n * sumX2 - sumX * sumX);
                                                                        double kV = (n * sumXY - sumX * sumY)
                                                                                        / (n * sumX2 - sumX * sumX);

                                                                        NumberFormat formatter = new DecimalFormat(
                                                                                        "#0.00000");
                                                                        // System.out.println(
                                                                        // "********** Drive FF Characterization Results
                                                                        // **********");
                                                                        // System.out.println("\tkS: "
                                                                        // + formatter.format(kS));
                                                                        // System.out.println("\tkV: "
                                                                        // + formatter.format(kV));
                                                                }));
        }

        /** Measures the robot's wheel radius by spinning in a circle. */
        public static Command wheelRadiusCharacterization(Swerve drive) {
                SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
                WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

                return Commands.parallel(
                                // Drive control sequence
                                Commands.sequence(
                                                // Reset acceleration limiter
                                                Commands.runOnce(
                                                                () -> {
                                                                        limiter.reset(0.0);
                                                                }),

                                                // Turn in place, accelerating up to full speed
                                                Commands.run(
                                                                () -> {
                                                                        double speed = limiter.calculate(
                                                                                        WHEEL_RADIUS_MAX_VELOCITY);
                                                                        drive.runVelocity(new ChassisSpeeds(0.0, 0.0,
                                                                                        speed), true);
                                                                },
                                                                drive)),

                                // Measurement sequence
                                Commands.sequence(
                                                // Wait for modules to fully orient before starting measurement
                                                Commands.waitSeconds(1.0),

                                                // Record starting measurement
                                                Commands.runOnce(
                                                                () -> {
                                                                        state.positions = drive
                                                                                        .getWheelRadiusCharacterizationPositions();
                                                                        state.lastAngle = drive.getRotation();
                                                                        state.gyroDelta = 0.0;
                                                                }),

                                                // Update gyro delta
                                                Commands.run(
                                                                () -> {
                                                                        var rotation = drive.getRotation();
                                                                        state.gyroDelta += Math.abs(
                                                                                        rotation.minus(state.lastAngle)
                                                                                                        .getRadians());
                                                                        state.lastAngle = rotation;
                                                                })

                                                                // When cancelled, calculate and print results
                                                                .finallyDo(
                                                                                () -> {
                                                                                        double[] positions = drive
                                                                                                        .getWheelRadiusCharacterizationPositions();
                                                                                        double wheelDelta = 0.0;
                                                                                        for (int i = 0; i < 4; i++) {
                                                                                                wheelDelta += Math.abs(
                                                                                                                positions[i] - state.positions[i])
                                                                                                                / 4.0;
                                                                                        }
                                                                                        double wheelRadius = (state.gyroDelta
                                                                                                        * DriveConstants.driveBaseRadius)
                                                                                                        / wheelDelta;

                                                                                        NumberFormat formatter = new DecimalFormat(
                                                                                                        "#0.000");
                                                                                        // System.out.println(
                                                                                        // "********** Wheel Radius
                                                                                        // Characterization Results
                                                                                        // **********");
                                                                                        // System.out.println(
                                                                                        // "\tWheel Delta: "
                                                                                        // + formatter.format(
                                                                                        // wheelDelta)
                                                                                        // + " radians");
                                                                                        // System.out.println(
                                                                                        // "\tGyro Delta: " + formatter
                                                                                        // .format(state.gyroDelta)
                                                                                        // + " radians");
                                                                                        // System.out.println(
                                                                                        // "\tWheel Radius: "
                                                                                        // + formatter.format(
                                                                                        // wheelRadius)
                                                                                        // + " meters, "
                                                                                        // + formatter.format(
                                                                                        // Units.metersToInches(
                                                                                        // wheelRadius))
                                                                                        // + " inches");
                                                                                })));
        }

        private static class WheelRadiusCharacterizationState {
                double[] positions = new double[4];
                Rotation2d lastAngle = new Rotation2d();
                double gyroDelta = 0.0;
        }

        public static class LocateToReefCommand extends Command {
                Swerve drive;
                boolean toLeft;
                CommandPS5Controller controller;

                public LocateToReefCommand(Swerve drive, CommandPS5Controller controller, boolean toLeft) {
                        this.drive = drive;
                        this.controller = controller;
                        this.toLeft = toLeft;
                        addRequirements(drive);
                }

                @Override
                public void initialize() {
                        Pose2d destination;
                        try {
                                destination = getClosestReef(drive.getPose(), toLeft);
                        } catch (Exception e) {
                                return;
                        }
                        // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                        // drive.getPose(),
                        // destination);
                        // PathPlannerPath path = new PathPlannerPath(waypoints,
                        // new PathConstraints(maxSpeedMetersPerSec, maxAccMetersPerSecSquared,
                        // maxSpeedRadiansPerSec,
                        // maxAccRadiansPerSecSquared),
                        // null,
                        // new GoalEndState(0, destination.getRotation()));
                        // path.preventFlipping = true;
                        // Logger.recordOutput("current reef destination", destination);
                        // Command cmd = AutoBuilder.followPath(path)
                        // .until(() ->
                        // drive.getPose().getTranslation().getDistance(destination.getTranslation()) <
                        // 0.6);

                        // cmd = cmd.andThen(new DriveToPosition(drive, destination));
                        // cmd.schedule();

                        new DriveToPosition(drive, destination)
                                        .andThen(joystickDriveRobotRelative(drive, () -> 0.35, () -> 0, () -> 0)
                                                        .withTimeout(0.3))
                                        // .raceWith(Commands
                                        // .runEnd(() -> controller.vibrate(0.2),
                                        // () -> controller.vibrate(0))
                                        // .withTimeout(0.3)))
                                        .schedule();
                }

                @Override
                public boolean isFinished() {
                        return true;
                }

                public Pose2d getClosestReef(Pose2d currentPose, boolean toLeft) throws Exception {
                        Pose2d[] branches = /*
                                             * DriverStation.getAlliance().orElseGet(() -> Alliance.Red) == Alliance.Red
                                             */ currentPose
                                        .getX() > FieldConstants.fieldLength / 2
                                                        ? (toLeft ? FieldConstants.Reef.redLeftBranches
                                                                        : FieldConstants.Reef.redRightBranches)
                                                        : (toLeft ? FieldConstants.Reef.blueLeftBranches
                                                                        : FieldConstants.Reef.blueRightBranches);
                        // Get the closest reef to the robot
                        double minDistance = Double.MAX_VALUE;
                        Pose2d closestReef = FieldConstants.Reef.blueCenterFaces[0];
                        for (int i = 0; i < FieldConstants.Reef.blueCenterFaces.length; i++) {
                                double distance = currentPose.getTranslation()
                                                .getDistance(branches[i].getTranslation());
                                if (distance < minDistance) {
                                        minDistance = distance;
                                        closestReef = branches[i];
                                }
                        }
                        double allowedDist = 2.5;
                        if (minDistance > allowedDist) {
                                throw new Exception("No reef is close enough");
                        }
                        return closestReef;
                }

        }

        public static class LocateToReefAlgaeOuttakeCommand extends Command {
                Swerve drive;
                PomXboxController controller;

                public LocateToReefAlgaeOuttakeCommand(Swerve drive, PomXboxController controller) {
                        this.drive = drive;
                        this.controller = controller;
                        addRequirements(drive);
                }

                @Override
                public void initialize() {
                        Pose2d destination;
                        try {
                                destination = getClosestReef(drive.getPose());
                        } catch (Exception e) {
                                return;
                        }
                        // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                        // drive.getPose(),
                        // destination);
                        // PathPlannerPath path = new PathPlannerPath(waypoints,
                        // new PathConstraints(maxSpeedMetersPerSec, maxAccMetersPerSecSquared,
                        // maxSpeedRadiansPerSec,
                        // maxAccRadiansPerSecSquared),
                        // null,
                        // new GoalEndState(0, destination.getRotation()));
                        // path.preventFlipping = true;
                        // Logger.recordOutput("current reef destination", destination);
                        // Command cmd = AutoBuilder.followPath(path)
                        // .until(() ->
                        // drive.getPose().getTranslation().getDistance(destination.getTranslation()) <
                        // 0.6);

                        // cmd = cmd.andThen(new DriveToPosition(drive, destination));
                        // cmd.schedule();

                        new DriveToPosition(drive, destination)
                                        .andThen(joystickDriveRobotRelative(drive, () -> 0.35, () -> 0, () -> 0)
                                                        .withTimeout(0.3))
                                        .schedule();
                }

                @Override
                public boolean isFinished() {
                        return true;
                }

                public Pose2d getClosestReef(Pose2d currentPose) throws Exception {
                        Pose2d[] branches = /*
                                             * DriverStation.getAlliance().orElseGet(() -> Alliance.Red) == Alliance.Red
                                             */ currentPose
                                        .getX() > FieldConstants.fieldLength / 2
                                                        ? (FieldConstants.Reef.redCenterFaces)
                                                        : (FieldConstants.Reef.blueCenterFaces);
                        // Get the closest reef to the robot
                        double minDistance = Double.MAX_VALUE;
                        Pose2d closestReef = FieldConstants.Reef.blueCenterFaces[0];
                        for (int i = 0; i < FieldConstants.Reef.blueCenterFaces.length; i++) {
                                double distance = currentPose.getTranslation()
                                                .getDistance(branches[i].getTranslation());
                                if (distance < minDistance) {
                                        minDistance = distance;
                                        closestReef = branches[i];
                                }
                        }
                        double allowedDist = 2.5;
                        if (minDistance > allowedDist) {
                                throw new Exception("No reef is close enough");
                        }
                        return closestReef;
                }

        }

        public static Command locateToReefCommand(Swerve drive, CommandPS5Controller controller, boolean toLeft) {
                return new LocateToReefCommand(drive, controller, toLeft);
        }

        public static Command stopDrive(Swerve m_drive) {
                return Commands.runOnce(() -> m_drive.stop(), m_drive);
        }

        public static class DriveToPosition extends Command {
                private final Swerve m_drive;
                private final Pose2d m_target;
                private final ProfiledPIDController m_controllerX;
                private final ProfiledPIDController m_controllerY;
                private final ProfiledPIDController m_controllerTheta;
                private final Timer m_timer = new Timer();

                LoggedNetworkNumber kpTune = new LoggedNetworkNumber("tunes/translation kp", KP_XY);
                LoggedNetworkNumber kiTune = new LoggedNetworkNumber("tunes/translation ki", KI_XY);
                LoggedNetworkNumber kdTune = new LoggedNetworkNumber("tunes/translation kd", KD_XY);
                LoggedNetworkNumber maxVelocityTune = new LoggedNetworkNumber("tunes/translation max velocity",
                                MAX_VELOCETY_XY);
                LoggedNetworkNumber maxAccelerationTune = new LoggedNetworkNumber("tunes/translation max acceleration",
                                MAX_ACCELERATION_XY);

                LoggedNetworkNumber kpThetaTune = new LoggedNetworkNumber("tunes/rotation kp", KP_OMEGA);
                LoggedNetworkNumber kiThetaTune = new LoggedNetworkNumber("tunes/rotation ki", KI_OMEGA);
                LoggedNetworkNumber kdThetaTune = new LoggedNetworkNumber("tunes/rotation kd", KD_OMEGA);
                LoggedNetworkNumber maxVelocityThetaTune = new LoggedNetworkNumber("tunes/rotation max velocity",
                                MAX_VELOCETY_OMEGA);
                LoggedNetworkNumber maxAccelerationThetaTune = new LoggedNetworkNumber(
                                "tunes/rotation max acceleration",
                                MAX_ACCELERATION_OMEGA);

                LoggedNetworkNumber translationTolerance = new LoggedNetworkNumber("tunes/translation tolerance",
                                TRANSLATION_TOLERANCE);
                LoggedNetworkNumber omegaTolerance = new LoggedNetworkNumber("tunes/rotation tolerance",
                                OMEGA_TOLERANCE);

                public DriveToPosition(Swerve drive, Pose2d target) {
                        m_drive = drive;
                        m_target = target;
                        m_controllerX = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
                        m_controllerX.setTolerance(TRANSLATION_TOLERANCE);
                        m_controllerY = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
                        m_controllerY.setTolerance(TRANSLATION_TOLERANCE);
                        m_controllerTheta = new ProfiledPIDController(KP_OMEGA, KI_OMEGA, KD_OMEGA,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_OMEGA, MAX_ACCELERATION_OMEGA));
                        m_controllerTheta.setTolerance(OMEGA_TOLERANCE);
                        m_controllerTheta.enableContinuousInput(-Math.PI, Math.PI);
                        addRequirements(drive);
                }

                @Override
                public void initialize() {
                        Logger.recordOutput("pid target", m_target);
                        m_timer.reset();
                        m_timer.start();
                        var currPose = m_drive.getPose();
                        ChassisSpeeds currS = m_drive.getChassisSpeeds();
                        m_controllerX.reset(currPose.getX(), currS.vxMetersPerSecond);
                        m_controllerY.reset(currPose.getY(), currS.vyMetersPerSecond);
                        m_controllerTheta.reset(currPose.getRotation().getRadians(), currS.omegaRadiansPerSecond);
                }

                @Override
                public void execute() {
                        m_controllerX.setPID(kpTune.get(), kiTune.get(), kdTune.get());
                        m_controllerX
                                        .setConstraints(new TrapezoidProfile.Constraints(maxVelocityTune.get(),
                                                        maxAccelerationTune.get()));
                        m_controllerY.setPID(kpTune.get(), kiTune.get(), kdTune.get());
                        m_controllerY
                                        .setConstraints(new TrapezoidProfile.Constraints(maxVelocityTune.get(),
                                                        maxAccelerationTune.get()));
                        m_controllerTheta.setPID(kpThetaTune.get(), kiThetaTune.get(), kdThetaTune.get());
                        m_controllerTheta.setConstraints(
                                        new TrapezoidProfile.Constraints(maxVelocityThetaTune.get(),
                                                        maxAccelerationThetaTune.get()));

                        var pose = m_drive.getPose();
                        var chassisSpeeds = new ChassisSpeeds(
                                        m_controllerX.calculate(pose.getTranslation().getX(),
                                                        m_target.getTranslation().getX()),
                                        m_controllerY.calculate(pose.getTranslation().getY(),
                                                        m_target.getTranslation().getY()),
                                        m_controllerTheta.calculate(pose.getRotation().getRadians(),
                                                        m_target.getRotation().getRadians()));

                        // if ((pose.getRotation().getDegrees() % 360 + 470) % 360 > 180) {
                        // chassisSpeeds = new ChassisSpeeds(-chassisSpeeds.vxMetersPerSecond,
                        // -chassisSpeeds.vyMetersPerSecond,
                        // chassisSpeeds.omegaRadiansPerSecond);
                        // }
                        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds,
                                        pose.getRotation().unaryMinus());
                        Logger.recordOutput("Requested speeds", chassisSpeeds);
                        // Logger.recordOutput("pose", pose);
                        // Logger.recordOutput("pose to", m_target);
                        Logger.recordOutput("error x", m_target.getTranslation().getX() - pose.getTranslation().getX());
                        Logger.recordOutput("error y", m_target.getTranslation().getY() - pose.getTranslation().getY());
                        m_drive.runVelocity(chassisSpeeds, false);
                }

                @Override
                public void end(boolean interrupted) {
                        m_drive.stop();
                }

                @Override
                public boolean isFinished() {
                        // return m_timer.hasElapsed(5) ||
                        return (m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerTheta.atGoal());
                }
        }

        public static class DriveToReef extends Command {
                private final Swerve m_drive;
                VisionSubsystem vision;
                int camera;
                private Transform2d m_target;
                private final ProfiledPIDController m_controllerX;
                private final ProfiledPIDController m_controllerY;
                private final ProfiledPIDController m_controllerTheta;
                private final Timer m_timer = new Timer();

                LoggedNetworkNumber kpTune = new LoggedNetworkNumber("tunes/translation kp", KP_XY);
                LoggedNetworkNumber kiTune = new LoggedNetworkNumber("tunes/translation ki", KI_XY);
                LoggedNetworkNumber kdTune = new LoggedNetworkNumber("tunes/translation kd", KD_XY);
                LoggedNetworkNumber maxVelocityTune = new LoggedNetworkNumber("tunes/translation max velocity",
                                MAX_VELOCETY_XY);
                LoggedNetworkNumber maxAccelerationTune = new LoggedNetworkNumber("tunes/translation max acceleration",
                                MAX_ACCELERATION_XY);

                LoggedNetworkNumber kpThetaTune = new LoggedNetworkNumber("tunes/rotation kp", KP_OMEGA);
                LoggedNetworkNumber kiThetaTune = new LoggedNetworkNumber("tunes/rotation ki", KI_OMEGA);
                LoggedNetworkNumber kdThetaTune = new LoggedNetworkNumber("tunes/rotation kd", KD_OMEGA);
                LoggedNetworkNumber maxVelocityThetaTune = new LoggedNetworkNumber("tunes/rotation max velocity",
                                MAX_VELOCETY_OMEGA);
                LoggedNetworkNumber maxAccelerationThetaTune = new LoggedNetworkNumber(
                                "tunes/rotation max acceleration",
                                MAX_ACCELERATION_OMEGA);

                LoggedNetworkNumber translationTolerance = new LoggedNetworkNumber("tunes/translation tolerance",
                                TRANSLATION_TOLERANCE);
                LoggedNetworkNumber omegaTolerance = new LoggedNetworkNumber("tunes/rotation tolerance",
                                OMEGA_TOLERANCE);

                public DriveToReef(Swerve drive, VisionSubsystem vision, boolean left) {
                        m_drive = drive;
                        this.vision = vision;
                        camera = left ? 0 : 1;
                        m_target = left ? VisionConstants.transformLeftBranch : VisionConstants.transformRightBranch;
                        m_controllerX = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
                        m_controllerX.setTolerance(TRANSLATION_TOLERANCE);
                        m_controllerY = new ProfiledPIDController(KP_XY, KI_XY, KD_XY,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_XY, MAX_ACCELERATION_XY));
                        m_controllerY.setTolerance(TRANSLATION_TOLERANCE);
                        m_controllerTheta = new ProfiledPIDController(KP_OMEGA, KI_OMEGA, KD_OMEGA,
                                        new TrapezoidProfile.Constraints(MAX_VELOCETY_OMEGA, MAX_ACCELERATION_OMEGA));
                        m_controllerTheta.setTolerance(OMEGA_TOLERANCE);
                        m_controllerTheta.enableContinuousInput(-Math.PI, Math.PI);
                        addRequirements(drive);
                }

                @Override
                public void initialize() {
                        m_timer.reset();
                        m_timer.start();
                        // var currPose = m_drive.getPose();
                        var transform = vision.getBestTarget(camera);
                        // if (transform != null) {
                        // m_target = currPose
                        // .transformBy(new Transform2d(transform.getX(), transform.getY(),
                        // transform.getRotation().toRotation2d()));
                        // } else {
                        // m_target = currPose;
                        // }
                        ChassisSpeeds currS = m_drive.getChassisSpeeds();
                        m_controllerX.reset(transform.getX(), currS.vxMetersPerSecond);
                        m_controllerY.reset(transform.getY(), currS.vyMetersPerSecond);
                        m_controllerTheta.reset(transform.getRotation().toRotation2d().getRadians(),
                                        currS.omegaRadiansPerSecond);
                }

                @Override
                public void execute() {
                        m_controllerX.setPID(kpTune.get(), kiTune.get(), kdTune.get());
                        m_controllerX
                                        .setConstraints(new TrapezoidProfile.Constraints(maxVelocityTune.get(),
                                                        maxAccelerationTune.get()));
                        m_controllerY.setPID(kpTune.get(), kiTune.get(), kdTune.get());
                        m_controllerY
                                        .setConstraints(new TrapezoidProfile.Constraints(maxVelocityTune.get(),
                                                        maxAccelerationTune.get()));
                        m_controllerTheta.setPID(kpThetaTune.get(), kiThetaTune.get(), kdThetaTune.get());
                        m_controllerTheta.setConstraints(
                                        new TrapezoidProfile.Constraints(maxVelocityThetaTune.get(),
                                                        maxAccelerationThetaTune.get()));

                        // var pose = m_drive.getPose();
                        var tranform = vision.getBestTarget(camera);
                        if (tranform == null)
                                return;
                        var chassisSpeeds = new ChassisSpeeds(
                                        -m_controllerX.calculate(tranform.getTranslation().getX(),
                                                        m_target.getTranslation().getX()),
                                        -m_controllerY.calculate(tranform.getTranslation().getY(),
                                                        m_target.getTranslation().getY()),
                                        m_controllerTheta.calculate(tranform.getRotation().toRotation2d().getRadians(),
                                                        m_target.getRotation().getRadians()));

                        // if ((pose.getRotation().getDegrees() % 360 + 470) % 360 > 180) {
                        // chassisSpeeds = new ChassisSpeeds(-chassisSpeeds.vxMetersPerSecond,
                        // -chassisSpeeds.vyMetersPerSecond,
                        // chassisSpeeds.omegaRadiansPerSecond);
                        // }
                        // chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds,
                        // tranform.getRotation().toRotation2d().unaryMinus());
                        Logger.recordOutput("Requested speeds", chassisSpeeds);
                        // Logger.recordOutput("pose", pose);
                        // Logger.recordOutput("pose to", m_target);
                        Logger.recordOutput("error x",
                                        m_target.getTranslation().getX() - tranform.getTranslation().getX());
                        Logger.recordOutput("error y", m_target.getTranslation().getY() - tranform
                                        .getTranslation().getY());
                        m_drive.runVelocity(chassisSpeeds, false);
                }

                @Override
                public void end(boolean interrupted) {
                        m_drive.stop();
                }

                @Override
                public boolean isFinished() {
                        // return m_timer.hasElapsed(5) ||
                        return (m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerTheta.atGoal());
                }
        }
}
