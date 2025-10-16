package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.backLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.backRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.driveEncoderPositionFactor;
import static frc.robot.subsystems.drive.DriveConstants.driveKd;
import static frc.robot.subsystems.drive.DriveConstants.driveKp;
import static frc.robot.subsystems.drive.DriveConstants.driveKs;
import static frc.robot.subsystems.drive.DriveConstants.driveKv;
import static frc.robot.subsystems.drive.DriveConstants.driveRampRate;
import static frc.robot.subsystems.drive.DriveConstants.driveSlipCurrent;
import static frc.robot.subsystems.drive.DriveConstants.frontLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.frontRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.odometryFrequency;
import static frc.robot.subsystems.drive.DriveConstants.swerveBaseID;
import static frc.robot.subsystems.drive.DriveConstants.swerveModuleIDsCount;
import static frc.robot.subsystems.drive.DriveConstants.turnEncoderPositionFactor;
import static frc.robot.subsystems.drive.DriveConstants.turnEncoderVelocityFactor;
import static frc.robot.subsystems.drive.DriveConstants.turnInverted;
import static frc.robot.subsystems.drive.DriveConstants.turnKd;
import static frc.robot.subsystems.drive.DriveConstants.turnKp;
import static frc.robot.subsystems.drive.DriveConstants.turnKs;
import static frc.robot.subsystems.drive.DriveConstants.turnMotorCurrentLimit;
import static frc.robot.subsystems.drive.DriveConstants.turnMotorRampRate;
import static frc.robot.subsystems.drive.DriveConstants.turnPIDMaxInput;
import static frc.robot.subsystems.drive.DriveConstants.turnPIDMinInput;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.POM_lib.Motors.POMTalonFX;
import frc.robot.util.SparkUtil;

public class ModuleIOPOM implements ModuleIO {
  private final int module;

  // Hardware objects
  private final POMTalonFX driveMotor;
  private final SparkMax turnMotor;
  private final CANcoder turnEncoder;
  private final Timer resetToAbsoluteTimer = new Timer();
  // Closed loop controllers
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  public ModuleIOPOM(int module) {
    this.module = module;

    Rotation2d zeroRotation = switch (module) {
      case 0 -> frontLeftZeroRotation;
      case 1 -> frontRightZeroRotation;
      case 2 -> backLeftZeroRotation;
      case 3 -> backRightZeroRotation;
      default -> new Rotation2d();
    };
    driveMotor = new POMTalonFX(swerveBaseID + swerveModuleIDsCount * module);
    turnMotor = new SparkMax(swerveBaseID + 1 + swerveModuleIDsCount * module, MotorType.kBrushless);

    turnEncoder = new CANcoder(swerveBaseID + 2 + swerveModuleIDsCount * module);
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = module == 3 ? SensorDirectionValue.CounterClockwise_Positive
        : SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = zeroRotation.getRotations();
    tryUntilOk(5, () -> turnEncoder.getConfigurator().apply(encoderConfig, 0.25));

    turnController = turnMotor.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    Slot0Configs driveMotorGains = new Slot0Configs()
        .withKP(driveKp).withKD(driveKd).withKS(driveKs).withKV(driveKv);
    driveConfig.Slot0 = driveMotorGains;
    
    driveConfig.Feedback.SensorToMechanismRatio = driveEncoderPositionFactor;
    driveConfig.Feedback.RotorToSensorRatio = 1.0;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveSlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveSlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveSlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = driveRampRate;
    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = driveRampRate;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = driveRampRate;
    tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveMotor.setPosition(0.0, 0.25));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(turnMotorCurrentLimit)
        .voltageCompensation(12.0)
        .closedLoopRampRate(turnMotorRampRate)
        .openLoopRampRate(turnMotorRampRate);
    turnConfig.encoder
        // .inverted(turnEncoderInverted)
        .positionConversionFactor(turnEncoderPositionFactor)
        .velocityConversionFactor(turnEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    turnConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(turnKp, 0.0, turnKd, 0.0)
        .outputRange(-0.25, 0.25);
    turnConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnMotor,
        5,
        () -> turnMotor.configure(
            turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = OdometryThread.getInstance()
        .registerSignal(() -> Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble()));
    turnPositionQueue = OdometryThread.getInstance().registerSignal(turnMotor, turnMotor.getEncoder()::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    var driveStatus = BaseStatusSignal.refreshAll(
        driveMotor.getPosition(),
        driveMotor.getVelocity(),
        driveMotor.getMotorVoltage(),
        driveMotor.getStatorCurrent());
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
    inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = driveMotor.getStatorCurrent().getValueAsDouble();

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnMotor,
        () -> turnMotor.getEncoder().getPosition(),
        (value) -> inputs.turnPosition = new Rotation2d(value));
    ifOk(
        turnMotor,
        () -> turnMotor.getEncoder().getVelocity(),
        (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnMotor,
        new DoubleSupplier[] { turnMotor::getAppliedOutput, turnMotor::getBusVoltage },
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
    inputs.absolutePosition = getAbsolutePosition();

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream()
        .map((Double value) -> new Rotation2d(value))
        .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

    if (resetToAbsoluteTimer.get() > 2) {
      resetToAbsoluteTimer.restart();
      resetToAbsolute();
    }
    if (!resetToAbsoluteTimer.isRunning()) {
      resetToAbsoluteTimer.start();
    }
  }

  public void resetToAbsolute() {
    SparkUtil.tryUntilOk(
        turnMotor,
        5,
        () -> turnMotor
            .getEncoder()
            .setPosition(getAbsolutePosition()));
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnMotor.setVoltage(output);
    Logger.recordOutput("angle output", output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setTurnPosition(Rotation2d setpoint) {
    double error = setpoint.minus(Rotation2d.fromRadians(getAbsolutePosition())).getRadians();
    if (Math.abs(error) >= Math.PI) {
      error -= Math.copySign(Math.PI, error);
      error *= -1;
    }
    var ks = Math.copySign(turnKs, error);
    Logger.recordOutput(getModuleString() + "/ks", ks);
    Logger.recordOutput(getModuleString() + "/error", error);
    if (Math.abs(error) > 0.05) {
      turnController.setReference(setpoint.getRadians(), ControlType.kPosition,
          ClosedLoopSlot.kSlot0, ks, ArbFFUnits.kVoltage);
    } else {
      setTurnOpenLoop(0);
    }
  }

  public double getAbsolutePosition() {
    return turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
  }

  private String getModuleString() {
    return "Module " + switch (module) {
      case 0 -> "FL";
      case 1 -> "FR";
      case 2 -> "BL";
      case 3 -> "BR";
      default -> "Unknown";
    };
  }
}
