package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.elevatorConstance.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class elevatorIOReal implements elevatorIO {
    POMSparkMax motor;
    RelativeEncoder encoder;
    private ProfiledPIDController pidController;
    private ElevatorFeedforward feedforward;
    private POMDigitalInput foldSwitch;
    private POMDigitalInput brakeSwitch;
    private BooleanSupplier isCoralIn;

    public elevatorIOReal(POMDigitalInput brakeSwitch) {
        motor = new POMSparkMax(ELEVATOR_ID);
        encoder = motor.getEncoder();
        foldSwitch = new POMDigitalInput(FOLD_SWITCH);
        this.brakeSwitch = brakeSwitch;
        pidController.setTolerance(TOLERANCE);

        feedforward = new ElevatorFeedforward(KS, KG, KV);
        pidController = new ProfiledPIDController(KP, KI, KD,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoder();

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorConnected = true;
        inputs.elevatorVelocity = encoder.getVelocity();
        inputs.elevatorPosition = encoder.getPosition();
        inputs.elevatorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.foldSwitch = foldSwitch.get();
        inputs.brakeSwitch = brakeSwitch.get();

        resetIfPressed();
    }

    private void resetEncoder() {
        encoder.setPosition(0.0);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setGoal(double goal) {
        pidController.setGoal(goal);

        setVoltage(pidController.calculate(encoder.getPosition()) +
                feedforward.calculate(pidController.getSetpoint().velocity));
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> pidController.atGoal();
    }

    @Override
    public void stopMotor() {
        motor.setVoltage(feedforward.calculate(pidController.getSetpoint().velocity));
    }

    @Override
    public void resetIfPressed() {
        if (foldSwitch.get()) {
            encoder.setPosition(0.0);
        }
        if (brakeSwitch.get()) {
            motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            resetEncoder();
        } else {
            motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        }
    }

    @Override
    public boolean isPressed() {
        return foldSwitch.get();
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();
    }

    @Override
    public void resetPID() {
        pidController.reset(encoder.getPosition(), encoder.getVelocity());
    }

    @Override
    public void resetPID(double newGoal) {
        if (newGoal - encoder.getPosition() > 0) {
            pidController.reset(encoder.getPosition(), Math.max(encoder.getVelocity(), feedforward.calculate(1)));
        } else {
            pidController.reset(encoder.getPosition(), Math.min(encoder.getVelocity(), feedforward.calculate(1)));
        }
    }

    // @Override
    public void initSendable(SendableBuilder builder) {// TODO check if work
        builder.setSmartDashboardType("Coral Arm");
        builder.addDoubleProperty("P", pidController::getP, pidController::setP);
        builder.addDoubleProperty("I", pidController::getI, pidController::setI);
        builder.addDoubleProperty("D", pidController::getD, pidController::setD);
        builder.addDoubleArrayProperty("Max Velocity, Max Acceleration", () -> {
            Constraints constraints = pidController.getConstraints();
            return new double[] { constraints.maxVelocity, constraints.maxAcceleration };
        },
                (constraints) -> pidController.setConstraints(new Constraints(constraints[0],
                        constraints[1])));

        builder.addDoubleArrayProperty("FeedForward", () -> {
            return new double[] { feedforward.getKg(), feedforward.getKs(),
                    feedforward.getKv() };
        },
                (feedForwardArray) -> {
                    feedforward.setKg(feedForwardArray[0]);
                    feedforward.setKs(feedForwardArray[1]);
                    feedforward.setKv(feedForwardArray[2]);
                });
    }

}
