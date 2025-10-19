package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.armConstance.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.POM_lib.Motors.POMSparkMax;
import frc.robot.POM_lib.sensors.POMDigitalInput;

public class armIOReal implements armIO {
    POMSparkMax motor;
    RelativeEncoder encoder;
    private ProfiledPIDController pidController;
    private ArmFeedforward feedforward;
    private POMDigitalInput lowSwitch;
    private POMDigitalInput highSwitch;
    private POMDigitalInput brakeSwitch;
    private BooleanSupplier isCoralIn;
    public double currentGoal = CLOSE_ARM_POSITION;
    private boolean manual = false;

    public armIOReal(POMDigitalInput brakeSwitch) {
        motor = new POMSparkMax(CORAL_ARM_ID);
        feedforward = new ArmFeedforward(KS, KG, KV);
        pidController = new ProfiledPIDController(KP, KI, KD,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        encoder = motor.getEncoder();

        highSwitch = new POMDigitalInput(HIGH_SWITCH);
        lowSwitch = new POMDigitalInput(LOW_SWITCH);
        this.brakeSwitch = brakeSwitch;
        pidController.setTolerance(TOLERANCE);// TODO check this

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake).inverted(INVERTED)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(VOLTAGE_COMPENSATION);

        // config.softLimit.forwardSoftLimit(FORWARD_SOFT_LIMIT);
        config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(POSITION_CONVERSION_FACTOR / 60.0);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetEncoder();
    }

    @Override
    public void updateInputs(armIOInputs inputs) {
        inputs.motorConnected = true /* turnConnectedDebouncer.calculate(sparkStickyFault) */;
        inputs.coralArmVelocity = encoder.getVelocity();
        inputs.coralArmPosition = encoder.getPosition();
        inputs.coralArmAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage(); // FIXME Wont Return Motor
                                                                                        // Voltage
        inputs.lowSwitch = lowSwitch.get();
        inputs.highSwitch = highSwitch.get();
        inputs.brakeSwitch = brakeSwitch.get();
        resetIfPressed();
    }

    private void resetEncoder() {
        if (highSwitch.get()) {
            encoder.setPosition(Math.PI / 2);
        }
        if (lowSwitch.get()) {
            encoder.setPosition(-Math.PI / 2);
        }
    }

    @Override
    public void resetIfPressed() {
        if (lowSwitch.get() || highSwitch.get()) {
            resetEncoder();
        }
        if (brakeSwitch.get()) {
            motor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        } else {
            motor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        }

    }

    @Override
    public void setVelocity(double speed) {
        motor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setGoal(double goal) {
        pidController.setGoal(goal);

        setVoltage(pidController.calculate(encoder.getPosition())
                + getFeedForwardVelocity(pidController.getSetpoint().velocity));

        currentGoal = goal;
        manual = false;
    }

    private double getFeedForwardVelocity(double velocity) {
        org.littletonrobotics.junction.Logger.recordOutput("real kG", feedforward.getKg());
        double voltage = feedforward.calculate(encoder.getPosition(), velocity);
        if (encoder.getPosition() < 0 && encoder.getPosition() > -1.35) {
            voltage += 0.05;
        }
        if (encoder.getPosition() > currentGoal) {
            // voltage += 0.05;
            feedforward.setKg(0.94);
        } else {
            feedforward.setKg(0.41);
        }

        return voltage;
    }

    @Override
    public BooleanSupplier atGoal() {
        return () -> pidController.atGoal();
    }

    @Override
    public void stopMotor() {
        motor.setVoltage(getFeedForwardVelocity(0));
    }

    @Override
    public void resistGravity() {
        setVoltage(getFeedForwardVelocity(0));
    }

    @Override
    public void setFeedForward(double velocity) {
        motor.setVoltage(getFeedForwardVelocity(velocity));
    }

    @Override
    public void setVoltageWithResistGravity(double voltage) {
        motor.setVoltage(getFeedForwardVelocity(0) + voltage);
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
            pidController.reset(encoder.getPosition(), Math.max(encoder.getVelocity(), getFeedForwardVelocity(1)));
        } else {
            pidController.reset(encoder.getPosition(), Math.min(encoder.getVelocity(), getFeedForwardVelocity(1)));
        }
    }

    @Override
    public double directionalHighSpeed() {
        return encoder.getPosition() > 0 ? -12 : 12;
    }

    @Override
    public void stayInCurrentGoal() {
        if (manual) {
            currentGoal = getPosition();
            resetPID();
        }
        if (getLowSwitch()) {
            setVoltage(-0.5);
        } else if (getHighSwitch() || encoder.getPosition() > (Math.PI / 2) - 0.01) {
            setVoltage(0.5);
        } else {
            setGoal(this.currentGoal);
        }
    }

    @Override
    public boolean getHighSwitch() {
        return highSwitch.get();
    }

    @Override
    public boolean getLowSwitch() {
        return lowSwitch.get();
    }

}
