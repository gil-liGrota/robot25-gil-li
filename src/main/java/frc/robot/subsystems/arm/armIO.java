package frc.robot.subsystems.arm;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface armIO {

    @AutoLog
    public static class armIOInputs {

        boolean motorConnected = false;
        double coralArmVelocity = 0.0;
        double coralArmPosition = 0.0;
        double coralArmAppliedVolts = 0.0;
        boolean lowSwitch = false;
        boolean highSwitch = false;
        boolean brakeSwitch = false;
    }

    public default void updateInputs(armIOInputs inputs) {
    }

    public default void setVelocity(double speed) {
    }

    public default void setVoltage(double voltage) {
    }

    public default void setGoal(double goal) {
    }

    public default BooleanSupplier atGoal() {
        return () -> false;
    }

    public default void stopMotor() {
    }

    public default void resistGravity() {
    }

    public default void resetIfPressed() {
    }

    public default void setFeedForward(double velocity) {
    }

    public default boolean isPressed() {
        return false;
    }

    public default void setVoltageWithResistGravity(double voltage) {
    }

    public default double getPosition() {
        return 0;
    }

    public default void resetPID() {
    }

    public default void resetPID(double newGoal) {
    }

    public default double directionalHighSpeed() {
        return 0.0;
    }

    public default void stayInCurrentGoal() {

    }

    public default boolean getHighSwitch() {
        return false;
    }

    public default boolean getLowSwitch() {
        return false;
    }

}
