package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

public interface transferIO {

    @AutoLog
    public static class transferIOInputs {
        public double velocity;
        public double voltage;
        public boolean transferSensorInput;
    }

    public default void updateInputs(transferIOInputs inputs) {
    }

    public default void setVoltage(double voltage) {

    }

    public default void setVelocity(double velocity) {

    }

    public default void stopMotor() {

    }

    public default boolean isCoralIn() {
        return false;
    }
}
