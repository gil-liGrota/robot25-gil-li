package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.AutoLog;

public interface transferIO {

    @AutoLog
    public static class transferIOInputs {
        public double velocity;
        public double voltage;
        public double current;
        public boolean transferSensorInput;
    }
}
