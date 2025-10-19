package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public interface ledsIO {
    @AutoLog
    public static class ledsIOInputs {
        public String[] ledColorList;
    }

    public default void updateInputs(ledsIOInputs inputs) {
    }

    public default void setAll(Color color) {
    }

    public default void setParts(Color... colors) {
    }

    public default void blink(Color color, double seconds) {
    }

    public default void blink(LEDPattern pattern, double seconds) {
    }

    public default void rainbow() {
    }
}