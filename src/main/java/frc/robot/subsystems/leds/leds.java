package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class leds extends SubsystemBase {
    private ledsIO io;
    private final ledsIOInputsAutoLogged inputs = new ledsIOInputsAutoLogged();

    public leds(ledsIO io) {
        this.io = io;
    }

    public void setAll(Color color) {
        io.setAll(color);
    }

    public void setParts(Color... colors) {
        io.setParts(colors);
    }

    public void blink(Color color, double seconds) {
        io.blink(color, seconds);
    }

    public void blink(LEDPattern pattern, double seconds) {
        io.blink(pattern, seconds);
    }

    public void rainbow() {
        io.rainbow();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("leds", inputs);
    }
}
