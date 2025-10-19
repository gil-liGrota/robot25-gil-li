package frc.robot.commands;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.leds.leds;

public class ledsCommands {
    public static Command setAll(leds leds, Color color) {
        return Commands.run(() -> leds.setAll(color), leds);
    }

    public static Command setParts(leds leds, Color... colors) {
        return Commands.run(() -> leds.setParts(colors), leds);
    }

    public static Command blink(leds leds, Color color, double seconds) {
        return Commands.run(() -> leds.blink(color, seconds), leds);
    }

    public static Command blink(leds leds, LEDPattern pattern, double seconds) {
        return Commands.run(() -> leds.blink(pattern, seconds), leds);
    }

    public static Command rainbow(leds leds) {
        return Commands.run(() -> leds.rainbow(), leds);

    }

}
