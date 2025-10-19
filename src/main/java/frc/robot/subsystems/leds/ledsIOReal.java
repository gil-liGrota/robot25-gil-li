package frc.robot.subsystems.leds;

import static frc.robot.subsystems.leds.ledsConstance.*;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.ledsIO.ledsIOInputs;

import static edu.wpi.first.units.Units.*;

public class ledsIOReal implements ledsIO {
    private final AddressableLED led = new AddressableLED(LED_PORT);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LENGTH);
    LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    Distance kLedSpacing = Meters.of(1 / 70.0);

    LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(2),
            kLedSpacing);

    public ledsIOReal() {
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void updateInputs(ledsIOInputs inputs) {
        inputs.ledColorList = new String[ledBuffer.getLength()];
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            inputs.ledColorList[i] = ledBuffer.getLED(i).toHexString();
        }
    }

    // Sets all LEDs to the same color.
    @Override
    public void setAll(Color color) {
        LEDPattern solidColorPattern = LEDPattern.solid(color);

        solidColorPattern.applyTo(ledBuffer);
        updateLEDs();
    }

    // Gets a list of Colors and splits them equally across the LED strip
    @Override
    public void setParts(Color... colors) {
        int numberOfParts = colors.length;
        Map<Double, Color> map = new HashMap<Double, Color>();

        for (int i = 0; i < numberOfParts; i++) {
            map.put(i * (1.0 / numberOfParts), colors[i]);
        }

        LEDPattern steps = LEDPattern.steps(map);

        steps.applyTo(ledBuffer);
        updateLEDs();

    }

    // Blinks a color on and off for a given number of seconds
    @Override
    public void blink(Color color, double seconds) {
        LEDPattern pattern = LEDPattern.solid(color);
        blink(pattern, seconds);
    }

    // Blinks a pattern on and off for a given number of seconds
    @Override
    public void blink(LEDPattern pattern, double seconds) {
        LEDPattern blinking = pattern.blink(Seconds.of(seconds));
        blinking.applyTo(ledBuffer);
        updateLEDs();
    }

    @Override
    public void rainbow() {
        m_scrollingRainbow.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void updateLEDs() {
        led.setData(ledBuffer);
    }

}
