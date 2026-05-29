package frc.robot.subsystems;

import java.util.function.BiFunction;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Color;

/**
 * Subsystem for handling an LED string
 */
public class LedSubsystem extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public LedSubsystem(int ledPort, int ledLength) {
        this.m_ledBuffer = new AddressableLEDBuffer(ledLength);

        this.m_led = new AddressableLED(ledPort);
        this.m_led.setLength(ledLength);
        this.m_led.setData(this.m_ledBuffer);
        this.m_led.start();
    }

    /**
     * Set the color of all the LEDs
     * 
     * @param color
     */
    public void setSolidColor(Color color) {
        setColors((i, l) -> color);
    }

    /**
     * Set the colors of the LEDs in the string to the value returned by the function
     * 
     * @param callback
     */
    public void setColors(BiFunction<Integer, Integer, Color> callback) {
        for (int i = 0; i < this.m_ledBuffer.getLength(); ++i) {
            Color color = callback.apply(i, this.m_ledBuffer.getLength());

            m_ledBuffer.setRGB(i, color.r(), color.g(), color.b());
        }

        m_led.setData(m_ledBuffer);
    }
}
