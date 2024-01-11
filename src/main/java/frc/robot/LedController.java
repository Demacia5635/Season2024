package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LedController {
    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    public LedController(int port, int count) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(count);
        led.setLength(buffer.getLength());
    }

    public void start() {
        led.start();
    }

    public void stop() {
        led.stop();
    }

    public void changeColor(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        led.start();
        led.setData(buffer);
    }
}
