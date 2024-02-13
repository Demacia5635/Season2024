package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

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

    public void changeColor(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.start();
        led.setData(buffer);
    }
}