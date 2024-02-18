package frc.robot.subsystems.led;

import java.util.Arrays;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.led.LedBlink;
import frc.robot.subsystems.led.utils.IndividualLed;

public class SubStrip extends SubsystemBase {
    public final int size;
    private final int offset;

    public SubStrip(int offset, int size) {
        this.offset = offset;
        this.size = size;
    }

    public SubStrip(int size){
        this.offset = 0;
        this.size = size;
    }

    public Command setColor(IndividualLed... leds) {
        return new InstantCommand(()-> LedsManager.getInstance().update(Arrays.stream(leds)
                .map((led) -> new IndividualLed(offset + led.index, led.color)).toArray(IndividualLed[]::new)), this).repeatedly();
    }

    public Command setColor(Color color) {
        return setColor(IntStream.range(0, size).mapToObj((i) -> new IndividualLed(i, color))
                .toArray(IndividualLed[]::new));
    }

    public Command turnOff() {
        return setColor(IntStream.range(0, size).mapToObj((i) -> new IndividualLed(i, Color.kBlack))
                .toArray(IndividualLed[]::new));
    }

    public Color[] getColors() {
        return LedsManager.getInstance().getColors(offset, size);
    }

    public Command setColor(Color[] colors) {
        return setColor(IntStream.range(0, Math.min(colors.length, size)).mapToObj((i) -> new IndividualLed(i, colors[i]))
                .toArray(IndividualLed[]::new));
    }
    
    public Command setBlink(IndividualLed... leds) {
        return new LedBlink(this, leds).repeatedly();
    }

    public Command setBlink(Color color){
        return new LedBlink(this, color).repeatedly();
    }

    public Command setBlink(Color[] colors) {
        return new LedBlink(this, colors).repeatedly();
    }

}
