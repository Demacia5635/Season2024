package frc.robot.subsystems.leds;

import java.util.Arrays;
import java.util.stream.IntStream;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.leds.LedBlink;
import frc.robot.commands.leds.Rainbow;
import frc.robot.subsystems.leds.utils.IndividualLed;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_MOTOR;

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

    public void setColor(IndividualLed... leds) {
        LedsManager.getInstance().update(Arrays.stream(leds)
                .map((led) -> new IndividualLed(offset + led.index, led.color)).toArray(IndividualLed[]::new));
    }

    public void setColor(Color color) {
        setColor(IntStream.range(0, size).mapToObj((i) -> new IndividualLed(i, color))
                .toArray(IndividualLed[]::new));
    }

    public void turnOff() {
        setColor(IntStream.range(0, size).mapToObj((i) -> new IndividualLed(i, Color.kBlack))
                .toArray(IndividualLed[]::new));
    }

    public Color[] getColors() {
        return LedsManager.getInstance().getColors(offset, size);
    }

    public void setColor(Color[] colors) {
        setColor(IntStream.range(0, Math.min(colors.length, size)).mapToObj((i) -> new IndividualLed(i, colors[i]))
                .toArray(IndividualLed[]::new));
    }
    
    public void setBlink(IndividualLed... leds) {
        new LedBlink(this, leds).repeatedly().schedule();
    }

    public void setBlink(Color color){
        new LedBlink(this, color).repeatedly().schedule();
    }

    public void setBlink(Color[] colors) {
        new LedBlink(this, colors).repeatedly().schedule();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        
        double[] llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
        double Dist = llpython[0];
        boolean isStart = RobotContainer.robotContainer.driveToNote.isScheduled();
        boolean isNotePresent = RobotContainer.robotContainer.intake.isNotePresent();
        boolean isShooterReady = Math.abs(RobotContainer.robotContainer.wantedAngle - RobotContainer.robotContainer.shooter.getAngle()) < 1 && 
                                 Math.abs(RobotContainer.robotContainer.wantedShootingVel - RobotContainer.robotContainer.shooter.getMotorVel(SHOOTER_MOTOR.UP)) < 0.3;
        // System.out.println("Dist is : " + Dist);
        if(isShooterReady){
            setColor(Color.kGreen);
        } else if(isStart){
            setColor(Color.kOrange);
        } else if (isNotePresent){
            setColor(Color.kPurple);
        } else if (Dist != 0){
            setColor(Color.kYellow);
        } else {
            turnOff();
            // new Rainbow(this, 3).repeatedly().ignoringDisable(true).schedule();
        }

    }
}
