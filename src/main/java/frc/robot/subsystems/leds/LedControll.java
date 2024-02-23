package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_MOTOR;
import frc.robot.utils.Utils;

public class LedControll extends SubsystemBase{
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    public int size;
    public double currentH;
    public boolean GAY = false;

    public LedControll(int port, int count) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(count);
        led.setLength(buffer.getLength());
        this.size = count;
        this.currentH = 0;
        led.start();
    }
    
    public void setColor(Color color) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        led.setData(buffer);
    }

    public void setColor(Color[] colors) {
        for (int i = 0; i < colors.length; i++) {
            buffer.setLED(i, colors[i]);
        }
        led.setData(buffer);
    }

    /**
     * needs to add {@code currentH = 0;} after the function have been calld
     */
    public void rainbow(){
        Color[] colors = new Color[size];
        for (int i = 0; i < colors.length; i++) {
            colors[i] = Color.fromHSV((int) (currentH + i * 3), 255, 255);

        }
        setColor(colors);
        currentH += 3;
        currentH %= 180;
    }

    @Override
    public void periodic() {
        super.periodic();

        

        boolean Dist = Utils.seeNote();//llpython[0];
        // System.out.println(Dist);
        boolean isStart = DriveToNote.isStart;
        boolean isNotePresent = RobotContainer.robotContainer.intake.isNotePresent();
        boolean isShooterReady = RobotContainer.robotContainer.shooter.isShootingReady();
        // System.out.println("Dist is : " + Dist);
        if(isShooterReady){
            setColor(Color.kWhite);
        } else if(isStart){
            setColor(Color.kOrange);
        } else if (isNotePresent){
            setColor(Color.kPurple);
        } else if (Dist){
            setColor(Color.kGreen);
        } else {
            setColor(Color.kBlack);
        }
    }
}