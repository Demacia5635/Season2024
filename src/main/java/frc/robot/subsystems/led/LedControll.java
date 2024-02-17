package frc.robot.subsystems.led;

import static frc.robot.subsystems.chassis.ChassisConstants.CYCLE_DT;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.chassis.DriveAndPickNote;
import frc.robot.subsystems.intake.Intake;

public class LedControll extends SubsystemBase{
    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    public enum ledState{
        SEE_NOTE, INTAKE, DONE_INTAKE, READY_TO_USE, OFF;
    }
    public static ledState state = ledState.OFF; 

    public LedControll(int port, int count) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(count);
        led.setLength(buffer.getLength());
    }

    public void updateState(ledState state){
        switch (state) {
            case INTAKE:
                setColor(255, 255, 0);
                break;
            case SEE_NOTE:
                blink(0, 255, 0, 500);
                break;
            case DONE_INTAKE: 
                setColor(255, 0, 0);
                break;
            case READY_TO_USE:
                setColor(148, 0,211);
                break;
            default:
                setColor(0, 0, 0);
                break;
        }

    }

    public void start() {
        led.start();
    }

    public void stop() {
        led.stop();
    }

    public void blink(int r, int g, int b, double miliSeconds) {
        double milisecondTime = Timer.getFPGATimestamp() * 1000;
        if(milisecondTime % miliSeconds >= miliSeconds/2){
            setColor(r, g, b);
        }else{
            setColor(0, 0, 0);
        }
    };
    
    public void setColor(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.start();
        led.setData(buffer);
    }
    @Override
    public void periodic() {
        // updateState(state);
        
        
        
    }
}