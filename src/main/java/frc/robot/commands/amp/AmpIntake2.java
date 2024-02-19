package frc.robot.commands.amp;

import java.lang.reflect.Parameter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.amp.AmpConstants.Parameters;

public class AmpIntake2 extends CommandBase{

    private final Amp amp;
    private boolean noteWasDetected = false;
    private double initialEncoderCount = 0;
    private boolean hasEntered = false;

    public AmpIntake2(Amp amp) {
        this.amp = amp;
        addRequirements(amp);
        
    }
    
    @Override
    public void initialize() {
        super.initialize();
        amp.setBrakeArm();
        amp.setBrake();
        amp.neosSetInverted(true);
        amp.resetStartPulses();
        initialEncoderCount = 0;
    }

    @Override
    public void execute() {

        SmartDashboard.putBoolean("en 1", initialEncoderCount > 0 && amp.getNeoPoseByPulses() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST);// Stop motors if resting position reached
        SmartDashboard.putBoolean("en 2", noteWasDetected);
        SmartDashboard.putBoolean("en lol", !hasEntered);
        if(amp.isNote()) noteWasDetected = true;
        if(amp.isCriticalCurrent()) hasEntered = true;

        // Check for note reaching resting spot based on encoder counts, but only after
        // initialization
        if (initialEncoderCount > 0 && amp.getNeoPoseByPulses() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST) {
            amp.setNeosPower(0, 0);
            
        } else if (noteWasDetected) { // Note detected, use transfer speed
            amp.setNeosPower(Parameters.INTAKE_TRANSFER_POWER); // Run motors at transfer speed
        } else if (!hasEntered ){
            amp.setNeosPower(Parameters.INTAKE_POWER); // Run motors at intake speed until note is detected
           
        } else {
            if(initialEncoderCount > 0 && amp.getNeoPoseByPulses() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST){
                amp.setNeosPower(Parameters.INTAKE_PRE_LIMIT_POWER); // Run motors at intake speed until note is detected
                SmartDashboard.putBoolean("en 3", true);
            }
        }

        if (noteWasDetected) { // Placeholder for sensor detection
            SmartDashboard.putBoolean("detected ", true);
            if (initialEncoderCount == 0) { // Initialize only when note is first detected
                SmartDashboard.putNumber("limit amp", amp.getLimitVolt());

                initialEncoderCount = amp.getNeoPoseByPulses();
                SmartDashboard.putBoolean("en 6", true);
            }
        }
        

        System.out.println("current= "+ amp.getMotorCurrent());

    }

    @Override
    public boolean isFinished() {
        // Command ends when note is detected and reaches resting spot
        return initialEncoderCount > 0 && amp.getNeoPoseByPulses() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST;
    }

    @Override
    public void end(boolean interrupted) {
        amp.setNeosPower(0);// Ensure motors stop
    }
    
}
