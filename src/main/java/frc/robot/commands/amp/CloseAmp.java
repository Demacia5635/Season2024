package frc.robot.commands.amp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.amp.AmpConstants;
import frc.robot.subsystems.amp.Amp;
import frc.robot.utils.Trapezoid;
import frc.robot.utils.TrapezoidCalc;

public class CloseAmp extends CommandBase {

    private Amp amp;
    private double velRad;



    public CloseAmp(Amp amp, double velRad) {
        this.amp = amp;
        this.velRad = velRad;
        addRequirements(amp);
        
    } 
    
    @Override
    public void initialize() {
        amp.setBrake();
    }
    
    @Override
    public void execute() {
        amp.setVel(velRad);
    }

    @Override
    public boolean isFinished() {
        return amp.isClose();
    }

    @Override
    public void end(boolean interrupted) {
        amp.setPowerArm(0);
    }

}

