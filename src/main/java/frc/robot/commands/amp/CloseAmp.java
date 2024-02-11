package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.amp.AmpConstants;
import frc.robot.subsystems.amp.Amp;
import frc.robot.utils.Trapezoid;
import frc.robot.utils.TrapezoidCalc;

public class CloseAmp extends CommandBase {

    private Amp amp;
    private TrapezoidCalc trapezoid;
    private double target = AmpConstants.Parameters.CLOSE_ANGLE;
    private double vel;


    public CloseAmp(Amp amp) {
        this.amp = amp;
        trapezoid = new TrapezoidCalc();
        amp.setBrake();
        addRequirements(amp);

    } 
    

    @Override
    public void execute() {
        vel = trapezoid.trapezoid(amp.getVelRadArm(), AmpConstants.Parameters.MAX_ARM_VEL_CLOSE,
         0, AmpConstants.Parameters.MAX_ARM_ACCEL_CLOSE, target - amp.getArmAngle());

        amp.setArmVelocityClose(vel);
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

