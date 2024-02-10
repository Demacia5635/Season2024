package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.amp.AmpConstants;
import frc.robot.subsystems.amp.Amp;
import frc.robot.utils.Trapezoid;
import frc.robot.utils.TrapezoidCalc;

public class OpenAmp extends CommandBase {

    private Amp amp;
    private TrapezoidCalc trapezoid;
    private double target = AmpConstants.Parameters.OPEN_ANGLE;


    public OpenAmp(Amp amp) {
        this.amp = amp;
        trapezoid = new TrapezoidCalc();
        addRequirements(amp);

    } 
    

    @Override
    public void execute() {
        trapezoid.trapezoid(amp.getVelRadArm(), AmpConstants.Parameters.MAX_ARM_VEL_OPEN,
         0, AmpConstants.Parameters.MAX_ARM_ACCEL_OPEN, target - amp.getArmAngle());

        amp.setVel(target, target);
    }

}
