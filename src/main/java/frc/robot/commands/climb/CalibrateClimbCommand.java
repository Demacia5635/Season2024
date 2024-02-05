package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

import static frc.robot.Constants.ClimbConstants.*;

public class CalibrateClimbCommand extends Command {
    private final ClimbSubsystem s_Climb;

    public CalibrateClimbCommand(ClimbSubsystem climb) {
        s_Climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        s_Climb.s_Braker.release();
        s_Climb.setLeftPower(-CLIMB_POWER);
    }

    @Override
    public void end(boolean interrupt) {
        s_Climb.s_Braker.brake();
        s_Climb.stop();
        s_Climb.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return s_Climb.getRightLimSwitch() && s_Climb.getLeftLimSwitch();
    }
}
