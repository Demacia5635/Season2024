package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbBrakerSubsystem;

import static frc.robot.Constants.ClimbConstants.*;

public class BrakeClimbCommand extends Command {
    private final ClimbBrakerSubsystem s_Braker;

    public BrakeClimbCommand(ClimbBrakerSubsystem braker) {
        this.s_Braker = braker;
        addRequirements(braker);
    }

    @Override
    public void initialize() {
        s_Braker.setPower(1);
    }

    @Override
    public void end(boolean interrupt) {
        s_Braker.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return s_Braker.getCurrent() >= BRAKE_CURRENT_THRESHOLD;
    }
}
