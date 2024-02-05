package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbManualCommand extends Command {
    private final ClimbSubsystem s_Telescope;
    private final XboxController s_Controller;

    public ClimbManualCommand(ClimbSubsystem telescope, XboxController xboxController) {
        this.s_Telescope = telescope;
        this.s_Controller = xboxController;
        addRequirements(telescope);
    }

    @Override
    public void initialize() {
        s_Telescope.stop();
        s_Telescope.s_Braker.release();
    }

    @Override
    public void execute() {
        double powerRight = deadband(s_Controller.getLeftY(), 0.1);
        double powerLeft = deadband(s_Controller.getRightY(), 0.1);

        s_Telescope.setRightPower(powerRight);
        s_Telescope.setLeftPower(powerLeft);
    }

    @Override
    public void end(boolean interrupted) {
        s_Telescope.s_Braker.brake();
        s_Telescope.stop();
    }

    private double deadband(double x, double threshold) {
        if (Math.abs(x) < threshold) return 0;
        else return x;
    }
}
