package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.telescope.Telescope;

public class ClimbManualCommand  extends Command{
    private final Telescope telescope;
    private final XboxController controller;

    public ClimbManualCommand(Telescope telescope, XboxController xboxController) {
        this.telescope = telescope;
        this.controller = xboxController;
        addRequirements(telescope);
    }

    @Override
    public void execute() {
        double powerRight = deadband(controller.getRightY(), 0.1);
        double powerLeft = deadband(controller.getLeftY(), 0.1);

        telescope.setRightPower(powerRight);
        telescope.setLeftPower(powerLeft);
    }

    @Override
    public void end(boolean interrupted) {
        telescope.stop();
    }

    private double deadband(double x, double threshold) {
        if (Math.abs(x) < threshold) return 0;
        else return x;
    }
}
