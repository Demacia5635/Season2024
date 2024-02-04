package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.telescope.Telescope;
import frc.robot.subsystems.telescope.Telescope.TelescopeType;

public class BrakeTelescopeCommand extends Command {
    private final Telescope telescope;
    private double startVoltage = 0;

    public BrakeTelescopeCommand(Telescope telescope) {
        this.telescope = telescope;
        addRequirements(telescope);
    }

    @Override
    public void initialize() {
        telescope.setBrakePower(1);
        startVoltage = telescope.getBrakeCurrent();
    }

    @Override
    public void end(boolean interrupt) {
        telescope.setBrakePower(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(startVoltage - telescope.getBrakeCurrent()) <= 0.2;
    }
}
