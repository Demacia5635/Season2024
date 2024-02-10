package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.Parameters;
import frc.robot.subsystems.intake.Intake;

public class ShootCommand extends CommandBase {
    private final Intake intake;

    public ShootCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
        intake.setBrake();
        SmartDashboard.putData("Shoot Notes", new InstantCommand(
                () -> new ShootCommand(intake).schedule()).withTimeout(3));
    }

    @Override
    public void execute() {
        intake.setVelocity(IntakeConstants.Parameters.SHOOT_POWER); // Set shoot velocity
        // withTimeout(IntakeConstants.Parameters.SHOOT_TIME); // Set timeout for shooting
    }

    @Override
    public boolean isFinished() {
        // Command ends when current falls under minimum
        return intake.getMotorCurrent() < IntakeConstants.Parameters.MIN_CURRENT_TO_SHOOTER;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}