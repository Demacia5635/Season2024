package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.*;
import frc.robot.Constants.IntakeConstants.Parameters;
import frc.robot.subsystems.Intake;

public class ShootCommand extends CommandBase {
    private final Intake intake;
    private double count = 0;;

    public ShootCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        SmartDashboard.putData("Shoot Notes", new InstantCommand(
                () -> new ShootCommand(intake).schedule()).withTimeout(3));
    }

    @Override
    public void execute() {
        count+=0.02;
        intake.setVelocity(IntakeConstants.Parameters.shootVelocity); // Set shoot velocity
    }

    @Override
    public boolean isFinished() {
        return count>=Parameters.dispenseTime;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}