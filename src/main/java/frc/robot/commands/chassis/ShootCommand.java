package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.chassis.Intake;

public class ShootCommand extends CommandBase {
    private final Intake intake;

    public ShootCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        SmartDashboard.putData("Shoot Notes", new InstantCommand(
                () -> new ShootCommand(intake).schedule()).withTimeout(3));
    }

    @Override
    public void initialize() {
        intake.setVelocity(IntakeConstants.Parameters.shootVelocity); // Set shoot velocity
        withTimeout(IntakeConstants.Parameters.shootTime); // Set timeout for shooting
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}