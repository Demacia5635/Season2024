package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class DispenseCommand extends CommandBase {
    private final Intake intake;

    public DispenseCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        SmartDashboard.putData("Dispense Note", new InstantCommand(
                () -> new DispenseCommand(intake).schedule()));
    }

    @Override
    public void initialize() {
        intake.setVelocity(IntakeConstants.Parameters.dispenseVelocity); // Set dispense velocity
        withTimeout(IntakeConstants.Parameters.dispenseTime); // Set timeout for dispensing
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}