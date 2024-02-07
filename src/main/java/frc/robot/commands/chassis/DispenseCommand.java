package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.*;
import frc.robot.Constants.IntakeConstants.Parameters;
import frc.robot.subsystems.Intake;

public class DispenseCommand extends CommandBase {
    private final Intake intake;
    private double count = 0;

    public DispenseCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        SmartDashboard.putData("Dispense Note", new InstantCommand(
                () -> new DispenseCommand(intake).schedule()));
    }

    @Override
    public void initialize() {
        intake.setPower(IntakeConstants.Parameters.dispenseVelocity);  // Set timeout for dispensing
        count+=0.02;
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