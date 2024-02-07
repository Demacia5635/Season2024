package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeConstants.Parameters;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private boolean noteWasDetected = false;
    private double initialEncoderCount = 0;

    public IntakeCommand(Intake intake) {
        addRequirements(intake);
        this.intake = intake;

        SmartDashboard.putData("Intake Notes", new InstantCommand(
                () -> new IntakeCommand(intake).schedule()));
    }

    @Override
    public void initialize() {
        super.initialize();
        initialEncoderCount = intake.getEncoderPos();
    }

    @Override
    public void execute() {
        // Check for note reaching resting spot based on encoder counts, but only after
        // initialization
        if (initialEncoderCount > 0 && intake.getEncoderPos() >= initialEncoderCount + Parameters.sensorToRestDist) {
            intake.setVelocity(0); // Stop motors if resting position reached
        } else if (noteWasDetected) { // Note detected, use transfer speed
            intake.setVelocity(Parameters.intakeTransferSpeed); // Run motors at transfer speed
        } else {
            intake.setVelocity(Parameters.intakeSpeed); // Run motors at intake speed until note is detected
        }

        if (noteWasDetected) { // Placeholder for sensor detection
            if (initialEncoderCount == 0) { // Initialize only when note is first detected
                initialEncoderCount = intake.getEncoderPos();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Command ends when note is detected and reaches resting spot
        return noteWasDetected && intake.getEncoderPos() >= initialEncoderCount + Parameters.sensorToRestDist;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(0); // Ensure motors stop
    }
}
