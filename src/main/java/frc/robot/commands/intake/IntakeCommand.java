package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.Parameters;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private boolean noteWasDetected = false;
    private double initialEncoderCount = 0;
    private boolean hasEntered = false;

    public IntakeCommand(Intake intake) {
        this.intake = intake;

        addRequirements(intake);


        SmartDashboard.putData("Intake Notes", new InstantCommand(
                () -> new IntakeCommand(intake).schedule()));
    }

    @Override
    public void initialize() {
        super.initialize();
        initialEncoderCount = 0;
    }

    @Override
    public void execute() {

        SmartDashboard.putBoolean("entered 1", initialEncoderCount > 0 && intake.getEncoderPos() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST);// Stop motors if resting position reached
        SmartDashboard.putBoolean("entered 2", noteWasDetected);
        SmartDashboard.putBoolean("entered lol", !hasEntered);

        if(intake.isNotePresent()) noteWasDetected = true;
        if(intake.isCriticalCurrent()) hasEntered = true;

        // Check for note reaching resting spot based on encoder counts, but only after
        // initialization
        if (initialEncoderCount > 0 && intake.getEncoderPos() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST) {
            intake.setPower(0); 
            
        } else if (noteWasDetected) { // Note detected, use transfer speed
            intake.setPower(Parameters.INTAKE_TRANSFER_POWER); // Run motors at transfer speed
        } else if (!hasEntered ){
            intake.setPower(Parameters.INTAKE_POWER); // Run motors at intake speed until note is detected
           
        }
        else {
            intake.setPower(Parameters.INTAKE_PRE_LIMIT_POWER); // Run motors at intake speed until note is detected
            SmartDashboard.putBoolean("entered 3", true);
        }

        if (noteWasDetected) { // Placeholder for sensor detection
            SmartDashboard.putBoolean("entered 4", true);
            if (initialEncoderCount == 0) { // Initialize only when note is first detected
                SmartDashboard.putNumber("limit", intake.getLimitVolt());

                initialEncoderCount = intake.getEncoderPos();
                SmartDashboard.putBoolean("entered 6", true);
            }
        }

        System.out.println("current= "+ intake.getMotorCurrent());
    }

    @Override
    public boolean isFinished() {
        // Command ends when note is detected and reaches resting spot
        SmartDashboard.putBoolean("is finished", noteWasDetected && intake.getEncoderPos() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST);
        return initialEncoderCount > 0 && intake.getEncoderPos() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);// Ensure motors stop
    }
}