package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeConstants.Parameters;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class FeedShooter extends CommandBase {
    
    Shooter shooter;
    private boolean noteWasDetected = false;
    private double initialEncoderCount = 0;


    public FeedShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }


    @Override
    public void initialize() {
        initialEncoderCount = shooter.getFeedingEncoderPose();

    }

    @Override
    public void execute() {
        // Check for note reaching resting spot based on encoder counts, but only after
        // initialization
        if (initialEncoderCount > 0 && shooter.getFeedingEncoderPose() >= initialEncoderCount + ShooterConstants.DIST_LEFT_SENSOR) {
            shooter.feedingSetPow(0); // Stop motors if resting position reached
        } else if (noteWasDetected) { // Note detected, use transfer speed
            shooter.feedingSetPow(ShooterConstants.SHOOTER_TRANSFER_SPEED); // Run motors at transfer speed
        } else {
            shooter.feedingSetPow(ShooterConstants.SHOOTER_SPEED); // Run motors at intake speed until note is detected
        }


        if (noteWasDetected) { // Placeholder for sensor detection
            if (initialEncoderCount == 0) { // Initialize only when note is first detected
                initialEncoderCount = shooter.getFeedingEncoderPose();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Command ends when note is detected and reaches resting spot
        return noteWasDetected && shooter.getFeedingEncoderPose() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.feedingSetPow(0); // Ensure motors stop
    }


}
