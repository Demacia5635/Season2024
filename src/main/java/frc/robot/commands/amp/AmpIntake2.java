package frc.robot.commands.amp;

import java.lang.reflect.Parameter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.amp.AmpConstants.Parameters;

public class AmpIntake2 extends CommandBase {

    private final Amp amp;
    private boolean noteWasDetected = false;
    private double initialIntakeRev = 0;
    private boolean hasEntered = false;

    public AmpIntake2(Amp amp) {
        this.amp = amp;
        addRequirements(amp);

    }

    @Override
    public void initialize() {
        super.initialize();
        amp.setArmBrake();
        initialIntakeRev = 0;
        noteWasDetected = false;
        hasEntered = false;
    }

    @Override
    public void execute() {

        SmartDashboard.putBoolean("en 1", initialIntakeRev > 0
                && amp.getIntakeRev() >= initialIntakeRev + Parameters.SENSOR_TO_REST_DIST);// Stop motors if
                                                                                                     // resting position
                                                                                                     // reached
        SmartDashboard.putBoolean("en 2", noteWasDetected);
        SmartDashboard.putBoolean("en lol", !hasEntered);
        if (amp.isSensingNote())
            noteWasDetected = true;
        if (amp.isIntakePushingNote())
            hasEntered = true;

        // Check for note reaching resting spot based on encoder counts, but only after
        // initialization
        if (initialIntakeRev > 0
                && amp.getIntakeRev() >= initialIntakeRev + Parameters.SENSOR_TO_REST_DIST) {
            amp.setIntakePower(0);
            System.out.println("AT POSITION: hasEntered = " + hasEntered +
                    " noteWasDetec = " + noteWasDetected + " pose = " + amp.getIntakeRev() +
                    "StartPose = " + initialIntakeRev);

        } else if (noteWasDetected) { // Note detected, use transfer speed
            amp.setIntakePower(Parameters.INTAKE_TRANSFER_POWER); // Run motors at transfer speed
            System.out.println("TRANSFER: hasEntered = " + hasEntered +
                    " noteWasDetec = " + noteWasDetected + " pose = " + amp.getIntakeRev() +
                    "StartPose = " + initialIntakeRev);
        } else if (!hasEntered) {
            amp.setIntakePower(Parameters.INTAKE_POWER); // Run motors at intake speed until note is detected
            System.out.println("INTAKE: hasEntered = " + hasEntered +
                    " noteWasDetec = " + noteWasDetected + " pose = " + amp.getIntakeRev() +
                    "StartPose = " + initialIntakeRev);

        } else {
            // if(initialEncoderCount > 0 && amp.getNeoPoseByPulses() >= initialEncoderCount
            // + Parameters.SENSOR_TO_REST_DIST){
            amp.setIntakePower(Parameters.INTAKE_PRE_LIMIT_POWER); // Run motors at intake speed until note is detected
            System.out.println("PRE: hasEntered = " + hasEntered +
                    " noteWasDetec = " + noteWasDetected + " pose = " + amp.getIntakeRev() +
                    "StartPose = " + initialIntakeRev);
        }

        if (noteWasDetected) { // Placeholder for sensor detection
            SmartDashboard.putBoolean("detected ", true);
            if (initialIntakeRev == 0) { // Initialize only when note is first detected
                SmartDashboard.putNumber("limit amp", amp.getNoteSensorVolt());

                initialIntakeRev = amp.getIntakeRev();
                SmartDashboard.putBoolean("en 6", true);
            }
        }

        // System.out.println("current= "+ amp.getMotorCurrent());

    }

    @Override
    public boolean isFinished() {
        // Command ends when note is detected and reaches resting spot
        return initialIntakeRev > 0
                && amp.getIntakeRev() >= initialIntakeRev + Parameters.SENSOR_TO_REST_DIST;
    }

    @Override
    public void end(boolean interrupted) {
        amp.setIntakePower(0);// Ensure motors stop
    }

}