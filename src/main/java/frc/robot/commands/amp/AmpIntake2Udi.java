package frc.robot.commands.amp;

import java.lang.reflect.Parameter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.amp.AmpConstantsUdi;
import frc.robot.subsystems.amp.AmpUdi;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.amp.AmpConstants.Parameters;

public class AmpIntake2Udi
        extends CommandBase {

    private final AmpUdi amp;
    private boolean noteWasDetected = false;
    private double initialEncoderCount = 0;
    private boolean hasEntered = false;

    public AmpIntake2Udi(AmpUdi amp) {
        this.amp = amp;
        addRequirements(amp);

    }

    @Override
    public void initialize() {
        super.initialize();
        amp.setArmBrake();
        amp.setWheelsBrake();
        amp.wheelsSetInverted(true);
        amp.wheelsEncoderReset();
        initialEncoderCount = 0;
    }

    @Override
    public void execute() {

        SmartDashboard.putBoolean("en 1", initialEncoderCount > 0
                && amp.getSmallWheelsPosition() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST);// Stop motors
                                                                                                         // if resting
                                                                                                         // position
                                                                                                         // reached
        SmartDashboard.putBoolean("en 2", noteWasDetected);
        SmartDashboard.putBoolean("en lol", !hasEntered);
        if (amp.isSensingNote())
            noteWasDetected = true;
        if (amp.isCriticalCurrent())
            hasEntered = true;

        // Check for note reaching resting spot based on encoder counts, but only after
        // initialization
        if (amp.isAtSensor()) {
            if (initialEncoderCount > 0
                    && amp.getSmallWheelsPosition() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST) {
                amp.setWheelsPower(0, 0);

            } else if (noteWasDetected) { // Note detected, use transfer speed
                amp.setWheelsPower(Parameters.INTAKE_TRANSFER_POWER); // Run motors at transfer speed
            } else if (!hasEntered) {
                amp.setWheelsPower(Parameters.INTAKE_POWER); // Run motors at intake speed until note is detected

            } else {
                if (initialEncoderCount > 0
                        && amp.getSmallWheelsPosition() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST) {
                    amp.setWheelsPower(Parameters.INTAKE_PRE_LIMIT_POWER); // Run motors at intake speed until note is
                                                                           // detected
                    SmartDashboard.putBoolean("en 3", true);
                }
            }

            if (noteWasDetected) { // Placeholder for sensor detection
                SmartDashboard.putBoolean("detected ", true);
                if (initialEncoderCount == 0) { // Initialize only when note is first detected
                    SmartDashboard.putNumber("limit amp", amp.getNoteSensorVolt());

                    initialEncoderCount = amp.getSmallWheelsPosition();
                    SmartDashboard.putBoolean("en 6", true);
                }
            }
        }

        System.out.println("current= " + amp.getSmallWheelsMotorCurrent());

        if ((amp.isOpen()) && (!amp.isSensingNote())) {
            amp.setWheelsPower(-Parameters.INTAKE_POWER); // Run motors at intake speed until note is out;
        }
    }

    @Override
    public boolean isFinished() {
        // Command ends when note is detected and reaches resting spot
        return amp.isOpen() && amp.isSensingNote();
    }

    @Override
    public void end(boolean interrupted) {
        amp.setWheelsPower(0);// Ensure motors stop
    }

}
