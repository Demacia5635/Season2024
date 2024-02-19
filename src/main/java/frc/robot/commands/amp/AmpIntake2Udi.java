package frc.robot.commands.amp;


import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.SENSEOR_ANGLE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.AmpUdi;
import frc.robot.subsystems.amp.AmpConstants.Parameters;

public class AmpIntake2Udi extends Command {

    private final AmpUdi amp;
    private boolean noteWasDetected = false;
    private double initialEncoderCount = 0;
    private boolean hasEntered = false;

    private boolean debug = true;

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
        amp.setArmTargetAngle(SENSEOR_ANGLE);
        initialEncoderCount = 0;
    }

    private void debug(String msg) {
        if(debug) System.out.println(msg);
    }

    @Override
    public void execute() {

        debug("en 1" + (initialEncoderCount > 0
                && amp.getSmallWheelsPosition() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST));
        debug("en 2" + noteWasDetected);
        debug("en lol " +  !hasEntered);
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
                    debug("en 3 true");
                }
            }

            if (noteWasDetected) { // Placeholder for sensor detection
                debug("detected " +true);
                if (initialEncoderCount == 0) { // Initialize only when note is first detected
                    debug("limit amp =" + amp.getNoteSensorVolt());

                    initialEncoderCount = amp.getSmallWheelsPosition();
                    debug("en 6 true");
                }
            }
        }

        debug("current= " + amp.getSmallWheelsMotorCurrent());

        if ((amp.isOpen()) && (!amp.isSensingNote())) {
            amp.setWheelsPower(-Parameters.INTAKE_POWER); // Run motors at intake speed until note is out;
        }
    }

    @Override
    public boolean isFinished() {
        // Command ends when note is detected and reaches resting spot
        return amp.isAtSensor() && amp.isSensingNote();
    }

    @Override
    public void end(boolean interrupted) {
        amp.setWheelsPower(0);// Ensure motors stop
    }

}
