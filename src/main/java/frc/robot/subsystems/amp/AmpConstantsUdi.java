package frc.robot.subsystems.amp;

public class AmpConstantsUdi {
  public static class AmpDeviceID {

    public static final int ARM_MOTOR_ID = 21;
    public static final int LOCK_MOTOR_ID = 22;
    public static final int SMALL_WHEEL_MOTOR_ID = 23;
    public static final int BIG_WHEEL_MOTOR_ID = 24;
    public static final int ARM_POSITION_SENSOR_ID = 0;

  }

  public static class ConvertionParams {

    public static final double ARM_GEAR_RATIO = 1 / 128.0;
    public static final double SMALL_WHEEL_GEAR_RATIO = 1 / 8.0;
    public static final double BIG_WHEEL_GEAR_RATIO = 1 / 8.0;

    public static final double FALCON_PULSES_PER_ROTATION = 2048;
    public static final double NEO_PULES_PER_ROTATION = 42;

    public static final double PULSE_PER_RAD = FALCON_PULSES_PER_ROTATION / ARM_GEAR_RATIO / 2 / Math.PI;

  }

  public static class Parameters {

    public static final double DEADBAND = 0.2;

    public static final double ARM_KP = 0.0570401;
    public static final double ARM_KI = 0.0;
    public static final double ARM_KD = 0.0;

    public static final double MAX_ARM_VEL_UP = Math.toRadians(180);
    public static final double MAX_ARM_ACCEL_UP = Math.toRadians(600);
    public static final double ARM_DOWN_POWER = -0.1;
    public static final double ARM_RAD_ERROR = Math.toRadians(3);

    public static final double UP_ANGLE = Math.toRadians(55);
    public static final double HOME_ANGLE = Math.toRadians(-60);
    public static final double SENSEOR_ANGLE = Math.toRadians(-55);

    // intake data
    public static final double NUMBER_OF_ROTATION = 0.8;
    public static final double SENSOR_TO_REST_DIST = NUMBER_OF_ROTATION / (ConvertionParams.SMALL_WHEEL_GEAR_RATIO);
    public static final double NOTE_VOLTAGE = 2.7;
    public static final double CRITICAL_CURRENT = 0;
    public static final double INTAKE_POST_SENSOR_POWER = 0.4;
    public static final double INTAKE_INITIAL_POWER = 0.4;
    public static final double INTAKE_PRE_SENSOR_POWER = 0.4;

    public static final double INTAKE_RELEASE_SMALL_POWER = -0.1;
    public static final double INTAKE_RELEASE_BIG_POWER = -0.4;
    public static final double INTAKE_RELEASE_DURATION = 0.7;

    // lock data
    public static final double LOCK_CURRENT = 1.2;
    public static final double LOCK_POWER = 0.9;
    public static final double UNLOCK_POWER = 0.4;
    public static final double UNLOCK_TIME = 0.5;

  }

  public static class ArmFFParamaters {
    public static final double ARM_UP_KS = 1.287228076;
    public static final double ARM_UP_KV = 0.005323938;
    public static final double ARM_UP_KA = -0.004043448;
    public static final double ARM_KCOS = -1.202533577;
  }

}