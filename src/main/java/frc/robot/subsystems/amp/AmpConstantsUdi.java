package frc.robot.subsystems.amp;

import static frc.robot.subsystems.chassis.ChassisConstants.MOTOR_PULSES_PER_ROTATION;

public class AmpConstantsUdi {
  public static class AmpDeviceID {
    public static final int ArmMotorID = 21;
    public static final int LockMotorID = 22;
    public static final int SmallWheelMotorID = 23;
    public static final int BigWheelMotorID = 24;
    public static final int NoteSensorID = 0;
    public static final int ArmPositionSensorID = 0;
  }

  public static class ConvertionParams {
    public static final double ArmMotorGearRatio = 1 / 128.0;
    public static final double SmallWheelGearRatio = 1 / 8.0;
    public static final double BigWheelGearRatio = 1 / 8.0;

    public static final double FALCON_PULSES_PER_ROTATION = 2048;
    public static final double NEO_PULES_PER_REV = 42;
    public static final double PULSE_PER_RAD = FALCON_PULSES_PER_ROTATION / ArmMotorGearRatio / 2 / Math.PI;

  }

  public static class Parameters {
    public static final double Deadband = 0.2;

    public static final double ARM_KP = 0.1;
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
    public static final double SENSOR_TO_REST_DIST = NUMBER_OF_ROTATION / (ConvertionParams.SmallWheelGearRatio);
    public static final double NOTE_VOLTAGE = 2.7;
    public static final double CRITICAL_CURRENT = 0;
    public static final double INTAKE_TRANSFER_POWER = 0.4;
    public static final double INTAKE_POWER = 0.4;
    public static final double INTAKE_RELEASE_POWER = -0.4;
    public static final double INTAKE_PRE_LIMIT_POWER = 0.4;

    // lock data
    public static final double LOCK_CURRENT = 1.2;
    public static final double LOCK_POWER = 0.9;
    public static final double UNLOCK_POWER = 0.4;
    public static final double UNLOCK_TIME = 0.5;

  }

  public static class ArmFFParamaters {
    /*
     * the three numbers in each array is for every state of the arm
     * 
     * 0 = the arm is going up and less then angle 36
     * 1 = the arm is going up and more then angle 36 but less then 72
     * 2 = the arm is going up and more then angle 72 until 110
     * 3 = the arm is going down and more then angle 55
     * 4 = the arm is going down from 55 to 0
     */
    public static final double ARM_UP_KS = 1.287228076;
    public static final double ARM_UP_KV = 0.005323938;
    public static final double ARM_UP_KA = -0.004043448;
    public static final double ARM_Kcos = -1.202533577;
  }

  public static class CommandParams {

    public static final double NUM_OF_ROTATION = 5;
    public static final double v1 = 0.5;
    public static final double v2 = 0.25;

    public static final double ANGLE_RADIANS_AMP = 0;
    public static final double MAX_VEL_RAD = 0;
    public static final double MAX_ACCEL_RAD = 0;
    public static final double ANGLE_RADIANS_CLOSE = 0;

  }

}