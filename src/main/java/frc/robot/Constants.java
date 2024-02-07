package frc.robot;
 
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
 
public final class Constants {
 
 
 
  public static final double CYCLE_DT = 0.02;
  public static final int CONTROLLER_PORT = 0;
 
  public static class ChassisConstants {
    public final static SwerveModuleConstants MODULE_FRONT_LEFT = new SwerveModuleConstants(
      3, 4, 12,
      new Translation2d(0.332, 0.277),
      257.607421875
    );
    public final static SwerveModuleConstants MODULE_FRONT_RIGHT = new SwerveModuleConstants(
      5, 6, 13,
      new Translation2d(0.332, -0.277),
      290.7
    );
    public final static SwerveModuleConstants MODULE_BACK_LEFT = new SwerveModuleConstants(
      2, 1, 11,
      new Translation2d(-0.332, 0.288),
      230
    );
    public final static SwerveModuleConstants MODULE_BACK_RIGHT = new SwerveModuleConstants(
      8, 7, 14,
      new Translation2d(-0.332, -0.288),
      276.1
    );
    public static final int GYRO_ID = 15;
 
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      MODULE_FRONT_LEFT.moduleTranslationOffset,
      MODULE_FRONT_RIGHT.moduleTranslationOffset,
      MODULE_BACK_LEFT.moduleTranslationOffset,
      MODULE_BACK_RIGHT.moduleTranslationOffset
    );
 
    public static final double MAX_DRIVE_VELOCITY = 3;
    public static final double DRIVE_ACCELERATION = 8;
    public static final double MAX_ANGULAR_VELOCITY = 600;
    public static final double ANGULAR_ACCELERATION = 6000;
 
    public static final double FORWORD_PULSES_PER_METER = 52226.56641604010025062656641604;
    public static final double FORWORD_PULSES_PER_DEGREE = (12.8 * 2048)/360;
 
    // public static final double WHEEL_DIAMETER = (4 * 2.54) / 100;
    // public static final double WHEEL_PERIMETER = WHEEL_DIAMETER * Math.PI;
    // public static final double MK4I_GEAR_RATIO = 8.14;
    // public static final double MOTOR_PULSES_PER_SPIN = 2048;
    // public static final double BACKWARD_PULSES_PER_METER = (1 / (WHEEL_PERIMETER))*MOTOR_PULSES_PER_SPIN * MK4I_GEAR_RATIO;
    // public static final double BACKWARD_PULSES_PER_DEGREE = 12.9047619048;
    public static final double BACKWARD_PULSES_PER_METER = 52226.56641604010025062656641604;
    public static final double BACKWARD_PULSES_PER_DEGREE = ((150/7)*2048)/360;
 
    public static class SwerveModuleConstants {
      public static final double FORWORD_MOVE_KP = 0.05;
      public static final double FORWORD_MOVE_KI = 0;
      public static final double FORWORD_MOVE_KD = 0;
 
      public static final double BACKWARD_MOVE_KP = 0.05;
      public static final double BACKWARD_MOVE_KI = 0;
      public static final double BACKWARD_MOVE_KD = 0;
 
      public static final double FORWORD_ANGLE_POSITION_KP = 0.001;
      public static final double FORWORD_ANGLE_POSITION_KI = 0.0000003;
      public static final double FORWORD_ANGLE_POSITION_KD = 0.0;
      public static final double FORWORD_ANGLE_VELOCITY_KP = 0.2;//0.05/*6.7422E-08*/; //0.07
      public static final double FORWORD_ANGLE_VELOCITY_KI = 0.0; //0.004;
      public static final double FORWORD_ANGLE_VELOCITY_KD = 0;
 
      public static final double BACKWARD_ANGLE_POSITION_KP = 0.0001;
      public static final double BACKWARD_ANGLE_POSITION_KI = 0.000001;
      public static final double BACKWARD_ANGLE_POSITION_KD = 0.0;
      public static final double BACKWARD_ANGLE_VELOCITY_KP = 0.2/*6.7422E-08*/; //0.07
      public static final double BACKWARD_ANGLE_VELOCITY_KI = 0.; //0.004;
      public static final double BACKWARD_ANGLE_VELOCITY_KD = 0;
 
      public static final double FORWORD_MOVE_KS = 0.05; // 0.15851/12; //0.0362;
      public static final double FORWORD_MOVE_KV = 0.263; //0.012314/12; //0.0862;
      public static final double FORWORD_ANGLE_KS = 0.05;//0.52557/12.0; //0.05;
      public static final double FORWORD_ANGLE_KV = 0.0006;//0.003737/12.0; //0.0962;
 
      public static final double BACKWARD_MOVE_KS = 0.05; // 0.15851/12; //0.0362;
      public static final double BACKWARD_MOVE_KV = 0.263; //0.012314/12; //0.0862;
      public static final double BACKWARD_ANGLE_KS = 0.047;//0.52557/12.0; //0.05;
      public static final double BACKWARD_ANGLE_KV = 0.00056;//0.003737/12.0; //0.0962;
 
      public static final double MAX_STEER_ERROR = 5;
 
      public final int moveMotorId;
      public final int angleMotorId;
      public final int absoluteEncoderId;
      public final Translation2d moduleTranslationOffset;
      public final double steerOffset;
 
      public SwerveModuleConstants(int moveMotorId, int angleMotorId, int absoluteEncoderId, Translation2d moduleTranslationOffset, double steerOffset) {
        this.moveMotorId = moveMotorId;
        this.angleMotorId = angleMotorId;
        this.absoluteEncoderId = absoluteEncoderId;
        this.moduleTranslationOffset = moduleTranslationOffset;
        this.steerOffset = steerOffset;
      }
 
    }
  }
  public static class AmpConstants{
    public static class AmpDeviceID{
      public static final int M1 = 1;
      public static final int M2 = 2;
      public static final int GYRO = 14;
      public static final int LIGHT_LIMIT = 0;
    }
    public static class ConvertionParams {
      public static final double m1GearRatio = 1/36;// (1/number)
      public static final double m2GearRatio = 0;// (1/number)
 
      public static final double MOTOR_PULSES_PER_SPIN = 2048;
      public static final double NEO_PULES_PER_REV = 42;
    }
    public static class Parameters{
      public static final double deadband = 0.2;
 
      public static final double KP1 = 0.1;
      public static final double ki1= 0.005;
      public static final double kd1 = 0;
 
      public static final double kp2 = 0;
      public static final double ki2= 0;
      public static final double kd2 = 0;
 
      public static final double ks1 = 0.1;
      public static final double kg1 = 0.05;
      public static final double ka1 = 0.03;
      public static final double kv1 = 0.5;
 
      public static final double ks2 = 0;
      public static final double ka2 = 0;
      public static final double kv2 = 0;
 
    }
  }
 
  public static class IntakeConstants{
    public static class IntakeDeviceID{
      public static final int MOTOR = 24;
      public static final int LIGHT_LIMIT = 0;
    }
    public static class ConvertionParams {
      public static final double MotorGearRatio = 1/1.9; // (1/number)
      public static final double MOTOR_PULSES_PER_SPIN = 2048;
    }
    public static class Parameters{
      public static final double deadband = 0.2;
      public static final boolean inverted = true;
      public static final double notePresenceVoltage = 4.6;


      public static final double intakeSpeed = 1;
      public static double intakeTransferSpeedPreSensor = 0.6;
      public static final double intakeTransferSpeed = 0.45;
      public static final double numFinalRotation = 3.8;
      public static final double CRITICAL_CURRENT = 100;
      public static final double sensorToRestDist = ConvertionParams.MOTOR_PULSES_PER_SPIN*ConvertionParams.MotorGearRatio*numFinalRotation;

      public static final double shootVelocity = 0;
      public static final double shootTime = 0;

      
      public static final double dispenseVelocity = -0.4;
      public static final double dispenseTime = 1;


      public static final double kP = 0.0;
      public static final double kI = 0.000;
      public static final double kD = 0;
 
      public static final double ks1 = 0.0;
      public static final double kg1 = 0.00;
      public static final double ka1 = 0.00;
      public static final double kv1 = 0.0;
    }
  }
}