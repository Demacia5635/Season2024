package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static class ArmConstants{
        /*
        * motor id of the arm
        * and the id of the limit swtich to the arm
        */
        public static final int motorID = 30;
        public static final int digitalInputID = 0;

        /*
        * get the pules per angle through math
        * use as the get angle and angelar velocity
        */
        private static final double pulesPerRotation = 2048;
        private static final double gearRatio = 180;
        public static final double pulsePerAngle = pulesPerRotation*gearRatio/360;


        /* the four numbers in each array is for every state of the arm
        * 
        * 0 = the arm is going forward and less then angle 43
        * 1 = the arm is going forward and more then angle 43
        * 2 = the arm is going backward and more then angle 43
        * 3 = the arm is going backward and less then angle 43
        */
        public static final double[] KS = {1.287228076, 0.004391115, 0.009169107, -0.901637521};
        public static final double[] KV = {0.005323938, -0.003403798, 0.005036478, 0.005099339};
        public static final double[] KA = {-0.004043448, 0.000252797, 0.003954434, -0.002012317};
        public static final double[] Kalpha = {0.180088628, 0.005150217, 0.004382342, -0.220649252};
        public static final double[] Ksin = {-15.69829677, 0, 0, 18.36092613};
        public static final double[] Kcos = {-1.202533577, 0, 0, 0.870858412};
        public static final double[] Kcossin = {5.16955116, 0, 0, -5.614001788};
        
    }
    
    public static class GripperConstants {

        public static final int motorID = 20;

        public static final double openPower = 1;
        public static final double closePower = -1;
    }

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
      public static final double FORWORD_ANGLE_VELOCITY_KP = 0.08;//0.05/*6.7422E-08*/; //0.07
      public static final double FORWORD_ANGLE_VELOCITY_KI = 0.0; //0.004;
      public static final double FORWORD_ANGLE_VELOCITY_KD = 0;

      public static final double BACKWARD_ANGLE_POSITION_KP = 0.0001;
      public static final double BACKWARD_ANGLE_POSITION_KI = 0.000001;
      public static final double BACKWARD_ANGLE_POSITION_KD = 0.0;
      public static final double BACKWARD_ANGLE_VELOCITY_KP = 0.0/*6.7422E-08*/; //0.07
      public static final double BACKWARD_ANGLE_VELOCITY_KI = 0.; //0.004;
      public static final double BACKWARD_ANGLE_VELOCITY_KD = 0;

      public static final double FORWORD_MOVE_KS = 0.05; // 0.15851/12; //0.0362;
      public static final double FORWORD_MOVE_KV = 0.263; //0.012314/12; //0.0862;
      public static final double FORWORD_ANGLE_KS = 0.035;//0.52557/12.0; //0.05;
      public static final double FORWORD_ANGLE_KV = 0.0003;//0.003737/12.0; //0.0962;

      public static final double BACKWARD_MOVE_KS = 0.05; // 0.15851/12; //0.0362;
      public static final double BACKWARD_MOVE_KV = 0.263; //0.012314/12; //0.0862;
      public static final double BACKWARD_ANGLE_KS = 0.0773;//0.52557/12.0; //0.05;
      public static final double BACKWARD_ANGLE_KV = 0.0005;//0.003737/12.0; //0.0962;

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

  public static class LedConstants {
    public static final int LED_ID = 0;
      public static final int LED_COUNT = 171;
  }
}
