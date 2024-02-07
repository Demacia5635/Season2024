package frc.robot;
 
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
 
public final class Constants {
 
 
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
   
  }
}