package frc.robot.subsystems.intake;

public class IntakeConstants {

    public static class IntakeDeviceID{
        public static final int MOTOR = 23;
        public static final int LIGHT_LIMIT = 0;
      }
      public static class ConvertionParams {
        public static final double MotorGearRatio = 1/999; // (1/number)
        public static final double MOTOR_PULSES_PER_SPIN = 2048;
      }
      public static class Parameters{
        public static final double deadband = 0.2;
        public static final boolean inverted = true;
        public static final double notePresenceVoltage = 4.55;
  
  
        public static final double intakeSpeed = 0;
        public static final double intakeTransferSpeed = 0;
        public static final double sensorToRestDist = 0;
  
        public static final double shootVelocity = 0;
        public static final double shootTime = 0;
  
        
        public static final double dispenseVelocity = 0;
        public static final double dispenseTime = 0;
  
  
        public static final double kP = 0.0;
        public static final double kI = 0.000;
        public static final double kD = 0;
   
        public static final double ks1 = 0.0;
        public static final double kg1 = 0.00;
        public static final double ka1 = 0.00;
        public static final double kv1 = 0.0;
      }

    
}
