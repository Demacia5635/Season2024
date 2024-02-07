package frc.robot.subsystems.intake;

public class IntakeConstants {

    public static class IntakeDeviceID{
        public static final int MOTOR = 23;
        public static final int LIGHT_LIMIT = 0;
      }
      public static class ConvertionParams {
        public static final double MOTOR_GEAR_RATIO = 1/999; // (1/number)
        public static final double MOTOR_PULSES_PER_SPIN = 2048;
      }
      public static class Parameters{
        public static final double DEADBAND = 0.2;
        public static final boolean IS_INVERTED = true;
        public static final double NOT_PRESENCE_VOLTAGE = 4.55;
  
  
        public static final double INTAKE_SPEED = 0;
        public static final double INTAKE_TRANSFER_SPEED = 0;
        public static final double SENSOR_TO_REST_DIST = 0;
  
        public static final double SHOOT_VELOCITY = 0;
        public static final double SHOOT_TIME = 0;
  
        
        public static final double DISPENSE_VELOCITY = 0;
        public static final double DISPENSE_TIME = 0;
  
  
        public static final double KP = 0.0;
        public static final double KI = 0.000;
        public static final double KD = 0;
   
        public static final double KS1 = 0.0;
        public static final double KG1 = 0.00;
        public static final double KA1 = 0.00;
        public static final double KV1 = 0.0;
      }

    
}
