package frc.robot.subsystems.amp;
public class AmpConstants{
    public static class AmpDeviceID{
      public static final int M1 = 1;      
      public static final int M2 = 2;
      public static final int NEO1 = 3;
      public static final int NEO2 = 4;
      public static final int GYRO = 14;
      public static final int LIGHT_LIMIT = 0;
    }
    public static class ConvertionParams {
      public static final double M1GearRatio = 1/96;// (1/number)
      public static final double NEO1GearRatio = 1/12;// (1/number)
      public static final double NEO2GearRatio = 1/12;// (1/number)

      public static final double MOTOR_PULSES_PER_SPIN = 2048;
      public static final double NEO_PULES_PER_REV = 4096;

      public static final double MOTOR_PULSES_PER_ANGLE = MOTOR_PULSES_PER_SPIN/M1GearRatio/360;
    }
    public static class Parameters{
      public static final double Deadband = 0.2;

      public static final double KP1 = 0.1;
      public static final double KI1= 0.005;
      public static final double KD1 = 0;
    
    }
    public static class armStatesParameters{
      /* the three numbers in each array is for every state of the arm
        * 
        * 0 = the arm is going up and less then angle 36
        * 1 = the arm is going up and more then angle 36 but less then 72
        * 2 = the arm is going up and more then angle 72 until 110
        * 3 = the arm is going down and more then angle 55
        * 4 = the arm is going down from 55 to 0 
        */
        public static final double[] KS = {1.287228076, 0.004391115, 0.009169107, -0.901637521, 0};
        public static final double[] KV = {0.005323938, -0.003403798, 0.005036478, 0.005099339, 0};
        public static final double[] KA = {-0.004043448, 0.000252797, 0.003954434, -0.002012317, 0};
        public static final double[] Kalpha = {0.180088628, 0.005150217, 0.004382342, -0.220649252, 0};
        public static final double[] Ksin = {-15.69829677, 0, 0, 18.36092613, 0};
        public static final double[] Kcos = {-1.202533577, 0, 0, 0.870858412, 0};
        public static final double[] Kcossin = {5.16955116, 0, 0, -5.614001788, 0};
    }


    public static class CommandParams {
    
      public static final double NUM_OF_ROTATION = 5;
      public static final double v1 = 0;
      public static final double v2 = 0;

      public static final double ANGLE_RADIANS_AMP = 0;
      public static final double MAX_VEL_RAD = 0;
      public static final double MAX_ACCEL_RAD = 0;



      
    }

  }