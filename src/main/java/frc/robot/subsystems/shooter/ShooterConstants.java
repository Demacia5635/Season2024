package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**the constants of the shooter */
public class ShooterConstants {

    
    /**ids for motors and sensors */
    public static class ShooterID {
        /*set up the id for all the motors */
        public static final int MOTOR_ANGLE_ID = 33;
        public static final int MOTOR_UP_ID = 31;
        public static final int MOTOR_DOWN_ID = 32;
        public static final int MOTOR_FEEDING_ID = 34;
        
        /*set up the id of the sensors */
        public static final int ANALOG_INPUT_ID = 1;
        public static final int LIMIT_SWITCH_ID = 1;
    }
    
    /**var of the shooter */
    public static class ShooterVar {
        /*vars that were found */
        public static final double VOLT_NOTE_PRESENT = 4.7;
        public static final double PEREMITER_OF_WHEEL = 0.075 * Math.PI;
        
        /*vars that were tested */
        public static final double WANTED_ANGLE_CLOSE = 54;
        public static final double WANTED_VEL_CLOSE = 14;
        
        /*constant var */
        public static final int PULES_PER_REV = 2048;
    }

    /**vars for the angle changer of the shooter */
    public static class AngleChanger {
        /*motor var */
        public static final int GEAR_RATIO = 2;
        public static final int REV_PER_MM = 8;
        public static final int PULES_PER_MM = ShooterVar.PULES_PER_REV * GEAR_RATIO / REV_PER_MM;
        
        /*the length of profiles in the angle changer */
        public static final double KA = 136;
        public static final double KB = 128;
        
        /*set up the feedForward var of the angle changer */
        public static final double KS = 0.056673856189171;
        public static final double KG = 0;
        public static final double KV = 0.000091655292302;
        
        /*set up the PID of the angle motor */
        public static final double KP = 0.3;
        public static final double KD = KP / 100;
    
        /*the max and min dis of the angle changer */
        public static final double MAX_DIS = 225;
        public static final double MIN_DIS = 115;

        public static final double BOUNDARY_DIS = 4;
    }

    /**vars for shooting */
    public static class Shooting {
        /*shooting feed forward */
        public static final double KS = 0.1510682394443;
        public static final double KV = 0.016769985504258;
        public static final double KV2 = 0.001011134688171;

        /*shooting pid */
        public static final double KP = 0.02;
        public static final double KI = 0.0007;
    }


    /**set up the lookup table var */
    public static class LookUpTableVar {
        public static final double[][] lookUpTable = {
            {1.7, 53, 16.7}, {1.96, 47.5, 17}, {2.5, 40, 17.5}, 
            {3, 35.1, 19},  {3.7, 37, 17.5}, {4.65, 34, 18}
              
        };
    }

    /*var for shooting to the amp */
    public static class AmpVar {
        public static final double ANGLE = 48.5;
        public static final double DOWN = 19;
        public static final double UP = 4.5;
    }

    /**var for shooting from the podium */
    public static class PodiumVar {
        public static final double UP = 16;
        public static final double DOWN = 16;
        public static final double ANGLE = 41;
        
    }
    
    /*set up the location of the 2 speakers */
    public static class Speaker {
        public static final Pose2d BLUE_ALLIANCE_SPEAKER = new Pose2d(-0.04,5.55, Rotation2d.fromDegrees(0));
        public static final Pose2d RED_ALLIANCE_SPEAKER = new Pose2d(16.54 - (-0.04),2.1842, Rotation2d.fromDegrees(180));
    }

    /**
     * <pre>
     * enum SHOOTER_MOTOR that contains all motors of the shooter
     * UP - the shooting motor thats up
     * DOWN - the shooting motor thats down
     * FEEDING - the feeding motor
     * ANGLE - the motor of the angle changer
     * </pre>
     */
    public enum SHOOTER_MOTOR {
        UP,
        DOWN,
        FEEDING,
        ANGLE,
    }

    public enum SHOOTER_MODE {
        IDLE, 
        AMP, 
        PODIUM, 
        SUBWOOFER, 
        AUTO, 
        AUTO_CONTINIOUS
    }
}
