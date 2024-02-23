package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {

    /*set up the id for all the motors */
    public static final int MOTOR_ANGLE_ID = 33;
    public static final int MOTOR_UP_ID = 31;
    public static final int MOTOR_DOWN_ID = 32;
    public static final int MOTOR_FEEDING_ID = 34;
    
    /*set up the id of the sensors */
    public static final int ANALOG_INPUT_ID = 1;
    public static final int LIMIT_SWITCH_ID = 1;
    
    /*set up the location of the 2 speakers */
    public static final Pose2d BLUE_ALLIANCE_SPEAKER = new Pose2d(-0.04,5.55, Rotation2d.fromDegrees(0));
    //TODO fix it impossible 
    public static final Pose2d RED_ALLIANCE_SPEAKER = new Pose2d(6.5273,2.1842, Rotation2d.fromDegrees(180));

    /*motor falcon stats */
    public static final int PULES_PER_REV = 2048;
    
    
    public static class AngleChanger {
        public static final int GEAR_RATIO = 2;
        public static final int REV_PER_MM = 8;
        public static final int PULES_PER_MM = PULES_PER_REV * GEAR_RATIO / REV_PER_MM;
        
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
    
        public static final double MAX_DIS = -246;
        public static final double MIN_DIS = 100;
    }
    
    public static final double VOLT_NOTE_PRESENT = 4.7;

    public static class Shooting {
        public static final double KS = 0.053395329232339;
        public static final double KV = 0.034360975093757;
        public static final double KV2 = 0.000234209075695;

        public static final double KP = 0.117977489346903;
    }

    public static final double PEREMITER_OF_WHEEL = 0.075 * Math.PI;
    public static final double ANGLE_CALIBRATE = 20;

    /**set up var for pow */
    public static class CommandParams {

        public static final double ANGLE_COLLECT = 0;
        public static final double MAX_VEL = 0;
        public static final double MAX_ACCCEL = 0;
        public static final double ANGLE_DEFAULT = 0;
        public static final double FEED_POWER = 0;
        public static final double SHOOT_POWER = 0;
        public static final double ANGLE_SHOOT = 0;
    }

    /**set up the lookup table var */
    public static class LookUpTable {
        public static final double[][] lookUpTable = {
            {1.7, 53, 16.7}, {1.96, 47.5, 17}, {2.5, 40, 17.5}, 
            {3, 35.1, 19},  {3.7, 37, 17.5}, {4.65, 34, 18}
              
        };
    }

    public static double WANTED_ANGLE_CLOSE = 54;
    public static double WANTED_VEL_CLOSE = 14;

    public static class AmpPera {
        public static final double ANGLE = 44;
        public static final double DOWN = 14;
        public static final double UP = 4;
    }

}
