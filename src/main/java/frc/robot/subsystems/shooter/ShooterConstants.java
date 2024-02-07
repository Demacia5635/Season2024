package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {

        /*set up the id for all the motors */
        public static final int MOTOR_ID = 3;
        public static final int MOTOR_1_ID = 1;
        public static final int MOTOR_2_ID = 2;
        public static final int MOTOR_FEEDING_ID = 4;
        
        /*set up the id of the sensor */
        public static final int LIMIT_INPUT_ID = 5;
        
        /*set up the location of the 2 speakers */
        public static final Pose2d BLUE_ALLIANCE_SPEAKER_POSE2D = new Pose2d(-0.15,2.1842, Rotation2d.fromDegrees(0));
        public static final Pose2d RED_ALLIANCE_SPEAKER_POSE2D = new Pose2d(6.5273,2.1842, Rotation2d.fromDegrees(180));

        /*motor falcon stats */
        public static final int PULES_PER_REV = 2048;
        public static final int GEAR_RATIO = 4;
        public static final int REV_PER_MM = 8;
        public static final int PULES_PER_MM = PULES_PER_REV * GEAR_RATIO / REV_PER_MM;
        
        /*the length of profiles in the angle changer */
        public static final double KA = 165;
        public static final double KB = 161;
        
        /*set up the feedForward var of the angle changer */
        public static final double KS = 0.056673856189171;
        public static final double KG = 0;
        public static final double KV = 0.000091655292302;
        
        /*set up the PID of the angle motor */
        public static final double KP = 0.3;
        public static final double KD = KP / 100;
    }
