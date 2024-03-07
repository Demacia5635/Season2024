package frc.robot.subsystems.vision.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Utility class for vision
 */
public class LimelightVisionUtils {

    public enum LimeLight {
        LimeLight2,
        LimeLight3
    }
    public static final NetworkTable LIMELIGHT_AMP_TABLE = NetworkTableInstance.getDefault().getTable("limelight-amp");
    public static final NetworkTable LIMELIGHT_SHOOTER_TABLE = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    public static final double MAX_DISTANCE_FOR_LIMELIGHT = 6;

    public static NetworkTable[] LimeLightTables = {LIMELIGHT_AMP_TABLE, LIMELIGHT_SHOOTER_TABLE};


    /**
     * Gets the pose of the robot from the vision system
     * 
     * @return The pose of the robot from the vision system, and the timestamp of
     *         the measurement, or null if no target is found
     */
    public static Pair<Pose2d, Double> getVisionPose(int camera) {
        return getVisionPose(LimeLightTables[camera]);
    }
    public static Pair<Pose2d, Double> getVisionPose(NetworkTable limelightTable) {
        double timeStamp = Timer.getFPGATimestamp();
        double hasTarget = limelightTable.getEntry("tv").getDouble(0);
        if (hasTarget == 0)
            return null;

        double[] robotPose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        // data
        // [0-X, 1-Y, 2-Z, 3-Roll, 4-Pitch, 5-Yaw, 6-Latecy, 7-Tag Count,8-Tag Span, 9-Distance, 10-Tag Area]
        if (robotPose.length <7)
            return null;

        double latency = robotPose[6]/1000.0;
        Rotation2d robotRotation = Rotation2d.fromDegrees(robotPose[5]);
        Translation2d robotTranslation = new Translation2d(robotPose[0], robotPose[1]);
        double distance =  (robotPose.length >= 9) ? robotPose[9]: 1;
        //double distance =  robotPose[9];

       
        if(distance > MAX_DISTANCE_FOR_LIMELIGHT || distance == 0){
            return null;
        }

//        if(limelightTable.g  etEntry("ti").getDouble(0))) 
        return new Pair<Pose2d, Double>(
                new Pose2d(robotTranslation, robotRotation),
                timeStamp - latency);
    }
}