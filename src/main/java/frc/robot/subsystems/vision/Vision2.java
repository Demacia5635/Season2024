package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.vision.utils.VisionData;
import frc.robot.subsystems.vision.utils.UpdatedPoseEstimatorClasses.SwerveDrivePoseEstimator;

public class Vision2 extends SubsystemBase {
    private NetworkTable limelight;
    private Field2d noFilterVisionField;
    private Field2d visionFieldavg5;
    private SwerveDrivePoseEstimator poseEstimator;
    private Chassis chassis;
    private VisionData[] buf5Avg;
    private int lastData5;
    private double lastUpdateTime5;

    public Vision2(Chassis chassis, SwerveDrivePoseEstimator estimator) {
        this.chassis = chassis;
        this.poseEstimator = estimator;
        this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
        this.noFilterVisionField = new Field2d();
        this.visionFieldavg5 = new Field2d();
        this.lastData5 = -1;
        this.buf5Avg = new VisionData[5];
        for (int i =  0; i < buf5Avg.length; i++) {
            this.buf5Avg[i] = new VisionData(null,  0, poseEstimator);
        }
        SmartDashboard.putData("no Filter vision field", noFilterVisionField);
        SmartDashboard.putData("vision Avg   5 field", visionFieldavg5);
    }

    private void updateVisionData() {
        double tx = limelight.getEntry("tx").getDouble(0);
        double ty = limelight.getEntry("ty").getDouble(0);
        double ta = limelight.getEntry("ta").getDouble(0);

        Rotation2d rotation = Rotation2d.fromDegrees(ta);
        Pose2d robotPose = new Pose2d(new Translation2d(tx, ty), rotation);
        noFilterVisionField.setRobotPose(robotPose);
    }

    public void updateRobotPose5() {
        double time = Timer.getFPGATimestamp();
        if (validBuf5(time)) {
            Pair<Pose2d, Double> vData5AvgPair = avg(buf5Avg);
            VisionData vDataAvg5 = new VisionData(vData5AvgPair.getFirst(), vData5AvgPair.getSecond(), poseEstimator);
            if (vDataAvg5.getPose() != null) {
                poseEstimator.addVisionMeasurement(new Pose2d(vDataAvg5.getPose().getTranslation(), chassis.getAngle()), vDataAvg5.getTimeStamp());
                visionFieldavg5.setRobotPose(vDataAvg5.getPose());
                lastUpdateTime5 = time;
                for (VisionData vd : buf5Avg) {
                    vd.clear();
                }
            }
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        updateVisionData();
        updateRobotPose5();
    }

    private Pair<Pose2d, Double> avg(VisionData[] visionDataArr) {
        // Implement averaging logic here
        // This is a placeholder. You need to calculate the average pose and timestamp based on your vision data buffer.
        return null;
    }

    private boolean validBuf5(double time) {
        // Implement buffer validation logic here
        // This is a placeholder. You need to check if the buffer is valid based on your criteria.
        return false;
    }

    private int next5() {
        return (lastData5 +  1) % buf5Avg.length;
    }

    public double getTime() {
        return Timer.getFPGATimestamp();
    }
}
