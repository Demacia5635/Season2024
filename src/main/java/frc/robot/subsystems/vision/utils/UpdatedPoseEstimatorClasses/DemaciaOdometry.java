package frc.robot.subsystems.vision.utils.UpdatedPoseEstimatorClasses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.WheelPositions;
import frc.robot.subsystems.chassis.ChassisConstants;
import frc.robot.subsystems.chassis.utils.SwerveKinematics;

public class DemaciaOdometry {
    private Pose2d pose;

    private SwerveKinematics kinematics;
    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;
    private SwerveDriveWheelPositions m_previousWheelPositions;
    
    public DemaciaOdometry(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d initialPoseMeters) {
        this.pose = initialPoseMeters;
        this.kinematics = ChassisConstants.KINEMATICS_CORRECTED;

        m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
        m_previousAngle = initialPoseMeters.getRotation();
        m_previousWheelPositions = new SwerveDriveWheelPositions(wheelPositions);
    }

    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
        pose = poseMeters;
        m_previousAngle = pose.getRotation();
        m_gyroOffset = pose.getRotation().minus(gyroAngle);
        m_previousWheelPositions = new SwerveDriveWheelPositions(wheelPositions);
    }

    public Pose2d getPose(){
        return pose;
    }

    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        Rotation2d angle = gyroAngle.plus(m_gyroOffset);
        SwerveDriveWheelPositions cur = new SwerveDriveWheelPositions(wheelPositions);

        Twist2d twist = kinematics.toTwist2d(m_previousWheelPositions, cur);

        twist.dtheta = angle.minus(m_previousAngle).getRadians();
        Pose2d newPose = exp(pose, twist); //need to test
        m_previousWheelPositions = cur;
        m_previousAngle = angle;
        pose = new Pose2d(newPose.getTranslation(), angle);

        return pose;
    }

    private Pose2d exp(Pose2d pose, Twist2d twist){

        return new Pose2d(pose.getX() + twist.dx, pose.getY() + twist.dy, pose.getRotation().plus(Rotation2d.fromDegrees(twist.dtheta)));


    }


}