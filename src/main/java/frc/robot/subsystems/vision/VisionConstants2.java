package frc.robot.subsystems.vision;

public  final class VisionConstants2 {

    
  public static final String AmpSideLimelightName = "Amp_Side_Limelight";
  public static final String ShooterSideLimelightName = "Shooter_Side_Limelight";

  public static final double maxValidVelcity = 2.0; // m/s - ignoring vision data abve this velocity
  public static final double maxValidAngleDiff = 10.0; // degrees - ignoring vision data if vision heading is off by more than this value
  public static final double maxDistanceOfCameraFromAprilTag = 4; // meters - ignoring vision data if apriltag is farther than this value

  public static final int numOfCyclesToResetPose = 100;

}