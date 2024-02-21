package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public  final class VisionConstants {

    
  public static final String AmpSideRaspberryName = "Amp_Side_Raspberry";
  public static final String ShooterSideRaspberryName = "Shooter_Side_Raspberry";


  public static final Transform3d robotCenterToAmpSideRaspberry = new Transform3d(0.21, -0.3, 0.49, 
  new Rotation3d(0, Math.toRadians(40), Math.toRadians(180)));

  public static final Transform3d robotCenterToShooterSideRaspberry = new Transform3d(-0.33, -0.23, 0.26, 
  new Rotation3d(0, Math.toRadians(45), 0));

  public static final double maxValidVelcity = 2.0; // m/s - ignoring vision data abve this velocity
  public static final double maxValidAngleDiff = 10.0; // degrees - ignoring vision data if vision heading is off by more than this value
  public static final double maxDistanceOfCameraFromAprilTag = 4; // meters - ignoring vision data if apriltag is farther than this value

  public static final int numOfCyclesToResetPose = 100;

}