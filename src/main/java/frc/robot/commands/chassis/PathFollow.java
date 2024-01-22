// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;




import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.subsystems.chassis.ChassisConstants.*;
import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.TrapezShay;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.*;
import frc.robot.utils.TrapezoidNoam;

public class PathFollow extends CommandBase {
  Pose2d closestAprilTag = new Pose2d();
  Chassis chassis;
  RoundedPoint[] corners;

  Pose2d chassisPose = new Pose2d();
  double distanceOffset = 0.01;
  final double pathLength;

  double totalLeft;
  int segmentIndex = 0;

  Segment[] segments;
  Translation2d vecVel;
  Rotation2d wantedAngle;
  Field2d trajField;

  TrapezoidNoam driveTrapezoid;
  TrapezoidNoam rotationTrapezoid;
  double velocity = 0;
  double rotationVelocity = 0;
  double safeVel = 1;
  double distancePassed = 0;

  /** Creates a new ArcPath.
   * @param chassis 
   * @param point Translation2d array of points for the path
   * @param radius double array of radius (for each turn)
   * @param maxVel the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */
  
  public PathFollow(Chassis chassis,pathPoint[] points, double maxVel, double maxAcc) {
    

    wantedAngle = points[points.length - 1].getRotation();
    corners = new RoundedPoint[points.length - 2];
    for(int i = 0; i < points.length - 2; i++)
    {
      corners[i] = new RoundedPoint(points[i], points[i+1], points[i+2]);
    }
    this.chassis = chassis;
    addRequirements(chassis);

    trajField = new Field2d();
    



    SmartDashboard.putData(this);

    driveTrapezoid = new TrapezoidNoam(maxVel, maxAcc);
    rotationTrapezoid = new TrapezoidNoam(180, 360);

    //calculate the total length of the path

    segments = new Segment[1 + ((points.length - 2) * 2)];

    //case for only 1 segment, only leg
    if(points.length < 3) {
      segments[0] = new Leg(points[0].getTranslation(), points[1].getTranslation(), false);
      System.out.println("------LESS THAN 3------");
    }
    else 
    {
      segments[0] = corners[0].getAtoCurveLeg();
    
      int segmentIndexCreator = 1;
      for(int i = 0; i < corners.length - 1; i +=1)
      {
        segments[segmentIndexCreator] = corners[i].getArc(); 
        segments[segmentIndexCreator+1] = new Leg(corners[i].getCurveEnd(), corners[i+1].getCurveStart(), false);
        segmentIndexCreator+=2;
      }
      segments[segments.length - 2] = corners[corners.length - 1].getArc();
      segments[segments.length - 1] = corners[corners.length - 1].getCtoCurveLeg();
    }


    double segmentSum = 0;
    for (Segment s : segments) {
      segmentSum += s.getLength();
    }
    pathLength = segmentSum;
    totalLeft = pathLength;

    

    printSegments();

    //segments[0] = new Leg(null, null);


  }


  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addStringProperty("Current Segment",() -> currentSegmentInfo(), null);
      builder.addDoubleProperty("Distance Passed",() -> {return distancePassed;}, null);
      builder.addDoubleProperty("Total Left",() -> {return totalLeft;}, null);
      builder.addDoubleProperty("Velocity", () -> {return velocity;}, null);
      builder.addDoubleProperty("Rotation Velocity", () -> {return rotationVelocity;}, null);
      builder.addDoubleProperty("Angle", () -> {return chassisPose.getRotation().getDegrees();}, null);
      builder.addDoubleProperty("Pose X", ()-> chassis.getPose().getX(), null);
      builder.addDoubleProperty("Pose Y", ()-> chassis.getPose().getY(), null);
  }


  public String currentSegmentInfo()
  {
    
    if(segments == null)
      return "";
    return segments[segmentIndex].toString();
  }

  @Override
  public void initialize() {
    driveTrapezoid.debug = true;
    // chassis.useAcceleration = false;
    totalLeft = pathLength;

    segmentIndex = 0;

    vecVel = new Translation2d(0,0);  
  }

  /*public Pose2d getClosestAprilTag(){
    Translation2d finalVector = new Translation2d();
    int finalAprilTagIndex = 0;
    for(int i = 0; i < 8; i++){
      
      Translation2d currentAprilTagVector =chassis.getPose().getTranslation()
      .minus(Constants.aprilTagsPositions[i].getTranslation());

     if(currentAprilTagVector.getNorm() < finalVector.getNorm()){
      finalAprilTagIndex++;
      finalVector = currentAprilTagVector;
     }
      
    }
    System.out.println("CLOSET APRILTAG: " +  new Pose2d(finalVector, Constants.aprilTagsPositions[finalAprilTagIndex].getRotation()));
    return new Pose2d(finalVector, Constants.aprilTagsPositions[finalAprilTagIndex].getRotation());
  } */


  @Override
  public void execute() {
    chassisPose = chassis.getPose();
    Translation2d currentVelocity = new Translation2d(chassis.getChassisSpeeds().vxMetersPerSecond, chassis.getChassisSpeeds().vyMetersPerSecond);
    distancePassed = totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation());
    
    if(segments[segmentIndex].distancePassed(chassisPose.getTranslation()) >= segments[segmentIndex].getLength() - distanceOffset){
      totalLeft -= segments[segmentIndex].getLength();
      if(segmentIndex != segments.length - 1 || segments[segmentIndex].getLength() <= 0.15)
        segmentIndex++;  
    }

    velocity = driveTrapezoid.calculate(totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation()),
     currentVelocity.getNorm(), 0);

    //if(!segments[segmentIndex].isAprilTagMode())
    rotationVelocity = rotationTrapezoid.calculate((wantedAngle.minus(chassis.getAngle())).getDegrees(), Rotation2d.fromRadians( chassis.getChassisSpeeds().omegaRadiansPerSecond).getDegrees(), 0);
    /*System.out.println("ROTATION VELOCITY: " + rotationVelocity);
    System.out.println("WANTED ANGLE: 90");
    System.out.println("Current Angle: " + chassis.getAngle());
    System.out.println("DISTANCE LEFT: " + (new Rotation2d(Math.PI / 2.0).minus(chassis.getAngle()).getRadians()));
    /*else
      rotationVelocity = rotationTrapezoid.calc(
        getClosestAprilTag().getTranslation().minus(chassis.getPose().getTranslation())
        .getAngle()
          .minus(
            chassis.getAngle()).getDegrees(),
            Math.toDegrees(chassis.getChassisSpeeds().omegaRadiansPerSecond));
    */
    Translation2d velVector = segments[segmentIndex].calc(chassisPose.getTranslation(), velocity);
    System.out.println(" vel vector=" + velVector + " v=" + velVector.getNorm());
    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX(), velVector.getY(), 0   /*Math.toRadians(rotationVelocity)*/);
    chassis.setVelocities(speed);
  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    driveTrapezoid.debug = false;
    //.useAcceleration = true;
  }

  @Override
  public boolean isFinished() {
    return totalLeft <= 0.01;
  }

  public void printSegments()
  {
    for (Segment s : segments) {
      System.out.println(s); 
    }
  }
}
