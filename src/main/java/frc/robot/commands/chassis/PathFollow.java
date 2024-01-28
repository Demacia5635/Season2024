package frc.robot.commands.chassis;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.subsystems.chassis.ChassisConstants.*;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;

import java.util.ArrayList;


import java.util.ArrayList;
import java.util.List;

import frc.robot.PathFollow.Util.Leg;
import frc.robot.PathFollow.Util.RoundedPoint;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.*;
import frc.robot.utils.TrapezoidNoam;

public class PathFollow extends CommandBase {
  
  Chassis chassis;
  RoundedPoint[] corners;
  Pose2d closestAprilTag = new Pose2d();

  Pose2d chassisPose = new Pose2d();
  double distanceOffset = 0.01;
  final double pathLength;

  double totalLeft;
  int segmentIndex = 0;

  Segment[] segments;
  Translation2d vecVel;
  Rotation2d wantedAngle;


  TrapezoidNoam driveTrapezoid;
  TrapezoidNoam rotationTrapezoid;
  
  double driveVelocity = 0;
  double rotationVelocity = 0;

  double distancePassed = 0;
  pathPoint[] points;
  PIDController rotationPidController = new PIDController(0.31, 0.006,0.0000025);

  /** Creates a new path follower using the given points.
   * @param chassis 
   * @param points 
   * @param maxVel the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */
  public PathFollow(Chassis chassis,pathPoint[] points, double maxVel, double maxAcc) {
    Field2d trajField = new Field2d();
    SmartDashboard.putData("Traj", trajField);
    this.points = points;
    //gets the wanted angle for the robot to finish the path in
   
    //creates new coreners array of the "arc points" in the path
    corners = new RoundedPoint[points.length - 2];
    for(int i = 0; i < points.length - 2; i++)
    {
      corners[i] = new RoundedPoint(points[i], points[i+1], points[i+2]);
    }
    this.chassis = chassis;
    addRequirements(chassis);
    SmartDashboard.putData(this);

    //creates trapezoid object for drive and rotation
    driveTrapezoid = new TrapezoidNoam(maxVel, maxAcc);
    rotationTrapezoid = new TrapezoidNoam(180, 360);

    //calculate the total length of the path
    segments = new Segment[1 + ((points.length - 2) * 2)];

    //case for 1 segment, need to create only 1 leg
    if(points.length < 3) {
      
      segments[0] = new Leg(points[0].getTranslation(), points[1].getTranslation(), points[1].isAprilTag());
      System.out.println("------LESS THAN 3------");
    }
    //case for more then 1 segment
    else 
    {
      //creates the first leg
      segments[0] = corners[0].getAtoCurveLeg();
    
      int segmentIndexCreator = 1;
      //creates arc than leg
      for(int i = 0; i < corners.length - 1; i +=1)
      {
        segments[segmentIndexCreator] = corners[i].getArc(); 
        segments[segmentIndexCreator+1] = new Leg(corners[i].getCurveEnd(), corners[i+1].getCurveStart(), points[segmentIndexCreator].isAprilTag());
        segmentIndexCreator+=2;
      }
      //creates the last arc and leg
      segments[segments.length - 2] = corners[corners.length - 1].getArc();
      segments[segments.length - 1] = corners[corners.length - 1].getCtoCurveLeg();
    }

    //calculates the length of the entire path
    double segmentSum = 0;
    for (Segment s : segments) {
      segmentSum += s.getLength();
    }
    pathLength = segmentSum;
    totalLeft = pathLength;


    ArrayList <Translation2d> list = new ArrayList<>();
    for(int i = 0; i < segments.length; i ++){
      Translation2d[] pointsForView = segments[i].getPoints();
      for(int j = 0; j < pointsForView.length; j ++){
        list.add(pointsForView[j]);
      }
    }
    edu.wpi.first.math.trajectory.Trajectory traj = TrajectoryGenerator.generateTrajectory(points[0], list, points[points.length - 1], null);
    trajField.getObject("Trajectory").setTrajectory(traj);
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

  //calculates the position of the closet april tag and returns it's position
  public Pose2d getClosestAprilTag(){
    Translation2d finalVector = new Translation2d(Integer.MAX_VALUE, Integer.MAX_VALUE);
    int finalAprilTagIndex = 0;
    //checks the distance from each april tag and finds
    for(int i = 0; i < 8; i++){
      

      Translation2d currentAprilTagVector = chassis.getPose().minus(aprilTagsPositions[i]).getTranslation();

     if(currentAprilTagVector.getNorm() < finalVector.getNorm()){
      finalAprilTagIndex = i;
      finalVector = currentAprilTagVector;
     }
      
    }
  
    return new Pose2d(finalVector, aprilTagsPositions[finalAprilTagIndex].getRotation());
  }


  @Override
  public void execute() {

    
    chassisPose = chassis.getPose();
    
    //current velocity vector
    Translation2d currentVelocity = new Translation2d(chassis.getChassisSpeeds().vxMetersPerSecond, chassis.getChassisSpeeds().vyMetersPerSecond);
    distancePassed = totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation());
    
    if(segments[segmentIndex].distancePassed(chassisPose.getTranslation()) >= segments[segmentIndex].getLength() - distanceOffset){
      totalLeft -= segments[segmentIndex].getLength();
      if(segmentIndex != segments.length - 1 || segments[segmentIndex].getLength() <= 0.15)
        segmentIndex++;  
    }
    wantedAngle = points[segmentIndex].getRotation();
    Translation2d velVector = segments[segmentIndex].calc(chassisPose.getTranslation(), driveVelocity);

    driveVelocity = driveTrapezoid.calculate(totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation()),
     currentVelocity.getNorm(), 0);
    System.out.println("APRILTAG MODE: " + segments[segmentIndex].isAprilTagMode());
    if(segments[segmentIndex].isAprilTagMode())
    {
      velVector = new Translation2d(velVector.getX() / 10, velVector.getY() / 10);
      wantedAngle = Rotation2d.fromDegrees(180   /*getClosestAprilTag().getRotation().getDegrees()*/);

    }

    else{
      wantedAngle = points[segmentIndex].getRotation();
    }

    rotationVelocity = rotationPidController.calculate(
      chassis.getAngle().getRadians(), wantedAngle.getRadians()) * MAX_OMEGA_VELOCITY;
      

    if(totalLeft <= 0.01) velVector = new Translation2d(0, 0);
    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX(), velVector.getY(), rotationVelocity);
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
    return totalLeft <= 0.01 && Math.abs(chassis.getAngle().getDegrees() - wantedAngle.getDegrees()) <= 0.5;
  }


  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addStringProperty("Current Segment",() -> currentSegmentInfo(), null);
      builder.addDoubleProperty("Distance Passed",() -> {return distancePassed;}, null);
      builder.addDoubleProperty("Total Left",() -> {return totalLeft;}, null);
      builder.addDoubleProperty("Velocity", () -> {return driveVelocity;}, null);
      builder.addDoubleProperty("Rotation Velocity", () -> {return Math.toDegrees(rotationVelocity);}, null);
      builder.addDoubleProperty("Angle", () -> {return chassisPose.getRotation().getDegrees();}, null);
      builder.addDoubleProperty("Pose X", ()-> chassis.getPose().getX(), null);
      builder.addDoubleProperty("Pose Y", ()-> chassis.getPose().getY(), null);
  }

  public void printSegments()
  {
    for (Segment s : segments) {
      System.out.println(s); 
    }
  }
}
