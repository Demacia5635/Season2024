package frc.robot.commands.chassis;
import edu.wpi.first.math.controller.PIDController;
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
  Field2d trajField;

  TrapezoidNoam driveTrapezoid;
  TrapezoidNoam rotationTrapezoid;
  
  double driveVelocity = 0;
  double rotationVelocity = 0;

  double distancePassed = 0;
  pathPoint[] points;
  PIDController rotationPidController = new PIDController(0.01, 0.002,0.000002);

  /** Creates a new path follower using the given points.
   * @param chassis 
   * @param points 
   * @param maxVel the max velocity in m/s
   * @param maxAccel the max accel in m/s2 (squared)
   * 
   */
  public PathFollow(Chassis chassis,pathPoint[] points, double maxVel, double maxAcc) {
    
    this.points = points;
    //gets the wanted angle for the robot to finish the path in
    wantedAngle = points[points.length - 1].getRotation();
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
      segments[0] = new Leg(points[0].getTranslation(), points[1].getTranslation(), false);
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
        segments[segmentIndexCreator+1] = new Leg(corners[i].getCurveEnd(), corners[i+1].getCurveStart(), false);
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
    System.out.println("wanted angle: " + wantedAngle.getDegrees());
    
    chassisPose = chassis.getPose();
    
    //current velocity vector
    Translation2d currentVelocity = new Translation2d(chassis.getChassisSpeeds().vxMetersPerSecond, chassis.getChassisSpeeds().vyMetersPerSecond);
    distancePassed = totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation());
    
    if(segments[segmentIndex].distancePassed(chassisPose.getTranslation()) >= segments[segmentIndex].getLength() - distanceOffset){
      totalLeft -= segments[segmentIndex].getLength();
      if(segmentIndex != segments.length - 1 || segments[segmentIndex].getLength() <= 0.15)
        segmentIndex++;  
    }

    driveVelocity = driveTrapezoid.calculate(totalLeft - segments[segmentIndex].distancePassed(chassisPose.getTranslation()),
     currentVelocity.getNorm(), 0);

/*  if(segments[segmentIndex].isAprilTagMode()) rotationVelocity = rotationTrapezoid.calculate(
      (wantedAngle.minus(chassis.getAngle())).getDegrees(),
      Rotation2d.fromRadians( chassis.getChassisSpeeds().omegaRadiansPerSecond).getDegrees(), 0);

    else*/
    rotationVelocity = rotationPidController.calculate(chassis.getAngle().getRadians(), wantedAngle.getRadians()) * MAX_OMEGA_VELOCITY;

    Translation2d velVector = segments[segmentIndex].calc(chassisPose.getTranslation(), driveVelocity);
    System.out.println("ROTATION VELOCITY: " + Math.toDegrees( rotationVelocity));
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
    return totalLeft <= 0.01 ;
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
