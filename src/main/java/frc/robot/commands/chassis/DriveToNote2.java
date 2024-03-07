// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;


public class DriveToNote2 extends Command {
  /** Creates a new DriveToPoint. */
  Chassis chassis;
  
  double remaningDistance;
  double angleToPoint;
  double selfAngleRotationRemainging;
  double intakeAngleRelativeToRobot = 0; 


  double maxVel;
  double velX;
  double velY;
  double rotVel;
  ChassisSpeeds speed;


  public DriveToNote2(Chassis chassis, double maxVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.maxVel = maxVel;
    this.chassis = chassis;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double[] distanceAndAngleToNote = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    remaningDistance = distanceAndAngleToNote[0];
    angleToPoint = distanceAndAngleToNote[1];
    selfAngleRotationRemainging = distanceAndAngleToNote[1] - intakeAngleRelativeToRobot;

    velX = maxVel * Math.cos(Math.toRadians(angleToPoint));
    velY = maxVel * Math.sin(Math.toRadians(angleToPoint));      
    
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double anglePerMeter = selfAngleRotationRemainging / remaningDistance;
    rotVel = Math.toRadians(anglePerMeter) * chassis.getMoveVelocity();
    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, rotVel);
    chassis.setVelocities(speeds);
    selfAngleRotationRemainging -= rotVel*0.02; // 0.02 is the cylce time in seconds
    remaningDistance -= chassis.getMoveVelocity()*0.02;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (remaningDistance <= 0) {
      return true;
    }

    return false;
  }
}
