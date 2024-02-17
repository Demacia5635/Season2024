
package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utils.TrapezoidNoam;

public class DriveToNote extends Command {
  Intake intake;
  Chassis chassis;
  double velocity = 1;
  
  double[] llpython;
  double distance;
  double angle;
  NetworkTableEntry llentry;
  PIDController rotationPidController = new PIDController(0.4, 0.00, 0.000025);

  public DriveToNote(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }
  @Override
  public void initialize() {
    llentry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython");    
  }

  

  
  @Override
  public void execute() {
    llpython = llentry.getDoubleArray(new double[8]);
    distance = llpython[0] + 50;
    angle = llpython[1];
    System.out.println("Angle Note: " + angle);

    SmartDashboard.putNumber("ANGLE NOTE", angle);
    SmartDashboard.putNumber("DISTANCE NOTE", distance);

    
    double rotateVel = (Math.abs(angle) <= 4) ? 0 : rotationPidController.calculate(-angle,4);
    SmartDashboard.putBoolean("isalligned", Math.abs(angle) <= 5); 
    SmartDashboard.putNumber("rotvel", -Math.toRadians(rotateVel));

    /*double Note_X = llpython[2];
    double Note_Y = llpython[3];*/
    double angle2 = angle + chassis.getAngle().getDegrees();
    angle2 = Math.toRadians(angle2);
    double v = distance == 0? 0 : velocity;
    ChassisSpeeds speed;
    if(rotateVel == 0) {
      speed = new ChassisSpeeds(-v*Math.cos(angle2), -v*Math.sin(angle2), 0);  
    }
    else{
      speed = new ChassisSpeeds(0,0, -Math.toRadians(rotateVel));
    }

    
    chassis.setVelocities(speed);
    
  }
@Override
  public void initSendable(SendableBuilder builder) {
       

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop(); 
  }


  
}
