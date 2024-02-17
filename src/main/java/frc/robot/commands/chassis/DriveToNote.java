
package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
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
  double velocity = 2;
  
  double[] llpython;
  double distance;
  double angle;
  NetworkTableEntry llentry;
  PIDController rotationPidController = new PIDController(0.4, 0.00, 0.0000);

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
    distance = llpython[0];
    angle = llpython[1];
    System.out.println("Angle: " + angle);
    
    double rotateVel = (Math.abs(angle) <= 2) ? 0 : Math.PI / 2 * Math.signum(angle); //rotationPidController.calculate(-angle,0); 
    /*double Note_X = llpython[2];
    double Note_Y = llpython[3];*/
    angle += chassis.getAngle().getDegrees();
    angle = Math.toRadians(angle);
    double v = 0; //distance == 0? 0 : velocity;
    ChassisSpeeds speed = new ChassisSpeeds(-v*Math.cos(angle), -v*Math.sin(angle), -rotateVel);
    chassis.setVelocities(speed);
    
  }
@Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("anglsgkhdhu", ()-> angle, null);    
    builder.addDoubleProperty("dist", ()-> distance, null);    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop(); 
  }


  
}
