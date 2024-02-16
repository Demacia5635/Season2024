
package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  double velocity;
  
  double[] llpython;
  double distance;
  double angle;
  double fieldRelativeAngle;
  double distanceLeft;
  double noteX;
  double noteY;
  Translation2d notePos;
  PIDController rotationPidController = new PIDController(0.31, 0.006, 0.0000025);
  public DriveToNote(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }
  @Override
  public void initialize() {
    llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    fieldRelativeAngle = llpython[1] + chassis.getAngle().getDegrees();
    distance = llpython[0];
    
    notePos = new Translation2d(distance, llpython[1]);
    
  }

  

  
  @Override
  public void execute() {
    llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    distance = llpython[0];
    angle = llpython[1];
    
    double rotateVel = (Math.abs(angle) <= 2) ? 0 : rotationPidController.calculate(chassis.getAngle().getDegrees(), fieldRelativeAngle) * 180; //rotateTrap.calculate(distanceLeft, Math.toDegrees(chassis.getChassisSpeeds().omegaRadiansPerSecond), 0);
    /*double Note_X = llpython[2];
    double Note_Y = llpython[3];*/
    
    velocity = 0;
    if(distance == 0) notePos = new Translation2d();
  

    ChassisSpeeds speed = new ChassisSpeeds(notePos.getX() * -0.5, notePos.getY() * -0.5, -Math.toRadians(rotateVel));
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
