
package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;

public class GoToNote extends Command {
  Intake intake;
  Chassis chassis;
  double velocity;
  PIDController rotationPidController = new PIDController(0.31, 0.006, 0.0000025);

  
  public GoToNote(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  

  
  @Override
  public void execute() {
    double[] llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    double distance = llpython[0];
    double angle = llpython[1];
    /*double Note_X = llpython[2];
    double Note_Y = llpython[3];*/
    
    velocity = (distance >= 1) ? 3 : 1;
    
    Translation2d velVector = new Translation2d(velocity, chassis.getAngle().plus(Rotation2d.fromDegrees(angle)));

    ChassisSpeeds speed = new ChassisSpeeds(velVector.getX() , velVector.getY(), rotationPidController.calculate(chassis.getAngle().getDegrees(),
     chassis.getAngle().getDegrees() + angle));
    chassis.setVelocities(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();  }

  
}
