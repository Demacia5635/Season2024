package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class GoToNoteCommand extends Command {
 private final Chassis chassis;
 private double angle;
 private double dist;
 private double[] llpython;

 public GoToNoteCommand(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
 }

 @Override
 public void initialize() {
    chassis.stop();
 }

 @Override
 public void execute() {
    llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    angle = llpython[1];
    dist = llpython[0];
    System.out.println("angle " + angle);
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0.5*Math.signum(angle));
    chassis.setVelocities(speeds);
 }

 @Override
 public boolean isFinished() {
   return Math.abs(angle) <= 1;

   
 }
 @Override
  public void end(boolean interrupted) {
   chassis.stop();
   System.out.println("doneeeeeeeeeeeeeee\n\n\n\n\n\n\n\n\n\n\n");
  }

}
