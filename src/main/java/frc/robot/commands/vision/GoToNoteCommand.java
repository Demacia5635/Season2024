package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class GoToNoteCommand extends Command {
 private final Chassis chassis;
 private double Angle;
 private double Dist;
 private double Dist_OfSet;
 private double No_Dist_OfSet;
 private double Mol;
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
    Angle = llpython[1];
    Dist = llpython[0];
    Mol = Dist * Math.tan(Angle);
    Mol = Math.abs(No_Dist_OfSet - Mol);
    Dist = Dist + Dist_OfSet;
    Angle = Math.atan(Mol/Dist);
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0.5*Math.signum(Angle));
    chassis.setVelocities(speeds);
 }

 @Override
 public boolean isFinished() {
   return Math.abs(Angle) <= 1;

   
 }
 @Override
  public void end(boolean interrupted) {
   chassis.stop();
   System.out.println("doneeeeeeeeeeeeeee\n\n\n\n\n\n\n\n\n\n\n");
  }

}
