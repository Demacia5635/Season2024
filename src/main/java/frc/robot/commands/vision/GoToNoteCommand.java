package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class GoToNoteCommand extends Command {
 private final Chassis chassis;
 private double Dist;
 private double Angle;
 private double Note_X;
 private double Note_Y;
 private double[] llpython;

 // Constructor initializes the chassis and adds it as a requirement
 public GoToNoteCommand(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
 }

 @Override
 public void initialize() {
    // Stop the chassis at the start of the command
    chassis.stop();
 }

 @Override
 public void execute() {
    // Get the distance and angle from the GetDistAndAngle method
    llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    Dist = llpython[0];
    Angle = llpython[1];
    Note_X = llpython[2];
    Note_Y = llpython[3];
 }
 @Override
 public boolean isFinished() {
   return false;
 }
 @Override
 public void end(boolean interrupted) {
   chassis.stop();
 }

}
