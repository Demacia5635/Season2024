package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class GoToNoteCommand extends Command {
 private final Chassis chassis;
 private double angle;
 private double startAngle;
 private double[] llpython;
 PIDController pid = new PIDController(0.31, 0.006,0.0000025);

 public GoToNoteCommand(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
 }

 @Override
 public void initialize() {
    startAngle = chassis.getAngle().getDegrees();
    chassis.stop();
 }

 @Override
 public void execute() {
    llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    angle = llpython[1];
    System.out.println(angle);
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, pid.calculate(chassis.getAngle().getDegrees() - startAngle, angle));
    chassis.setVelocities(speeds);
 }

 @Override
 public boolean isFinished() {
    return Math.abs(chassis.getAngle().getDegrees() - startAngle) - angle <= 0.5;
 }
}
