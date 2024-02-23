
package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import edu.wpi.first.wpilibj.Timer;

public class DriveToNote extends Command {
  Chassis chassis;
  double velocity = 1;
  double lastDistance;

  double[] llpython;
  double distance;
  double angle;
  double lastAngle;
  NetworkTableEntry llentry;
  ChassisSpeeds speed;
  boolean finish = false;

  PIDController rotationPidController = new PIDController(0.08, 0.00, 0.006);

  Timer timer = new Timer();

  public static boolean isStart = false;

  public DriveToNote(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    isStart = true;
    lastDistance = 0;
    distance = 0;
    llentry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython");
    llpython = llentry.getDoubleArray(new double[8]);
    finish = llpython[0] == 0;
    timer.start();
  }

  @Override
  public void execute() {
    if (finish) {
      return;
    }
    llpython = llentry.getDoubleArray(new double[8]);
    distance = llpython[0];
    angle = llpython[1];
    if (distance > 0) {
      timer.reset();
    } else {
      return;
    }
    double rotateVel = (Math.abs(angle - 3) <= 3) ? 0 : rotationPidController.calculate(-angle, 3);
    double angle2 = angle + chassis.getAngle().getDegrees();
    angle2 = Math.toRadians(angle2);
    speed = new ChassisSpeeds(velocity * Math.cos(angle2), velocity * Math.sin(angle2), rotateVel);

    lastDistance = distance;
    lastAngle = angle2;
    chassis.setVelocities(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
    isStart = false;
  }

  @Override
  public boolean isFinished() {
    return finish || timer.get() > 0.5;
  }

}
