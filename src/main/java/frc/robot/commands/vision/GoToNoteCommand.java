package frc.robot.commands.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;


public class GoToNoteCommand extends Command {
  private final Chassis chassis;
  private double dist;
  private double angle;

  public GoToNoteCommand(Chassis chassis, double dist, double angle) {
    this.chassis = chassis;
    this.dist = dist;
    this.angle = angle;

    addRequirements(chassis);

  }

  @Override
  public void initialize() {
    chassis.stop();
  }

  @Override
  public void execute() {

    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, Math.toRadians(angle));
    chassis.setVelocities(speeds);
  }
}
