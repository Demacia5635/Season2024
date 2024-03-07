
package frc.robot.commands.chassis.AutoPrevious;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class MidPlayoffTEST extends SequentialCommandGroup {
  Translation2d noteM = Field.WingNotes[1];
  pathPoint[] pointsToFirstNote = {
    new pathPoint(new Translation2d(), Rotation2d.fromDegrees(0), 0, false), new pathPoint(noteM.getX(), noteM.getY(), Rotation2d.fromDegrees(0), 0, false)
  };
  boolean isRed;
  Chassis chassis;
  Intake inake;
  Shooter shooter;
  public MidPlayoffTEST() {
    this.chassis = RobotContainer.robotContainer.chassis;
    this.inake = RobotContainer.robotContainer.intake;
    this.shooter = RobotContainer.robotContainer.shooter;
    this.isRed = RobotContainer.robotContainer.isRed();
    Command cmd = new SequentialCommandGroup(initShooter())
    .andThen(shooter.getShootCommand())
    .andThen((new PathFollow(pointsToFirstNote))
    .alongWith(new ActivateShooter(shooter, inake, chassis, true)))
    .andThen(shooter.getShootCommand());

    cmd.schedule();
  }


  private Command initShooter() {
    return new WaitUntilCommand(shooter::getIsShootingReady);
}
}
