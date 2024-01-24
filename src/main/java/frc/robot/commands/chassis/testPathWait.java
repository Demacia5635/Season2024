
package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.Chassis;

public class testPathWait extends SequentialCommandGroup {
  public testPathWait(Chassis chassis) {
    pathPoint[] points1 = {
      new pathPoint(0, 0, Rotation2d.fromDegrees(90), 0, false),
      new pathPoint(0, -3, Rotation2d.fromDegrees(90), 0, false),
    };
    pathPoint[] points2 = {
            new pathPoint(0, -3, Rotation2d.fromDegrees(90), 0, false),
      new pathPoint(0, -5, Rotation2d.fromDegrees(-90), 0, false)
    };
    pathPoint[] points3 = {
      new pathPoint(0, -5, Rotation2d.fromDegrees(-90), 0, false),
      new pathPoint(0, 0, Rotation2d.fromDegrees(90), 0, false),
    };
    
    addCommands(new PathFollow(chassis, points1, 4, 12).andThen(new WaitCommand(2).andThen(
    new PathFollow(chassis, points2, 4,12))).andThen(new WaitCommand(2)).andThen(
      new PathFollow(chassis, points3, 4, 12)
    ));
  }
}
