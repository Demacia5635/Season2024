
package frc.robot.commands.chassis.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.Paths.PathUtils.GetPointsToAMP;
import frc.robot.subsystems.chassis.Chassis;

public class GoToAMP extends SequentialCommandGroup {
  Chassis chassis;

  public GoToAMP(Chassis chassis, double maxVel, double maxAccel) {
    this.chassis = chassis;
    pathPoint[] points = (pathPoint[]) Chassis.pointsForPathTeleop.toArray();
    addCommands(new GetPointsToAMP(chassis), new PathFollow(chassis,points, maxVel, maxVel, 0, false));
  }
}
