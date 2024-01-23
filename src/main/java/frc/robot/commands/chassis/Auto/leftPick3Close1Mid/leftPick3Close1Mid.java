
package frc.robot.commands.chassis.Auto.leftPick3Close1Mid;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.PathFollow;
import frc.robot.subsystems.chassis.Chassis;

public class leftPick3Close1Mid extends SequentialCommandGroup {
  Chassis chassis;
  public leftPick3Close1Mid(Chassis chassis, double maxVel, double maxAccel) {
    this.chassis = chassis;
    pathPoint[] points = (pathPoint[]) Chassis.pointsForAuto.toArray();
    addCommands(new leftPick3Close1MidGEN(chassis, false), new PathFollow(chassis, points, maxVel, maxAccel));
  }
}