
package frc.robot.commands.chassis.Auto.StartingTOP;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.PathFollow;
import frc.robot.subsystems.chassis.Chassis;

public class StartingTOP extends SequentialCommandGroup {
  Chassis chassis;
  public StartingTOP(Chassis chassis, double maxVel, double maxAccel) {
    this.chassis = chassis;
    pathPoint[] points = (pathPoint[]) Chassis.pointsForAuto.toArray();

    addCommands(new StratingTOPGen(chassis), new PathFollow(chassis, points, maxVel, maxAccel, false));
  }
}
