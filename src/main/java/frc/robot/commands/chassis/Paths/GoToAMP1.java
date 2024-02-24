
package frc.robot.commands.chassis.Paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;


public class GoToAMP1 extends ParallelCommandGroup {
  
  
  public GoToAMP1(Shooter shooter, Chassis chassis, Intake intake, boolean isRed) {

    pathPoint[] pointsToAmp = {
      new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
      new pathPoint(14.321, 0.1, Rotation2d.fromDegrees(-90), 0.25, false)
    };
    addCommands(shooter.getActivateShooterToAmp());
    addCommands(new PathFollow(chassis, pointsToAmp, 4, 12, 0, isRed).andThen(shooter.getShootCommand()));
  }
}
