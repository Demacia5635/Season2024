
package frc.robot.commands.chassis.Paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.shooter.Shooter;


public class GoToAMP1 extends ParallelRaceGroup {
  
    pathPoint[] toAmp = {
      new pathPoint(new Translation2d(), new Rotation2d()),
      new pathPoint(Field.AMP, Rotation2d.fromDegrees(-90))
    };
  
  public GoToAMP1() {
    Shooter shooter = RobotContainer.robotContainer.shooter;
    addCommands(shooter.getActivateShooterToAmp());
    addCommands(new PathFollow(toAmp).andThen(shooter.getShootCommand()));
  }
}
