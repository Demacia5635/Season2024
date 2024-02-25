
package frc.robot.commands.chassis.Paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.intake.IntakeToShooter;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.commands.shooter.AngleGoToAngle;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.AmpPera;


public class GoToAMP extends SequentialCommandGroup {
  
  
  public GoToAMP(Shooter shooter, Chassis chassis, Intake intake, boolean isRed) {
    pathPoint[] pointsToAmp = 
      {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
      new pathPoint(15.13, 8.5, Rotation2d.fromDegrees(-90), 0, false)};
    

    addCommands(new PathFollow(chassis, pointsToAmp, 2, 6, 0, isRed).alongWith(new AngleGoToAngle(shooter, AmpPera.ANGLE))
    .alongWith(new ActivateShooter(shooter, intake, chassis, false).
    alongWith(new InstantCommand(()->shooter.isShootingAmp(true))))
    .andThen(new IntakeToShooter(intake, shooter, ShooterConstants.AmpPera.UP, ShooterConstants.AmpPera.DOWN)).withTimeout(1));
  }
}
