package frc.robot.commands.chassis.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveAndPickNote;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.IntakeToShooter;
import frc.robot.commands.shooter.AngleGoToAngle;
import frc.robot.commands.shooter.ShooterPowering;
import frc.robot.commands.shooter.ShooterShoot;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.CommandParams;
import frc.robot.subsystems.shooter.utils.shootFromAnyPlace;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class StartTOP extends SequentialCommandGroup {
  //shootFromAnyPlace shootFromAnyPlace = new shootFromAnyPlace();
  double wantedAngleClose = 56;
  double wantedVelClose = 14;
  double wantedAngleNoteT = -1;
  double wantedVelNoteT = -1;
  Translation2d NoteT = new Translation2d(-1, -1);

  pathPoint[] points1 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(PathFollow.convertAlliance(14.661), PathFollow.fixY(1.370 ), Rotation2d.fromDegrees(180), 0, false)};
    
 
  

  
  

  /** Creates a new StartTOP auto. */
  public StartTOP(Chassis chassis, Shooter shooter, Intake intake, boolean isRed) {
    Command ShooterAngleAndShootClose = new AngleGoToAngle(shooter, wantedAngleClose).alongWith(new ShooterPowering(shooter, wantedVelClose));
    Command ShooterAngleAndShootNoteT = new AngleGoToAngle(shooter, wantedAngleNoteT).alongWith( new ShooterPowering(shooter, wantedVelNoteT));
    Command IntakeToShooterClose = new IntakeToShooter(intake, shooter, wantedVelClose);
    Command IntakeToShooterNoteT = new IntakeToShooter(intake, shooter, wantedVelNoteT);
    

    
    addCommands(IntakeToShooterClose.andThen(ShooterAngleAndShootClose).andThen((new PathFollow(chassis, points1, 3, 6, 3, isRed))
    .alongWith(ShooterAngleAndShootNoteT)).andThen(new DriveAndPickNote(chassis, intake)));
    
  }
}

