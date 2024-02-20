package frc.robot.commands.chassis.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveAndPickNote;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.IntakeCommand;
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
  double wantedAngleClose = 57;
  double wantedVelClose = 15.5;
  double wantedAngleNoteT = -1;
  double wantedVelNoteT = -1;
  Translation2d NoteT = new Translation2d(-1, -1);
  
  pathPoint[] points0 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(1.2, 7.27, Rotation2d.fromDegrees(12), 0, false)};

  pathPoint[] points1 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(5.21, 7.7, Rotation2d.fromDegrees(0), 0, false)};
    
    
  pathPoint[] points2 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(4.5, 6.58, Rotation2d.fromDegrees(8), 0, false)};

  pathPoint[] points3 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(5.87, 6.53, Rotation2d.fromDegrees(-20), 0, false)};
  
  

  
  

  /** Creates a new StartTOP auto. */
  public StartTOP(Chassis chassis, Shooter shooter, Intake intake, boolean isRed) {


    Command ShooterAngleAndShootNoteT = new AngleGoToAngle(shooter, wantedAngleNoteT).alongWith( new ShooterPowering(shooter, wantedVelNoteT));
    Command IntakeToShooterNoteT = new IntakeToShooter(intake, shooter, wantedVelNoteT);
    

    
    addCommands((new AngleGoToAngle(shooter, wantedAngleClose).alongWith(new ShooterPowering(shooter, wantedVelClose)))
    .andThen(new IntakeToShooter(intake, shooter, wantedVelClose).raceWith(new WaitCommand(0.5)))
    .andThen(new PathFollow(chassis, points0, 3, 6, 0, isRed))
    .andThen((new DriveToNote(chassis).raceWith(new IntakeCommand(intake)))
    .andThen(new GoToAngleChassis(chassis, Rotation2d.fromDegrees(15)))
    .alongWith((new AngleGoToAngle(shooter, 38.5).alongWith(new ShooterPowering(shooter, 16.5))))
    .andThen(new IntakeToShooter(intake, shooter, 15.7).raceWith(new WaitCommand(0.5)))
    .andThen(new PathFollow(chassis, points1, 3, 6, 3, isRed)).andThen((new DriveToNote(chassis).raceWith(new IntakeCommand(intake))))
    .andThen(new PathFollow(chassis, points2, 3, 6, 0, isRed)
    .alongWith((new AngleGoToAngle(shooter, 36).alongWith(new ShooterPowering(shooter, 18)))))
    .andThen(new IntakeToShooter(intake, shooter, 18).raceWith(new WaitCommand(0.5))))
    .andThen(new PathFollow(chassis, points3, 3, 6, 2, isRed)).andThen((new DriveToNote(chassis).raceWith(new IntakeCommand(intake)))
    .andThen(new PathFollow(chassis, points2, 3, 6, 0, isRed)
    .alongWith((new AngleGoToAngle(shooter, 36).alongWith(new ShooterPowering(shooter, 18)))))
    .andThen(new IntakeToShooter(intake, shooter, 18).raceWith(new WaitCommand(0.5)))));


     
  }

}

