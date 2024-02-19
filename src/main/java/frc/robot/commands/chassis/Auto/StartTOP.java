package frc.robot.commands.chassis.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveAndPickNote;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.shooter.ShooterShoot;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.CommandParams;
import frc.robot.subsystems.shooter.utils.shootFromAnyPlace;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class StartTOP extends SequentialCommandGroup {
  shootFromAnyPlace shootFromAnyPlace = new shootFromAnyPlace();
  

  
  

  /** Creates a new pick2close2mid. */
  public StartTOP(Chassis chassis, Shooter shooter, Intake intake, boolean isRed) {
    /*pathPoint[] points1 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(PathFollow.convertAlliance(14.661), PathFollow.fixY(1.370 ), Rotation2d.fromDegrees(180), 0, false)};
    
    pathPoint[] points2 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(PathFollow.convertAlliance(14.020), PathFollow.fixY(1.913), Rotation2d.fromDegrees(90), 0, false)};

    pathPoint[] points3 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(PathFollow.convertAlliance(11.601), PathFollow.fixY(1.183), Rotation2d.fromDegrees(180), 0.4, false),
    new pathPoint(PathFollow.convertAlliance(10.010), PathFollow.fixY(0.994), Rotation2d.fromDegrees(180), 0.4, false)};

    pathPoint[] points4 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(PathFollow.convertAlliance(11.601), PathFollow.fixY(1.183), Rotation2d.fromDegrees(225), 0, false)};

    pathPoint[] points5 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(PathFollow.convertAlliance(10.022), PathFollow.fixY(2.608), Rotation2d.fromDegrees(225), 0, false)};

    pathPoint[] points6 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(PathFollow.convertAlliance(12.228), PathFollow.fixY(1.673), Rotation2d.fromDegrees(225), 0, false)};

    

    Translation2d Speaker = (isRed) ? RED_ALLIANCE_SPEAKER.getTranslation() : BLUE_ALLIANCE_SPEAKER.getTranslation();
    addCommands(new ShooterShoot(shooter, ShooterConstants.CommandParams.FEED_POWER, shootFromAnyPlace.getPow(Speaker.minus(chassis.getPose().getTranslation()).getNorm()),
      CommandParams.MAX_VEL, shootFromAnyPlace.getAngle(Speaker.minus(chassis.getPose().getTranslation()).getNorm()), CommandParams.MAX_VEL, CommandParams.MAX_ACCCEL),

      new PathFollow(chassis, points1, 3, 6, isRed), new DriveAndPickNote(chassis, intake),
      new ShooterShoot(shooter, ShooterConstants.CommandParams.FEED_POWER, shootFromAnyPlace.getPow(Speaker.minus(chassis.getPose().getTranslation()).getNorm()),
      CommandParams.MAX_VEL, shootFromAnyPlace.getAngle(Speaker.minus(chassis.getPose().getTranslation()).getNorm()), CommandParams.MAX_VEL, CommandParams.MAX_ACCCEL), 

      new PathFollow(chassis, points2, 3, 6, isRed), new DriveAndPickNote(chassis, intake),
      new ShooterShoot(shooter, ShooterConstants.CommandParams.FEED_POWER, shootFromAnyPlace.getPow(Speaker.minus(chassis.getPose().getTranslation()).getNorm()),
      CommandParams.MAX_VEL, shootFromAnyPlace.getAngle(Speaker.minus(chassis.getPose().getTranslation()).getNorm()), CommandParams.MAX_VEL, CommandParams.MAX_ACCCEL),

      new PathFollow(chassis, points3, 3, 6, isRed), new DriveAndPickNote(chassis, intake), new PathFollow(chassis, points4, 3, 6, isRed), 
      new ShooterShoot(shooter, ShooterConstants.CommandParams.FEED_POWER, shootFromAnyPlace.getPow(Speaker.minus(chassis.getPose().getTranslation()).getNorm()),
      CommandParams.MAX_VEL, shootFromAnyPlace.getAngle(Speaker.minus(chassis.getPose().getTranslation()).getNorm()), CommandParams.MAX_VEL, CommandParams.MAX_ACCCEL),

      new PathFollow(chassis, points5, 3, 6, isRed), new DriveAndPickNote(chassis, intake), new PathFollow(chassis, points6, 3, 6, isRed), 
      new ShooterShoot(shooter, ShooterConstants.CommandParams.FEED_POWER, shootFromAnyPlace.getPow(Speaker.minus(chassis.getPose().getTranslation()).getNorm()),
      CommandParams.MAX_VEL, shootFromAnyPlace.getAngle(Speaker.minus(chassis.getPose().getTranslation()).getNorm()), CommandParams.MAX_VEL, CommandParams.MAX_ACCCEL)


    );
    */
  }
}
