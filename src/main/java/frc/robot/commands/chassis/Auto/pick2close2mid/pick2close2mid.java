package frc.robot.commands.chassis.Auto.pick2close2mid;

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
import frc.robot.subsystems.shooter.utils.shootFromAnyPlace;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class pick2close2mid extends SequentialCommandGroup {
  shootFromAnyPlace shootFromAnyPlace = new shootFromAnyPlace();
  pathPoint[] points1 = {PathFollow};
  pathPoint[] points2 = {};
  pathPoint[] points3 = {};
  pathPoint[] points4 = {}; //place to shoot from
  pathPoint[] points5 = {};
  pathPoint[] points6 = {};

  
  

  /** Creates a new pick2close2mid. */
  public pick2close2mid(Chassis chassis, Shooter shooter, Intake intake, boolean isRed) {
    
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
  }
}
