
package frc.robot.commands.chassis.Auto.StartingTOP;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.PathFollow;
import frc.robot.subsystems.chassis.Chassis;

public class StratingTOPGen extends Command {
  Chassis chassis;
  List<pathPoint> points = new ArrayList<pathPoint>();
  Translation2d PointToShoot = new Translation2d(PathFollow.convertAlliance(14.070), 1.835);
  public StratingTOPGen(Chassis chassis) {
    
    this.chassis = chassis;
    addRequirements(chassis);
  }

  
  @Override
  /**
   * adds to pathPoint's list of points
   */
  public void initialize() {
    //need to add rotation based on speaker angle (track shooter to speaker)

    points.add(new pathPoint(chassis.getPose().getX(),chassis.getPose().getY(), chassis.getAngle(),
     0.1, false));
    points.add(new pathPoint(PathFollow.convertAlliance(14.112), PathFollow.fixY(1.689), Rotation2d.fromDegrees(180), 0, false));
    points.add(new pathPoint(PathFollow.convertAlliance(14.510), PathFollow.fixY(2.577), Rotation2d.fromDegrees(-1), 0.4, false)); //TODO add rotationToSpeaker
    
    points.add(new pathPoint(PathFollow.convertAlliance(13.994), PathFollow.fixY(3.187), Rotation2d.fromDegrees(-1), 0.4, false)); //TODO add rotationToSpeaker
    points.add(new pathPoint(PathFollow.convertAlliance(10.597), PathFollow.fixY(1.601), Rotation2d.fromDegrees(180), 0.5, false));
    points.add(new pathPoint(PathFollow.convertAlliance(10.597), PathFollow.fixY(1.601), Rotation2d.fromDegrees(180), 0.5, false));
    points.add(new pathPoint(PathFollow.convertAlliance(8.713), PathFollow.fixY(1.273), Rotation2d.fromDegrees(180), 0.1, false));
    points.add(new pathPoint(PointToShoot.getX(), PointToShoot.getY(), Rotation2d.fromDegrees(-1), 0.4, false)); //TODO add rotationToSpeaker
    points.add(new pathPoint(PathFollow.convertAlliance(8.692), PathFollow.fixY(2.918), Rotation2d.fromDegrees(180), 0, false));
    points.add(new pathPoint(PathFollow.convertAlliance(10.872), PathFollow.fixY(1.977), Rotation2d.fromDegrees(150), 0.4, false));
    points.add(new pathPoint(PointToShoot.getX(), PointToShoot.getY(), Rotation2d.fromDegrees(-1), 0, false)); //TODO add rotationToSpeaker
    Chassis.pointsForAuto = points;


  }

  
  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return true;
  }
}
