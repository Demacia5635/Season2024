
package frc.robot.commands.chassis;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.Util.CurrentPosOnField.Zone;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.Chassis;

public class GetPoinsPathTeleop extends Command {
  Chassis chassis;
  pathPoint lastPoint;
  List<pathPoint> points = new ArrayList<pathPoint>();
  /**
   * 
   * @param chassis
   * @param lastPoint
   */
  public GetPoinsPathTeleop(Chassis chassis, pathPoint lastPoint) {
    this.chassis = chassis;
    this.lastPoint = lastPoint;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    points.add(new pathPoint(chassis.getPose().getX(), chassis.getPose().getY(),
     chassis.getAngle(), 1, false));
     
    
    Zone zone = Zone.getZone(chassis.getPose().getTranslation());
    System.out.println("ZONE: " + zone);
    switch (zone) {
      case STAGE:

        break;
      case SOURCE:

        break;
      case SPEKAER:

        break;
      default:
      
        break;
    }
    Chassis.pointsForPathTeleop = points;
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
