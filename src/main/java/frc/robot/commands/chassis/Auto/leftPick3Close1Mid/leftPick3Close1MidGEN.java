
package frc.robot.commands.chassis.Auto.leftPick3Close1Mid;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.subsystems.chassis.Chassis;

public class leftPick3Close1MidGEN extends Command {
  Chassis chassis;
  List<pathPoint> points = new ArrayList<pathPoint>();
  public leftPick3Close1MidGEN(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  
  @Override
  /**
   * adds to pathPoint's list of points
   */
  public void initialize() {
    //need to add rotation based on speaker angle (track shooter to speaker)
    /**
     * goes to each note (the closer to the start point)
     * 
     */
    points.add(new pathPoint(chassis.getPose().getX(),chassis.getPose().getY(), chassis.getAngle(),
     0.1, false));
    points.add(new pathPoint(13.516, 1.261, null, 0.1, false));
    points.add(new pathPoint(14.018, 4.587, null, 0.5, false));
    points.add(new pathPoint(13.516, 4.946, new Rotation2d(0), 0.1, false));
    points.add(new pathPoint(14.206, 5.961, null, 0.5, false));
    points.add(new pathPoint(13.516, 6.281, null, 0.1, false));

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
