
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
  boolean pickTop;
  public leftPick3Close1MidGEN(Chassis chassis, boolean pickTop) {
    this.chassis = chassis;
    this.pickTop = pickTop;
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
    points.add(new pathPoint(13.838, 1.508, null, 0.1, false));
    points.add(new pathPoint(14.572, 2.27, null, 0.5, false));
    points.add(new pathPoint(13.895, 2.963, null, 0.1, false));
    points.add(new pathPoint(14.623, 3.785, null, 0.5, false));
    points.add(new pathPoint(13.907, 4.4, null, 0.1, false));
    if(pickTop){
      points.add(new pathPoint(12.974, 6.736, null, 0.1, false));
      points.add(new pathPoint(8.775, 7.866, null, 0.4, false));
    }
        
    else{
      points.add(new pathPoint(13.790, 5.776, null, 0.6, false));
      points.add(new pathPoint(11.014, 4.272, null, 0.6, false));
      points.add(new pathPoint(8.159, 6.181, null, 0.6, false));
    }

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
