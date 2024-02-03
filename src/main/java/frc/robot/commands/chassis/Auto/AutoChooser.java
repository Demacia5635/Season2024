
package frc.robot.commands.chassis.Auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.PathFollow.Util.RectanglePos;
import frc.robot.PathFollow.Util.pathPoint;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class AutoChooser {
    Translation2d[] zoneTop = {noteTop, noteMid, note1, note2};
    Translation2d[] zoneMid = {noteMid, noteBottom, note2, note3, note4};
    Translation2d[] zoneBottom = {noteBottom, note4, note5};
    Translation2d[] currentZone = {};

    boolean isRed = false;
    int direction = 0; //based on alliance

    Translation2d centerOfStage = new Translation2d();

    private SendableChooser firstNote;
    private SendableChooser secondNote;
    private SendableChooser thirdNote;
    private SendableChooser fourthNote;
    private SendableChooser fifthhNote;
    private SendableChooser startingZone;


    
    List points = new ArrayList<pathPoint>();

    
    public AutoChooser(boolean isRed){
        this.isRed = isRed;
        direction = isRed ? 1 : -1;

        startingZone.setDefaultOption("null", null);
        startingZone.addOption("TOP", zoneTop);
        startingZone.addOption("MIDDLE", zoneMid);
        startingZone.addOption("BOTTOM", zoneBottom);

    }

    public void setZone(){
        if(startingZone.getSelected() == null) System.out.println("Zone not selected");
        else currentZone = (Translation2d[]) startingZone.getSelected();
    }

    private pathPoint[] goThroughStage(Translation2d startingPos, Translation2d finalPos){
        List<pathPoint> points = new ArrayList<pathPoint>();
        if((finalPos.getX() - startingPos.getX()) * direction > 0)
        {
           
            if(startingPos.getY() > centerOfStage.getY()) 
                points.add(new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0.3, false)); //TODO top of stage
            else points.add(new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0.3, false)); //TODO bottom of stage
            points.add(new pathPoint(0, 0, Rotation2d.fromDegrees(180), 0.3, false)); // TODO cetner of stage
            points.add(new pathPoint(0, 0, Rotation2d.fromDegrees(180), 0.3, false)); // TODO exit of stage
        }
        else {
            points.add(new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false)); // TODO exit of stage
            points.add(new pathPoint(0, 0, Rotation2d.fromDegrees(180), 0.3, false)); // TODO cetner of stage
            if(finalPos.getY() > centerOfStage.getY()){
                points.add(new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0.3, false)); //TODO top of stage
            }
            else points.add(new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0.3, false)); // TODO bottom of stage
        }
        return (pathPoint[]) points.toArray();
    }

}
