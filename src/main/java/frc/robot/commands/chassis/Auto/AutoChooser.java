
package frc.robot.commands.chassis.Auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.PathFollow.Util.Arc;
import frc.robot.PathFollow.Util.RectanglePos;
import frc.robot.PathFollow.Util.Segment;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.PathFollow;

import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class AutoChooser {
    Translation2d PointToShootTOP = new Translation2d(PathFollow.convertAlliance(14.070), 1.835);
    Translation2d PointToShootMID = new Translation2d(PathFollow.convertAlliance(14.264), 3.160);
    Translation2d PointToShootBOTTOM = new Translation2d(PathFollow.convertAlliance(14.760), 5.463);


    Translation2d[] zoneTop = {noteTop, noteMid, note1, note2};
    Translation2d[] zoneMid = {noteMid, noteBottom, note2, note3, note4};
    Translation2d[] zoneBottom = {noteBottom, note4, note5};
    Translation2d[] currentZone = {};

    boolean isRed = false;
    int direction = 0; //based on alliance

    Translation2d centerOfStage = new Translation2d(PathFollow.convertAlliance(11.734), 4.247);

    private SendableChooser<Translation2d> firstNote;
    private SendableChooser<Translation2d> secondNote;
    private SendableChooser<Translation2d> thirdNote;
    private SendableChooser<Translation2d> fourthNote;
    private SendableChooser<Translation2d> fifthhNote;
    private SendableChooser<Translation2d[]> startingZone;


    
    List<pathPoint> points = new ArrayList<pathPoint>();

    
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
                points.add(new pathPoint(PathFollow.convertAlliance(12.283), 3.206, Rotation2d.fromDegrees(0), 0.3, false));
            else points.add(new pathPoint(PathFollow.convertAlliance(12.195), 5.023, Rotation2d.fromDegrees(0), 0.3, false)); 
            points.add(new pathPoint(centerOfStage.getX(), centerOfStage.getY(), Rotation2d.fromDegrees(180), 0.3, false)); 
            points.add(new pathPoint(PathFollow.convertAlliance(10.337), 4.231, Rotation2d.fromDegrees(180), 0.3, false)); 
        }
        else {
            points.add(new pathPoint(PathFollow.convertAlliance(10.337), 4.231, Rotation2d.fromDegrees(180), 0.3, false));
            points.add(new pathPoint(centerOfStage.getX(), centerOfStage.getY(), Rotation2d.fromDegrees(180), 0.3, false)); 
            if(finalPos.getY() > centerOfStage.getY()){
                points.add(new pathPoint(PathFollow.convertAlliance(12.195), 5.023, Rotation2d.fromDegrees(0), 0.3, false)); 
            }
            else points.add(new pathPoint(PathFollow.convertAlliance(12.283), 3.206, Rotation2d.fromDegrees(0), 0.3, false));
        }
        return (pathPoint[]) points.toArray();
    }

    /*public pathPoint[] createPoints(){
        List<pathPoint> points = new ArrayList<pathPoint>();
        for(int i = 0; i < currentZone.length; i++){
            //first point is automatically set to chassis position
            points.add(new pathPoint(0 , 0, null, 0, isRed));

        }
    }*/



    public void showLegalPoints(Translation2d[] zone){
        if(zone == zoneTop){
            firstNote.setDefaultOption("none", new Translation2d());
            secondNote.setDefaultOption("none", new Translation2d());
            thirdNote.setDefaultOption("none", new Translation2d());
            fourthNote.setDefaultOption("none", new Translation2d());
            firstNote.addOption("Note Top", noteTop);
            firstNote.addOption("Note Mid", noteMid);
            firstNote.addOption("Note 1", note1);
            firstNote.addOption("Note 2", note2);
            secondNote.addOption("Note Top", noteTop);
            secondNote.addOption("Note Mid", noteMid);
            secondNote.addOption("Note 1", note1);
            secondNote.addOption("Note 2", note2);
            thirdNote.addOption("Note Top", noteTop);
            thirdNote.addOption("Note Mid", noteMid);
            thirdNote.addOption("Note 1", note1);
            thirdNote.addOption("Note 2", note2);
            fourthNote.addOption("Note Top", noteTop);
            fourthNote.addOption("Note Mid", noteMid);
            fourthNote.addOption("Note 1", note1);
            fourthNote.addOption("Note 2", note2);
            
            
            
        }
        else if(zone == zoneMid){
            firstNote.setDefaultOption("none", null);
            firstNote.addOption("Note Mid", noteMid);
            firstNote.addOption("Note Bottom", noteBottom);
            firstNote.addOption("Note 2", note2);
            firstNote.addOption("Note 3", note3);
            firstNote.addOption("Note 4", note4);

            secondNote.setDefaultOption("none", null);
            secondNote.addOption("Note Mid", noteMid);
            secondNote.addOption("Note Bottom", noteBottom);
            secondNote.addOption("Note 2", note2);
            secondNote.addOption("Note 3", note3);
            secondNote.addOption("Note 4", note4);

            thirdNote.setDefaultOption("none", null);
            thirdNote.addOption("Note Mid", noteMid);
            thirdNote.addOption("Note Bottom", noteBottom);
            thirdNote.addOption("Note 2", note2);
            thirdNote.addOption("Note 3", note3);
            thirdNote.addOption("Note 4", note4);

            fourthNote.setDefaultOption("none", null);
            fourthNote.addOption("Note Mid", noteMid);
            fourthNote.addOption("Note Bottom", noteBottom);
            fourthNote.addOption("Note 2", note2);
            fourthNote.addOption("Note 3", note3);
            fourthNote.addOption("Note 4", note4);

            fifthhNote.setDefaultOption("none", null);
            fifthhNote.addOption("Note Mid", noteMid);
            fifthhNote.addOption("Note Bottom", noteBottom);
            fifthhNote.addOption("Note 2", note2);
            fifthhNote.addOption("Note 3", note3);
            fifthhNote.addOption("Note 4", note4);
        }
        else{
            firstNote.setDefaultOption("none", null);
            firstNote.addOption("Note Bottom", noteBottom);
            firstNote.addOption("Note 4", note4);
            firstNote.addOption("Note 5", note5);

            secondNote.setDefaultOption("none", null);
            secondNote.addOption("Note Bottom", noteBottom);
            secondNote.addOption("Note 4", note4);
            secondNote.addOption("Note 5", note5);

            thirdNote.setDefaultOption("none", null);
            thirdNote.addOption("Note Bottom", noteBottom);
            thirdNote.addOption("Note 4", note4);
            thirdNote.addOption("Note 5", note5);
        }
        
    }


}
