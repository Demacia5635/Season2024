
package frc.robot.commands.chassis.Auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.pathPoint;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class AutoChooser {
    public enum Zone{
        TOP, MID, BOTTOM, NULL
    }
    private HashMap<Zone, Translation2d[]> notesInZone = new HashMap<Zone, Translation2d[]>();
    boolean isRed;
    Translation2d centerOfStage = new Translation2d();

    private SendableChooser<Translation2d> notes;
    private SendableChooser<Zone> startingZone;

    
    List<pathPoint> points = new ArrayList<pathPoint>();

    
    public AutoChooser(){
        notesInZone.put(Zone.TOP, new Translation2d[]{noteTop, noteMid, note1, note2});
        notesInZone.put(Zone.MID, new Translation2d[]{noteMid, noteBottom, note2, note3, note4});
        notesInZone.put(Zone.BOTTOM, new Translation2d[]{noteBottom, note4, note5}); 
        this.isRed = RobotContainer.robotContainer.isRed();

        startingZone.setDefaultOption("null", null);
        startingZone.addOption("TOP", Zone.TOP);
        startingZone.addOption("MIDDLE", Zone.MID);
        startingZone.addOption("BOTTOM", Zone.BOTTOM);
    }
    private Zone getZone(){
        return (Zone)startingZone.getSelected() != null ? (Zone)startingZone.getSelected() : Zone.NULL;
    }
    public void showLegalPoints(){
        Zone zone = getZone();
        int i = 1;
        for (Translation2d note : notesInZone.get(zone)) {
            notes.addOption("note " + i, note);
            System.out.println();
            i++;
        }
    }


    

    

}
