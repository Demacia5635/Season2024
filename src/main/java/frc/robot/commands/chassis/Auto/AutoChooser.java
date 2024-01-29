
package frc.robot.commands.chassis.Auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.PathFollow.Util.pathPoint;

public class AutoChooser {
    Translation2d avoid = new Translation2d(0, 0);
    
    public enum Notes{
        note1, note2, note3, note4, note5, noteT, noteM, noteB;
    }
    private SendableChooser firstNote;
        private SendableChooser secondNote;
    private SendableChooser wantToDefend;

    
    List points = new ArrayList<pathPoint>();

    
    public AutoChooser(){
        firstNote.setDefaultOption("null", null);
        firstNote.addOption("note top", Notes.noteT);
        firstNote.addOption("note middle", Notes.noteM);
        firstNote.addOption("note bottom", Notes.noteB);

        wantToDefend.setDefaultOption("null", false);
        wantToDefend.addOption("yes",  true);
        wantToDefend.addOption("no", false);

        secondNote.setDefaultOption("null", null);
        secondNote.addOption(null, avoid);


    }
    
    public Notes getFirstNote(){
        return (Notes)firstNote.getSelected();
    }
    public Notes getSecondNote(){
        return (Notes)secondNote.getSelected();
    }
    public boolean wantToDefend(){
        if ((boolean)wantToDefend.getSelected()) return true;
        return false;
    }


}
