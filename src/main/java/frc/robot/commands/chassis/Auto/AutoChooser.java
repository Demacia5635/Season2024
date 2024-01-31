
package frc.robot.commands.chassis.Auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.PathFollow.Util.RectanglePos;
import frc.robot.PathFollow.Util.pathPoint;

public class AutoChooser {
    public enum PosToPass{
        TOP, MID, BOTTOM;
    }
    public enum Notes{
        note1, note2, note3, note4, note5, noteT, noteM, noteB;
    }
    private SendableChooser firstNote;
    private SendableChooser secondNote;
    private SendableChooser thirdNote;
    private SendableChooser fourthNote;

    private SendableChooser goTopMidOrBottom;
    private SendableChooser wantToDefend;

    
    List points = new ArrayList<pathPoint>();

    
    public AutoChooser(){
        firstNote.setDefaultOption("null", null);
        firstNote.addOption("note top", Notes.noteT);
        firstNote.addOption("note middle", Notes.noteM);
        firstNote.addOption("note bottom", Notes.noteB);
        firstNote.addOption("note top", Notes.note1);
        firstNote.addOption("note middle", Notes.note2);
        firstNote.addOption("note bottom", Notes.note3);
        firstNote.addOption("note bottom", Notes.note4);
        firstNote.addOption("note bottom", Notes.note5);


        secondNote.setDefaultOption("null", null);
        secondNote.addOption("note top", Notes.noteT);
        secondNote.addOption("note middle", Notes.noteM);
        secondNote.addOption("note bottom", Notes.noteB);
        secondNote.addOption("note top", Notes.note1);
        secondNote.addOption("note middle", Notes.note2);
        secondNote.addOption("note bottom", Notes.note3);
        secondNote.addOption("note bottom", Notes.note4);
        secondNote.addOption("note bottom", Notes.note5);

        thirdNote.setDefaultOption("null", null);
        thirdNote.addOption("note top", Notes.noteT);
        thirdNote.addOption("note middle", Notes.noteM);
        thirdNote.addOption("note bottom", Notes.noteB);
        thirdNote.addOption("note top", Notes.note1);
        thirdNote.addOption("note middle", Notes.note2);
        thirdNote.addOption("note bottom", Notes.note3);
        thirdNote.addOption("note bottom", Notes.note4);
        thirdNote.addOption("note bottom", Notes.note5);

        
        fourthNote.setDefaultOption("null", null);
        fourthNote.addOption("note top", Notes.noteT);
        fourthNote.addOption("note middle", Notes.noteM);
        fourthNote.addOption("note bottom", Notes.noteB);
        fourthNote.addOption("note top", Notes.note1);
        fourthNote.addOption("note middle", Notes.note2);
        fourthNote.addOption("note bottom", Notes.note3);
        fourthNote.addOption("note bottom", Notes.note4);
        fourthNote.addOption("note bottom", Notes.note5);

        wantToDefend.setDefaultOption("null", false);
        wantToDefend.addOption("yes",  true);
        wantToDefend.addOption("no", false);

        goTopMidOrBottom.setDefaultOption("null", false);
        goTopMidOrBottom.addOption("TOP",  PosToPass.TOP);
        goTopMidOrBottom.addOption("MID", PosToPass.MID);        
        goTopMidOrBottom.addOption("BOTTOM", PosToPass.BOTTOM);


    }
    
    public Notes getFirstNote(){
        return (Notes)firstNote.getSelected();
    }
    public Notes getSecondNote(){
        return (Notes)secondNote.getSelected();
    }
    public Notes getThirdNote(){
        return (Notes)thirdNote.getSelected();
    }
    public Notes getFourthNote(){
        return (Notes)fourthNote.getSelected();
    }
    public boolean wantToDefend(){
        if ((boolean)wantToDefend.getSelected()) return true;
        return false;
    }

    public PosToPass getTopMidBottom(){
        return (PosToPass) goTopMidOrBottom.getSelected();
    }


}
