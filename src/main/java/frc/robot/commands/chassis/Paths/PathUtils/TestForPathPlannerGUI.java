
package frc.robot.commands.chassis.Paths.PathUtils;

import java.io.*;
import java.util.*;
import org.json.simple.*;
import org.json.simple.parser.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.PathFollow.Util.pathPoint;

public class TestForPathPlannerGUI {
    JSONObject jsonObject;
    public TestForPathPlannerGUI(String path){
        JSONParser parser = new JSONParser();
        try{
            jsonObject = (JSONObject) parser.parse(new FileReader(path));

        }
        catch(Exception e){
            System.out.println("WRONG PATH");
        }
    }

    public Pose2d[] getPointsNoRadius(){
        JSONArray waypoints = (JSONArray)jsonObject.get("waypoints");
        JSONArray rotations = (JSONArray)jsonObject.get("rotationTargets");
        Pose2d[] points = new Pose2d[waypoints.size()];
        for(int i = 0; i < points.length; i++){
            JSONObject index = (JSONObject) waypoints.get(i);
            JSONObject anchor = (JSONObject) index.get("anchor");
            double y = (double) anchor.get("y");
            double x = (double) anchor.get("x");
            Rotation2d angle = Rotation2d.fromDegrees((double)((JSONObject)rotations.get(i)).get("rotationDegrees"));
            points[i] = new Pose2d(x, y, angle);
        }
        return points;
    }

}
