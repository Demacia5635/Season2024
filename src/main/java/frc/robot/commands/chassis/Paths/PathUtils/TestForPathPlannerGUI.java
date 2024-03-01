
package frc.robot.commands.chassis.Paths.PathUtils;

import java.io.*;
import java.util.*;
import org.json.simple.*;
import org.json.simple.parser.*;

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

    public Translation2d[] getXandY(){
        JSONArray waypoints = (JSONArray)jsonObject.get("waypoints");
        Translation2d[] points = new Translation2d[waypoints.size()];
        for(int i = 0; i < points.length; i++){
            JSONObject index = (JSONObject) waypoints.get(i);
            JSONObject anchor = (JSONObject) index.get("anchor");
            double y = (double) anchor.get("y");
            double x = (double) anchor.get("x");
            points[i] = new Translation2d(x, y);
        }
        return points;
    }

}
