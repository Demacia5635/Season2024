
package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.Constants.*;
public class CurrentPosOnField {
    
    public static enum Zone{
        AMP, SPEKAER, SOURCE, STAGE, OPEN;
        /**
         * get Zone based on
         * @param robotPos
         * @return
         */
        public static Zone getZone(Translation2d robotPos){
            if(rectAMP.isInside(robotPos)) return AMP;
            if(rectSPEAKER.isInside((robotPos))) return SPEKAER;
            if(rectSOURCE.isInside(robotPos)) return SOURCE;
            if(rectSTAGE.isInside(robotPos)) return STAGE;
            return OPEN;       
        }
    }


}
