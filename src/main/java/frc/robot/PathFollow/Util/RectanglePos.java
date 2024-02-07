
package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Translation2d;

public class RectanglePos {
    public Translation2d topLeft;
    public Translation2d bottomRight;
    public Translation2d topRight;
    public Translation2d bottomLeft;
    public RectanglePos(Translation2d topRight, Translation2d bottomLeft){
        if(bottomLeft.getY() >= topRight.getY() || bottomLeft.getX() >= topRight.getX()){

            //WRONG INPUT IN RECTANGLE Bottom left: Translation2d(X: 11.32, Y: 3.13) top right: Translation2d(X: 6.01, Y: 5.65)
            
            System.out.println("WRONG INPUT IN RECTANGLE Bottom left: " + bottomLeft +" top right: " + topRight);
        }
            
            
        
        else{
            this.bottomLeft = bottomLeft;
            this.topRight = topRight;
            topLeft = new Translation2d(bottomLeft.getX(), topRight.getY());
            bottomRight = new Translation2d(topRight.getX(), bottomLeft.getY());
        }
        
    }

    public boolean isInside(Translation2d pos){
        return pos.getX() >= bottomLeft.getX() && pos.getX() <= topRight.getX() &&
        pos.getY() >= bottomLeft.getY() && pos.getY() <= topRight.getY();
    }
}
