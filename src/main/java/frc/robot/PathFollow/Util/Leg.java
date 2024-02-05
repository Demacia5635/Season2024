// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.PathFollow.Util;

import static frc.robot.subsystems.chassis.ChassisConstants.robotLength;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */



public class Leg extends Segment{
    Translation2d totalVector;
    final Translation2d velDirection;
    /**
     * creates a leg type segment
     * @param p1 the first point of the leg
     * @param p2 the last point of the leg
     * @param isAprilTagMode search for closet april tag mode
     */
    public Leg(Translation2d p1, Translation2d p2, boolean isAprilTagMode)
    {
        super(p1,p2, isAprilTagMode);
        totalVector = p2.minus(p1);
        velDirection = totalVector.div(totalVector.getNorm());
    }

    @Override
    public Translation2d calc(Translation2d position, double velocity)
    {
        if(isAprilTagMode()) velocity = Math.min(velocity, 1);
        Translation2d relativePos = position.minus(p2);

        Rotation2d diffAngle = p1.minus(p2).getAngle().minus(relativePos.getAngle());

        return new Translation2d(velocity, relativePos.times(-1).getAngle().minus(diffAngle));
    }

    public double calcDistanceToCorner(double x, double y, double a, double c){
        return Math.abs((a * x) + y + c) / (Math.sqrt((Math.pow(a, 2) + 1 /*b to the power of 2 which is 1 */)));
    }

    public pathPoint calcFixPoint(RectanglePos rectangle, Rotation2d angle){

        double b = (-calcSlope()) * (p1.getX() + p1.getY());
        //offset for each direction (x and y)
        double offset = (robotLength + 15) / Math.sqrt(2);
        
        if(calcSlope() < 0){
            //case for top right is closer
            if(calcDistanceToCorner(rectangle.topRight.getX(), rectangle.topRight.getY(), calcSlope(), b) <
            calcDistanceToCorner(rectangle.bottomLeft.getX(), rectangle.bottomLeft.getY(), calcSlope(), b)){
                return new pathPoint(rectangle.topRight.getX() + offset, rectangle.topRight.getY() + offset, angle , 0.5, false);
            }
            else{
                return new pathPoint(rectangle.bottomLeft.getX() + offset, rectangle.bottomLeft.getY() + offset, angle , 0.5, false);
            }
        }
        else if(calcSlope() > 0){
            if(calcDistanceToCorner(rectangle.topLeft.getX(), rectangle.topLeft.getY(), calcSlope(), b) <
            calcDistanceToCorner(rectangle.bottomRight.getX(), rectangle.bottomRight.getY(), calcSlope(), b)){
                return new pathPoint(rectangle.topLeft.getX() + offset, rectangle.topLeft.getY() + offset, angle, 0.5, false);
            }
            else{
                return new pathPoint(rectangle.bottomRight.getX() + offset, rectangle.bottomRight.getY() + offset, angle , 0.5, false);
            }
        }
        else{
            if(p1.getY() > (rectangle.topRight.getY() - rectangle.bottomRight.getY()) / 2){
                return new pathPoint((rectangle.bottomLeft.getX() + rectangle.topRight.getX()) / 2,
                 rectangle.topRight.getY() + robotLength + 20 /*Safety measurment */, angle, 0.5, false);
            }
            else{
                return new pathPoint((rectangle.bottomLeft.getX() + rectangle.topRight.getX()) / 2,
                 rectangle.bottomLeft.getY() - robotLength - 20 /*Safety measurment */, angle, 0.5, false);
            }
        }

    }
    

    public double calcSlope(){
        return (p2.getY() - p1.getY()) / (p2.getX() - p1.getX());
    }

    public double calcXIntersection(double y, double b){
        return (y - b) / calcSlope(); 
    }

    public double calcYIntersection(double x, double b){
        return (calcSlope() * x) + b;
    }

    public boolean isLegal(RectanglePos rectangle){
        double slope = calcSlope();
        // y = mx+*b*
        double b = (-slope) * (p1.getX() + p1.getY());
        double x = calcXIntersection(rectangle.bottomLeft.getY(), b);
        if(rectangle.isInside(new Translation2d(x, rectangle.bottomLeft.getY()))){
            return false;
        }
        double y = calcYIntersection(rectangle.bottomLeft.getX(), b);
        if(rectangle.isInside(new Translation2d(rectangle.bottomLeft.getX(), y))){
            return false;
        }
        x = calcXIntersection(rectangle.topRight.getY(), b);
        if(rectangle.isInside(new Translation2d(x, rectangle.topRight.getY()))){
            return false;
        }
        y = calcYIntersection(rectangle.topRight.getX(), b);
        if(rectangle.isInside(new Translation2d(rectangle.topRight.getX(), y))){
            return false;
        }
        return true;

    }   

    @Override
    public double distancePassed(Translation2d position)
    {
        Translation2d relativePos = position.minus(p1);

        //double distanceMoved = (relativePos.getX() * velDirection.getX()) + (relativePos.getY()*velDirection.getY());
        return Math.abs(relativePos.getNorm());
    }

    @Override
    public double getLength()
    {
        return totalVector.getNorm();
    }

   

    @Override
    public String toString() {
        return "\n~Leg~\np1 : " + p1 + "\np2 : " + p2 + "\nTotalVector : " + totalVector;
    }
}
