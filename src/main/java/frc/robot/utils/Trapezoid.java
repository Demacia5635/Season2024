// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/** Add your docs here. */
public class Trapezoid {
    double maxVelocity; // Maximum permissible velocity
    double maxAcceleration; // Maximum permissible acceleration
    private double deltaVelocity; // Velocity increment at each time step
    private double lastTime  = 0;
    private double lastV;
    private double lastA;


    // Constructor to initialize with maximum velocity and acceleration
    public Trapezoid(double maxVelocity, double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        // Velocity increment calculated based on max acceleration
        deltaVelocity = maxAcceleration * 0.02;
    }

    // Helper function to calculate distance required to change from current velocity to target velocity
    private double distanceToVelocity(double currentVelocity, double targetVelocity, double acceleration) {
        double deltaVelocity = currentVelocity - targetVelocity;
        double time = Math.abs(deltaVelocity/acceleration);
        return (currentVelocity + targetVelocity)/2*time; // avg velocity * time
    }

    // Function to calculate the next velocity setpoint, based on remaining distance and current and target velocities
    public double calculate(double remainingDistance, double curentVelocity, double targetVelocity) {
        // Case for negative remaining distance
        if(remainingDistance < 0) {
            return  -calculate(-remainingDistance, -curentVelocity, -targetVelocity);
        }
        double time = Timer.getFPGATimestamp();
        double cv = curentVelocity;
        // update cv for situation that we are not accelerating or deaccelerating as expected
        if(time - lastTime < Constants.CYCLE_DT*2) {
            if(lastA > 0 && curentVelocity < lastV) { // 
                cv = lastV;
            } else if(lastA < 0 && curentVelocity > lastV) {
                cv = lastV;
            }
        }        // Case for below max velocity, and enough distance to reach targetVelocity at max acceleration
        if(cv < maxVelocity && distanceToVelocity(cv+deltaVelocity, targetVelocity, maxAcceleration) < remainingDistance - cycleDistanceWithAccel(cv)) {
            lastV = Math.min(curentVelocity + deltaVelocity, maxVelocity);
        } 
        // Case for enough distance to reach targetVelocity without acceleration
        else if(distanceToVelocity(cv, targetVelocity, maxAcceleration) < remainingDistance - cycleDistanceNoAccel(cv)) {
            lastV = cv;
        } 
        // Case for not enough distance to reach targetVelocity, must decelerate
        else {
            // calclate the required acceleration from cv to target v in remaining distance
            double t = 2 * remainingDistance / (curentVelocity + targetVelocity);
            lastV = cv - (curentVelocity-targetVelocity)*Constants.CYCLE_DT/t;
        }
        lastA = lastV - curentVelocity;
        lastTime = time;
        return lastV;
    }

    // Helper function to compute the distance travelled in one cycle without acceleration
    private double cycleDistanceNoAccel(double currentVelocity) {
        return currentVelocity * Constants.CYCLE_DT;
    }
    // Helper function to compute the distance travelled in one cycle with maximum acceleration
    private double cycleDistanceWithAccel(double currentVelocity) {
        return currentVelocity * Constants.CYCLE_DT + (0.5*maxAcceleration * Math.pow(Constants.CYCLE_DT, 2));
    }
}