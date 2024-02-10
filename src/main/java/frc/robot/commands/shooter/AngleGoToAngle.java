// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import static frc.robot.Constants.ShooterConstants.*;

/** commands that goes to a specific angle */
public class AngleGoToAngle extends Command {
    
    /**the shooter that being used */
    Shooter shooter;
    /**the wanted angle */
    double wantedAngle;
    /**
     * the wanted dis based on the wanted angle
     * being caculate by f(x) in Desmos - {@link https://www.desmos.com/calculator/4ja9zotx82} */
    double wantedDis;
    /**the max velocity wanted to the trapezoid */
    double maxVel;
    /**the acc wanted for the trapezoid */
    double acc;
    /**the start dis of the angle motor being used in isFinish */
    double startDis;
    
    /**
     * creates a new command that goes to a specific angle
     * @param shooter the shooter we want to control angle motor of
     * @param angle the wanted angle in degrees
     * @param maxVel the max velocity of the trapezoid in pules per 1/10 sec
     * @param acc the acc of the trapezoid  in pules per 1/10 sec
     */
    public AngleGoToAngle(Shooter shooter, double angle, double maxVel, double acc) {
        this.shooter = shooter;
        this.wantedAngle = angle;
        this.maxVel = maxVel;
        this.acc = acc;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    /**
     * set the angle motor in brake mode
     * set the start dis
     * caculate the wanted dis based on f(x) in desmos {@link https://www.desmos.com/calculator/4ja9zotx82} 
     */
    @Override
    public void initialize() {
        shooter.brake(4);
        startDis = shooter.getDis();

        wantedDis = KA * Math.cos(wantedAngle * Math.PI / 180) + Math.sqrt(Math.pow(KA, 2) * Math.pow(Math.cos(wantedAngle * Math.PI / 180), 2) - Math.pow(KA, 2) + Math.pow(KB, 2));
    }

    // Called every time the scheduler runs while the command is scheduled.
    /**using the motion magic control mode to go to the specific dis */
    @Override
    public void execute() {
        shooter.angleMotionMagic(wantedDis, maxVel, acc);
    }

    // Called once the command ends or is interrupted.
    /**stop the angle motor when the command have finished */
    @Override
    public void end(boolean interrupted) {
        shooter.anlgeStop();
    }

    // Returns true when the command should end.
    /**
     * every headline is for every if condition
     * @limits use the limits function from shooter to check if the anlge motor have not run off
     * @passWantedPositive if the direction is positive and the dis is passed the wanted dis
     * @passWantedNegative if the direction is negative and the dis is passed the wanted dis
     * @abs if the abs distance between the current dis and the wanted dis is exactly or less than 1 mm
     * 
     * @return if the function have finished
     */
    @Override
    public boolean isFinished() {
        if (!shooter.isSupplyLimit(4)){
            if (!shooter.isDisLimits(wantedDis - startDis > 0)){
                if (!((wantedDis - startDis > 0) && (shooter.getDis() >= wantedDis))){
                    if (!((wantedDis - startDis < 0) && (shooter.getDis() <= wantedDis))){
                        if (!(Math.abs(wantedDis - shooter.getDis()) < 1)){
                            return false;
                        }
                    }
                }
            }
        }

        return true;
    }
}
