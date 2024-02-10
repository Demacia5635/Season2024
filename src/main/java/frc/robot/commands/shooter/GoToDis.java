// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class GoToDis extends Command {
    
    Shooter shooter;
    double wantedDis;
    double maxVel;
    double acc;
    double startDis;

    /**
     * creates a new command that goes to a specifig distance
     * @param shooter the shooter we want to control angle motor of
     * @param dis the wanted dis in mm
     * @param maxVel the max velocity of the trapezoid in pules per 1/10 sec
     * @param acc the acc of the trapezoid in pules per 1/10 sec
     */
    public GoToDis(Shooter shooter, double dis, double maxVel, double acc) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.wantedDis = dis;
        this.maxVel = maxVel;
        this.acc = acc;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    /**
     * set the angle motor in brake mode
     * set the start dis
     */
    @Override
    public void initialize() {
        shooter.angleBrake();
        startDis = shooter.getDis();
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
        if (!shooter.limits(wantedDis - startDis > 0)){
            if (!((wantedDis - startDis > 0) && (shooter.getDis() >= wantedDis))){
                if (!((wantedDis - startDis < 0) && (shooter.getDis() <= wantedDis))){
                    if (!(Math.abs(wantedDis - shooter.getDis()) < 1)){
                        return false;
                    }
                }
            }
        }

        return true;
    }
}