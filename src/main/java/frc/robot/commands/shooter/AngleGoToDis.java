// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_MOTOR;

/**command that makes the angle motor go to a specific dis */
public class AngleGoToDis extends Command {
    
    /**the shooter we want to use */
    Shooter shooter;
    /**the wanted angle */
    double wantedDis;
    /**the start dis of the angle motor being used in isFinish */
    double startDis;

    /**
     * creates a new command that goes to a specifig distance
     * @param shooter the shooter we want to control angle motor of
     * @param dis the wanted dis in mm
     */
    public AngleGoToDis(Shooter shooter, double dis) {
        this.shooter = shooter;
        this.wantedDis = dis;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    /**
     * set the angle motor in brake mode
     * set the start dis
     */
    @Override
    public void initialize() {
        shooter.brake(SHOOTER_MOTOR.ANGLE);
        startDis = shooter.getDis();
    }

    // Called every time the scheduler runs while the command is scheduled.
    /**using the motion magic control mode to go to the specific dis */
    @Override
    public void execute() {
        if ((wantedDis - shooter.getDis()) > 0){
            shooter.setPow(0.4);
        } else {
            shooter.setPow(0.4);
        }
    }

    // Called once the command ends or is interrupted.
    /**stop the angle motor when the command have finished */
    @Override
    public void end(boolean interrupted) {
        shooter.angleStop();
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
        return ((shooter.isSupplyLimit(SHOOTER_MOTOR.ANGLE)) ||
                (shooter.isDisLimits(wantedDis - startDis > 0)) ||
                ((wantedDis - startDis > 0) && (shooter.getDis() >= wantedDis)) ||
                ((wantedDis - startDis < 0) && (shooter.getDis() <= wantedDis)) ||
                (Math.abs(wantedDis - shooter.getDis())) < 1);
        // if (!shooter.isSupplyLimit(SHOOTER_MOTOR.ANGLE)){
        //     if (!shooter.isDisLimits(wantedDis - startDis > 0)){
        //         if (!((wantedDis - startDis > 0) && (shooter.getDis() >= wantedDis))){
        //             if (!((wantedDis - startDis < 0) && (shooter.getDis() <= wantedDis))){
        //                 if (!(Math.abs(wantedDis - shooter.getDis()) < 1)){
        //                     return false;
        //                 }
        //             }
        //         }
        //     }
        // }

        // return true;
    }
}
