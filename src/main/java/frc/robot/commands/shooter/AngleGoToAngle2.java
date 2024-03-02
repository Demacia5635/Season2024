// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_MOTOR;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

/** commands that goes to a specific angle */
public class AngleGoToAngle2 extends Command {
    
    /**the shooter that being used */
    Shooter shooter;
    /**the wanted angle */
    double wantedAngle;
    /**
     * the wanted dis based on the wanted angle
     * being caculate by f(x) in Desmos - {@link https://www.desmos.com/calculator/4ja9zotx82} */
    double wantedDis;
    /**the start dis of the angle motor being used in isFinish */
    double startDis;
    
    /**
     * creates a new command that goes to a specific angle
     * @param shooter the shooter we want to control angle motor of
     * @param angle the wanted angle in degrees
     */
    public AngleGoToAngle2(Shooter shooter, double angle) {
        this.shooter = shooter;
        this.wantedAngle = angle;
    }

    // Called when the command is initially scheduled.
    /**
     * set the angle motor in brake mode
     * set the start dis
     * caculate the wanted dis based on f(x) in desmos {@link https://www.desmos.com/calculator/4ja9zotx82} 
     */
    @Override
    public void initialize() {
        shooter.brake(SHOOTER_MOTOR.ANGLE);
        startDis = shooter.getDis();
        //wantedAngle = SmartDashboard.getNumber("wanted angle", 20);

        wantedDis = AngleChanger.KA * Math.cos(wantedAngle * Math.PI / 180) + Math.sqrt(Math.pow(AngleChanger.KA, 2) * Math.pow(Math.cos(wantedAngle * Math.PI / 180), 2) - Math.pow(AngleChanger.KA, 2) + Math.pow(AngleChanger.KB, 2));
    }

    // Called every time the scheduler runs while the command is scheduled.
    /**using constant pow to go to the specific dis */
    @Override
    public void execute() {
        if ((wantedDis - shooter.getDis()) > 0){
            shooter.angleSetPow(0.4);
        } else {
            shooter.angleSetPow(-0.4);
        }

        //SmartDashboard.putNumber("power sign", (wantedAngle - shooter.getDis()));
        // shooter.angleMotionMagic(wantedDis);
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

        // SmartDashboard.putBoolean("1", !shooter.isSupplyLimit(SHOOTER_MOTOR.ANGLE));
        // SmartDashboard.putBoolean("2", !shooter.isDisLimits(wantedDis - startDis > 0));

        // SmartDashboard.putBoolean("3", !((wantedDis - startDis > 0) && (shooter.getDis() >= wantedDis)));
        // SmartDashboard.putBoolean("4", !((wantedDis - startDis < 0) && (shooter.getDis() <= wantedDis)));
        // SmartDashboard.putBoolean("5", !(Math.abs(wantedDis - shooter.getDis()) < 1));



        if (!shooter.isSupplyLimit(SHOOTER_MOTOR.ANGLE)){
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
