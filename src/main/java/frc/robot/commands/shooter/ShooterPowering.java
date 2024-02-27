// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_MOTOR;

/**command that will give vel to the shooter shooting motors */
public class ShooterPowering extends Command {
    
    /**the wanted shooter */
    Shooter shooter;

    /**the vel giving to the shooter up motor */
    double velUp;
    
    /**the vel giving to the shooter down motor */
    double velDown;

    /**the vel the shooter needs to be to shoot */
    double maxVel;

    /**
     * creates a new command that give vel to the shooter shooting motors
     * @param shooter the wanted shooter
     * @param shootingVelUp the vel giving the up motor in meter per sec
     * @param shootingVelDown the vel giving thed down motor in meter per sec 
     */
    public ShooterPowering(Shooter shooter, double shootingVelUp, double shootingVelDown) {
        this.shooter = shooter;
        this.velUp = shootingVelUp;
        this.velDown = shootingVelDown;
        addRequirements(shooter);
    }

    /**
     * creates a ne command that give el to  the shooter shooting motors
     * @param shooter the wanted shooter
     * @param shootingVel the vel giving to both of the shooter motors in meter per sec
     */
    public ShooterPowering(Shooter shooter, double shootingVel){
        this.shooter = shooter;
        this.velUp = shootingVel;
        this.velDown = shootingVel;
        addRequirements(shooter);
    }

    /**put the shooting motor at brake */
    @Override
    public void initialize() {
        shooter.brake(SHOOTER_MOTOR.UP, SHOOTER_MOTOR.DOWN);
    }

    /**giving the motor power */
    @Override
    public void execute() {
        shooter.setVel(velDown, velUp);
    }

    /**not stopping the motor bcz we want it still fast when the shooter shoot */
    @Override
    public void end(boolean interrupted) {
    }

    /**checks if the shooter have come to the wanted vel */
    @Override
    public boolean isFinished() {
        return Math.abs(velUp - shooter.getMotorVel(SHOOTER_MOTOR.UP) ) <= 0.3;
    }
}
