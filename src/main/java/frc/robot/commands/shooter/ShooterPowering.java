// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/**command that will power up the shooter */
public class ShooterPowering extends Command {
    
    /**the wanted shooter */
    Shooter shooter;
    /**the power giving the shooter */
    double pow;
    /**the vel the shooter needs to be to shoot */
    double maxVel;

    /**
     * creates a new command that power up the shooter
     * @param shooter the wanted shooter
     * @param shootingPow the power giving the shooter -1 to 1
     * @param shootingMaxVel the velocity we want the shooter to be in pules per 1/10 sec
     */
    public ShooterPowering(Shooter shooter, double shootingPow, double shootingMaxVel) {
        this.shooter = shooter;
        this.pow = shootingPow;
        this.maxVel = shootingMaxVel;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    /**giving the motor power */
    @Override
    public void execute() {
        shooter.setPow(pow);
    }

    // Called once the command ends or is interrupted.
    /**not stopping the motor bcz we want it still fast when the shooter shoot */
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    /**checks if the shooter have come to the wanted vel */
    @Override
    public boolean isFinished() {
        return shooter.getMotorVel(1) >= maxVel && shooter.getMotorVel(2) >= maxVel;
    }
}
