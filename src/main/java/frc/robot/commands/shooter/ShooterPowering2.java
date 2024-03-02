// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_MOTOR;

/**command that will power up the shooter */
public class ShooterPowering2 extends Command {
    
    /**the wanted shooter */
    Shooter shooter;
    /**the power giving the shooter */
    double velUp;
    double velDown;
    /**the vel the shooter needs to be to shoot */
    double maxVel;

    /**
     * creates a new command that power up the shooter
     * @param shooter the wanted shooter
     * @param shootingVelUp the power giving the shooter -1 to 1
     * @param shootingMaxVel the velocity we want the shooter to be in pules per 1/10 sec
     */
    public ShooterPowering2(Shooter shooter, double shootingVelUp, double shootingVelDown) {
        this.shooter = shooter;
        this.velUp = shootingVelUp;
        this.velDown = shootingVelDown;
        addRequirements(shooter);
    }

    public ShooterPowering2(Shooter shooter, double shootingVel){
        this.shooter = shooter;
        this.velUp = shootingVel;
        this.velDown = shootingVel;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    /**put the shooting motor at brake */
    @Override
    public void initialize() {
        shooter.brake(SHOOTER_MOTOR.UP, SHOOTER_MOTOR.DOWN);
        // velUp = SmartDashboard.getNumber("wanted vel", 0);
        // velDown = SmartDashboard.getNumber("wanted vel", 0);

    }

    // Called every time the scheduler runs while the command is scheduled.
    /**giving the motor power */
    @Override
    public void execute() {
        shooter.setVel(velDown, velUp);
        // shooter.setPow(pow);
    }

    // Called once the command ends or is interrupted.
    /**not stopping the motor bcz we want it still fast when the shooter shoot */
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    /**checks if the shooter have come to the wanted vel */
    @Override
    public boolean isFinished() {
   return Math.abs(velUp - shooter.getMotorVel(SHOOTER_MOTOR.UP) ) <= 0.3;
      //  return vel == shooter.getMotorVel();
        // return shooter.getMotorVel(SHOOTER_MOTOR.UP) >= maxVel && shooter.getMotorVel(SHOOTER_MOTOR.DOWN) >= maxVel;
    }
}
