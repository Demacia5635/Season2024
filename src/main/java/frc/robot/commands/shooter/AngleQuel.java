// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/**command that will quel the angle motor */
public class AngleQuel extends Command {

    /**the wanted shooter */
    Shooter shooter;

    /**
     * creates a new command that quel the shooter angle
     * @param shooter the wanted shooter
     */
    public AngleQuel(Shooter shooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    /**set the anlge motor pow */
    @Override
    public void execute() {
        shooter.angleSetPow(0.3);
    }

    // Called once the command ends or is interrupted.
    /**stops the anlge motor and reset the dis */
    @Override
    public void end(boolean interrupted) {
        shooter.angleStop();
        shooter.resetDis();
    }

    // Returns true when the command should end.
    /**check if the motor is at the limit */
    @Override
    public boolean isFinished() {
        return shooter.isLimit();
    }
}
