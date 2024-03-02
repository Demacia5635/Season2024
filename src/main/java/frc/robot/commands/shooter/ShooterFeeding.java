// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_MOTOR;

/** @deprecated
 * command that moving the feeding motor until the note is in the shotooter */
public class ShooterFeeding extends Command {

    /**the wanted shooter */
    Shooter shooter;
    /**the wanted pow of the feeding motor */
    double pow;
    boolean temp;
    int count;

    /**
     * creates a new command that feeding the shooter a note
     * @param shooter the wanted shooter
     */
    public ShooterFeeding(Shooter shooter, double feedingPow) {
        this.shooter = shooter;
        this.pow = feedingPow;
        addRequirements(shooter);
    }

    /**put the feeding motor at brake */
    @Override
    public void initialize() {
        temp = false;
        shooter.brake(SHOOTER_MOTOR.FEEDING);
    }

    /**set the feeding motor power */
    @Override
    public void execute() {
        shooter.feedingSetPow(pow);
        if (shooter.isNote()){
            count++;
        } else {
            count=0;
        }

        if (count>=10){
            temp = true;
        }
    }

    /**stop the feeding motor when the note is in the shooter */
    @Override
    public void end(boolean interrupted) {
        shooter.feedingStop();
    }

    /**checks if the note is in the shooter */
    @Override
    public boolean isFinished() {
        return temp && !shooter.isNote();
    }
}
