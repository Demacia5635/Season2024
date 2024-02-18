// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/**command that moving the feeding motor until the note is in the shotooter */
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

    // Called when the command is initially scheduled.
    /**put the feeding motor at brake */
    @Override
    public void initialize() {
        temp = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
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

    // Called once the command ends or is interrupted.
    /**stop the feeding motor when the note is in the shooter */
    @Override
    public void end(boolean interrupted) {
        shooter.feedingStop();
    }

    // Returns true when the command should end.
    /**checks if the note is in the shooter */
    @Override
    public boolean isFinished() {
        return temp && !shooter.isNote();
    }
}
