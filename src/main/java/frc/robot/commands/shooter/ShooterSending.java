// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

/**command that will send the note from the feeding motor to the shooting motor */
public class ShooterSending extends Command {

    /**the wanted shooter */
    Shooter shooter;
    /**the pow giving to the feeding motor */
    double feedingPow;
    /**the pow giving to the shooting motor */
    double shootingPow;
    /**var that will count to check the note has really been shot */
    int count;

    /**
     * creates a new command that will send from the feeding motor to the shooting motor the note
     * @param shooter the wanted shooter
     * @param feedingPow the wanted pow for the feeding motor -1 to 1
     * @param shootingPow the wanted pow for the shooting motor -1 to 1
     */
    public ShooterSending(Shooter shooter, double feedingPow, double shootingPow) {
        this.shooter = shooter;
        this.feedingPow = feedingPow;
        this.shootingPow = shootingPow;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    /**
     * reset the coount
     * put the motor at brake 
     */
    @Override
    public void initialize() {
        count = 0;
        shooter.brake(1);
        shooter.brake(2);
        shooter.brake(3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    /**setting pow to every motor */
    @Override
    public void execute() {
        shooter.setPow(shootingPow);
        shooter.feedingSetPow(feedingPow);

        /*reset the count if there is a note */
        if (shooter.isNote()){
            count = 0;
        } else {
            count++;
        }
    }

    // Called once the command ends or is interrupted.
    /**stoping all the motors if the note have been shot */
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.feedingStop();
    }

    // Returns true when the command should end.
    /**check if the count is bigger than 5 */
    @Override
    public boolean isFinished() {
        return count >= 5;
    }
}
