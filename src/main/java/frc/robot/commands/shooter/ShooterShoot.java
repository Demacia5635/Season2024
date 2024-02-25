// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/** @deprecated
 * sequential command group that will make the shooter shoot */
public class ShooterShoot extends SequentialCommandGroup {

    /**
     * creates a new command that will make the shooter shoot
     * @param shooter the wanted shooter
     * @param feedingPow the wanted pow for the feeding motor -1 to 1
     * @param shootingVel the wanted pow for the shooting motor -1 to 1
     * @param shootingMaxVel the wanted vel for the shooting motor in pules per 1/10 sec
     * @param angle the wanted angle to shoot at in degrees
     * @param angleMaxVel the max vel for the angle trapezoid
     * @param angleAcc the acc for the angle trapezoid
     */
    public ShooterShoot(Shooter shooter, double feedingPow, double shootingVel, double angle, double angleMaxVel, double angleAcc) {
        addRequirements(shooter);
        addCommands(
            // new ShooterFeeding(shooter, feedingPow),
            // new AngleGoToAngle(shooter, angle),
            new ShooterPowering(shooter, shootingVel),
            new ShooterSending(shooter, feedingPow, shootingVel)
        );
    }
}
