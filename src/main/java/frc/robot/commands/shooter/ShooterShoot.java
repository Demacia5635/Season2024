// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_MOTOR;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/**sequential command group that will make the shooter shoot */
public class ShooterShoot extends SequentialCommandGroup {

    /**
     * creates a new command that will make the shooter shoot
     * @param shooter the wanted shooter
     * @param feedingPow the wanted pow for the feeding motor -1 to 1
     * @param shootingPow the wanted pow for the shooting motor -1 to 1
     * @param shootingMaxVel the wanted vel for the shooting motor in pules per 1/10 sec
     * @param angle the wanted angle to shoot at in degrees
     * @param angleMaxVel the max vel for the angle trapezoid
     * @param angleAcc the acc for the angle trapezoid
     */
    public ShooterShoot(Shooter shooter, double feedingPow, double shootingPow, double shootingMaxVel, double angle, double angleMaxVel, double angleAcc) {
        addRequirements(shooter);
        addCommands(
            new InstantCommand(()-> { shooter.brake(SHOOTER_MOTOR.UP, SHOOTER_MOTOR.DOWN, SHOOTER_MOTOR.FEEDING, SHOOTER_MOTOR.ANGLE); }, shooter),
            // new ShooterFeeding(shooter, feedingPow),
            // new AngleGoToAngle(shooter, angle),
            new ShooterPowering(shooter, shootingPow, shootingMaxVel),
            new ShooterSending(shooter, feedingPow, shootingPow)
        );
    }
}
