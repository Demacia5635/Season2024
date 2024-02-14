// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.SHOOTER_MOTOR;

/**command that get take the note from the intake and giving it to the shooter */
public class IntakeToShooter extends Command {

    /**the wanted intake */
    Intake intake;
    /**the wanted shooter */
    Shooter shooter;
    // boolean hasEntered;

    /**
     * creates a new command thats takes from the intake and giving it to the shooter
     * @param intake the wanted intake
     * @param shooter the wanted shooter
     */
    public IntakeToShooter(Intake intake, Shooter shooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake, shooter);
        SmartDashboard.putData(this);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.brake(SHOOTER_MOTOR.UP, SHOOTER_MOTOR.DOWN, SHOOTER_MOTOR.FEEDING);
        intake.setBrake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    /**set pow to the feeding motors and the intake */
    @Override
    public void execute() {
        // if (!hasEntered) {
        //     intake.setPower(IntakeConstants.Parameters.INTAKE_PRE_LIMIT_POWER);
        // }
        // else {
        //     intake.setPower(1);
        // }
        // if(intake.isCriticalCurrentWhenShooter()) hasEntered = true;

        intake.setPower(1);
        shooter.feedingSetPow(1);
        // shooter.setPow(1);
    }

    // Called once the command ends or is interrupted.
    /**stopes all motors */
    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.feedingStop();
        // shooter.stop();
    }

    // Returns true when the command should end.
    /**checks if there is note on the shooter */
    @Override
    public boolean isFinished() {
        return shooter.isNote();
    }
}
