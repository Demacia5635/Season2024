// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class TurnAngle extends CommandBase {
    final Shooter shooter;
    double turns;
    double vel;
    double startPos;

    /** Creates a new TurnAngle. */
    public TurnAngle(Shooter shooter, double turns, double vel) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.turns = turns;
        this.vel = vel;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.neonEncoderReset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.neonSetPow(vel);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.neonSetPow(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooter.getNEONRev() >= turns+startPos;
    }
}
