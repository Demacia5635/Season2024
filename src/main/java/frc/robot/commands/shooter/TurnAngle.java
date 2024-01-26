// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class TurnAngle extends CommandBase {
    final Shooter shooter;
    double turns;
    double pow;
    double startPos;

    /** Creates a new TurnAngle. */
    public TurnAngle(Shooter shooter, double turns, double pow) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.turns = turns;
        this.pow = pow;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.neonEncoderReset();
        // shooter.angleResetPos();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.neonSetPow(pow);
        // shooter.anlgeSetPow(pow);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.neonSetPow(0);
        // shooter.anlgeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return shooter.getAnlgeEncoder() >= turns+startPos;
        return shooter.neon.getEncoder().getPosition() >= turns+startPos;
    }
}
