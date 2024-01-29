// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.KB;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import static frc.robot.Constants.ShooterConstants.*;

public class GoToAngle extends Command {
    
    Shooter shooter;
    double wantedAngle;
    double pow;
    double wantedDis;

    /** Creates a new GoToAngle. */
    public GoToAngle(Shooter shooter, double angle, double pow) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.wantedAngle = angle;
        this.pow = pow;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        wantedDis =  KB * Math.cos(wantedAngle * Math.PI / 180)+Math.sqrt(Math.pow(KA, 2) - Math.pow((KB * Math.sin(wantedAngle * Math.PI / 180)), 2));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (wantedDis > shooter.getDis()){
            shooter.anlgeSetPow(pow);    
        } else{
            shooter.anlgeSetPow(-1*pow);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.anlgeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(shooter.getDis()-wantedDis) < 3;
    }
}
