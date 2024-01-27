// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.TrapezoidCalc;

public class GoToAngle extends Command {

    Shooter shooter;
    double wantedAngle;
    double maxPow;
    double endPow;
    TrapezoidCalc calc;
    double acc;
    double startAngle;
    
    /** Creates a new GotoAngle. */
    public GoToAngle(Shooter shooter, double angle, double maxPow, double acc) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.wantedAngle = angle;
        this.maxPow = maxPow;
        this.endPow = 0;
        this.calc = new TrapezoidCalc();
        this.acc = acc;
    }

    /** Creates a new GotoAngle. */
    public GoToAngle(Shooter shooter, double angle, double maxPow, double acc, double endPow) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.wantedAngle = angle;
        this.maxPow = maxPow;
        this.endPow = endPow;
        this.calc = new TrapezoidCalc();
        this.acc = acc;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startAngle = shooter.getFalconAnlge();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pow = calc.trapezoid(shooter.getAngleVel(), maxPow, endPow, acc, (wantedAngle - shooter.getFalconAnlge()));
        System.out.println("angle pow: "+ pow);
        shooter.anlgeSetPow(pow);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.anlgeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isPassed() || (Math.abs(shooter.getFalconAnlge() - wantedAngle) <= 3);
    }

    private boolean isPassed(){
        if ((wantedAngle-startAngle > 0) && (shooter.getFalconAnlge() > wantedAngle)){
            return true;
        } else if ((wantedAngle - startAngle < 0) && (shooter.getFalconAnlge() < wantedAngle)){
            return true;
        } else {
            return false;
        }
    }
}
