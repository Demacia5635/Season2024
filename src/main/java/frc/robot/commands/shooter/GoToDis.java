// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Trapezoid;

public class GoToDis extends Command {
    
    Shooter shooter;
    double wantedDis;
    Trapezoid calc;
    double maxVel;
    double acc;
    double endVel;
    double startDis;
    
    /** Creates a new GoToAngle. */
    public GoToDis(Shooter shooter, double dis, double maxVel, double acc) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.wantedDis = dis;
        this.maxVel = maxVel;
        this.acc = acc;
        this.endVel = 0;
        addRequirements(shooter);
    }

    /** Creates a new GoToAngle. */
    public GoToDis(Shooter shooter, double dis, double maxVel, double acc, double endVel) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.wantedDis = dis;
        this.maxVel = maxVel;
        this.acc = acc;
        this.endVel = endVel;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    /**
     * Desmos - {@link https://www.desmos.com/calculator/4ja9zotx82}
     */
    @Override
    public void initialize() {
        shooter.angleBrake();
        startDis = shooter.getDis();
        calc = new Trapezoid(maxVel, acc);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!imFinished()){
            double vel = calc.calculate(wantedDis - shooter.getDis(), shooter.getAngleVel(), 0 );
            // System.out.println("vel = "+ vel);
            shooter.angleSetVel(vel);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.anlgeStop();
        if (!interrupted){
            if (Math.abs(wantedDis - shooter.getDis()) < 1){
                new GoToDis(shooter, wantedDis, maxVel, acc);
            }
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        if (!shooter.limits(wantedDis - startDis > 0)){
            if (!(Math.abs(wantedDis - shooter.getDis()) < 0.3)){
                return false;
            }
        }
    
        return true;
    }

    private boolean imFinished(){

        if (!shooter.limits(wantedDis - startDis > 0)){
            if (!((wantedDis - startDis > 0) && (shooter.getDis() >= wantedDis))){
                if (!((wantedDis - startDis < 0) && (shooter.getDis() <= wantedDis))){
                    if (!(Math.abs(wantedDis - shooter.getDis()) < 1)){
                        return false;
                    }
                }
            }
        }

        return true;
    }
}
