// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;
import frc.robot.utils.TrapezoidCalc;

public class GoToAngleAmp extends Command {
  Amp amp;
  double angleRad;
  double maxVelRad;
  double acceleRad;
  TrapezoidCalc trap;
  boolean check = false;

  /** Creates a new GoToAngleAmp. */
  public GoToAngleAmp( Amp amp, double angleRad, double maxVelRad, double acceleRad) {
    this.amp = amp;
    this.angleRad = angleRad;
    this.maxVelRad = maxVelRad;
    this.acceleRad = acceleRad;
    this.trap = new TrapezoidCalc();
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.setBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double velRad = 0;
      double currentAngleRad = amp.getArmAngle();
      if( angleRad > currentAngleRad) {
        velRad = trap.trapezoid(amp.getVelRadArm(), maxVelRad, 0, 
        Math.abs(acceleRad), angleRad-currentAngleRad);
        //System.out.println(" cur ang=" + currentAngleRad + " tgt=" + angleRad + " v=" + velRad);
        amp.setVel(velRad);
      } else {
          amp.setPowerArm(-0.2);
        }
    }
    
    
    
  
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //amp.setVel(0);
    amp.setPowerArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleRad - amp.getArmAngle()) < Math.toRadians(3);
  }
}