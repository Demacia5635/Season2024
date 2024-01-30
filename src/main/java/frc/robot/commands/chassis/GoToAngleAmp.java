// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Amp;
import frc.robot.utils.TrapezoidCalc;

public class GoToAngleAmp extends Command {
  Amp amp;
  double angleRad;
  double maxVelRad;
  double acceleRad;
  TrapezoidCalc trap;

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
    amp.neoEncoderReset();
    amp.startRad(amp.getPoseRad());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velRad = trap.trapezoid(amp.getVelRadArm(), maxVelRad, 0, Math.abs(acceleRad), angleRad-amp.getPoseRad());
    if((amp.getPoseRad()>1.7)&&(amp.getPoseRad()<Math.PI*0.7)){
      velRad += Math.PI*0.25;
    }
    
    amp.velFFArm(amp.getPoseRad(), velRad, acceleRad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((amp.getPoseRad()>=angleRad-10/360*2*Math.PI)&&(amp.getPoseRad()<=angleRad+10/360*2*Math.PI)){
      return true;
    }
    return false;
  }
}
