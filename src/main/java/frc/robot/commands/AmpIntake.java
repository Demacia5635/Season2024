// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Amp;

public class AmpIntake extends Command {
  Amp amp;
  boolean last;
  double v1;
  double v2;
  /** Creates a new AmpIntake. */
  public AmpIntake(Amp amp, double v1, double v2) {
    this.amp = amp;
    this.v1 = v1;
    this.v2 = v2;
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.neoEncoderReset();
    last = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(amp.isClose()){
      if(!amp.isNoteThere(last)){
        amp.neosSetVel(v1, v2);
      }else {
        amp.neosSetVel(0, 0);
      }
    }
    if(amp.isOpen()){
      if(amp.isNoteThere(last)){
        amp.neosSetVel(-v1, -v2);
      }else {
        amp.neosSetVel(0, 0);
      }
    }    
    last = amp.didNotePass();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.neosSetVel(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
