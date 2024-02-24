// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants;

public class Brake_Amp extends Command {
  /** Creates a new Brake_Amp. */
  private final Amp amp;
  private boolean brake = true;
  public Brake_Amp(Amp amp, boolean brake) {
    this.amp = amp;
    this.brake = brake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(brake){
      while (amp.getSnowblowerA() >= AmpConstants.Parameters.ubNormalA) {
      amp.setPowerSnowblower(1);
      }
    }
    else{
      amp.setPowerSnowblower(-1);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
