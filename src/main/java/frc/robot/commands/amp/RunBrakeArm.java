// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants.Parameters;

public class RunBrakeArm extends Command {
  Amp amp;
  double p;
  double startTime;
  /** Creates a new SnowBlowerRun. */
  public RunBrakeArm(Amp amp, double p) {
    this.amp = amp;
    this.p = p;
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    amp.setPowerSnowblower(p);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((amp.getSnowblowerA()>Parameters.ARM_BRAKE_MAX_A)||((p<0)&&(Timer.getFPGATimestamp()-startTime >0.7)));
  }
}
