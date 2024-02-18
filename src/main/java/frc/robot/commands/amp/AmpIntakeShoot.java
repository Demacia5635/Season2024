// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants.Parameters;

public class AmpIntakeShoot extends Command {
  Amp amp;
  double initialEncoderCount;
  /** Creates a new AmpIntakeShoot. */
  public AmpIntakeShoot(Amp amp) {
    this.amp = amp;
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    amp.setBrakeArm();
    amp.setBrake();
    amp.neosSetInverted(true);
    amp.resetStartPulses();
    initialEncoderCount = amp.getNeoPoseByPulses();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    amp.setNeosPower(-Parameters.INTAKE_TRANSFER_POWER); // Run motors at transfer speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.setNeosPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(amp.getPoseByPulses() >= initialEncoderCount + Parameters.SENSOR_TO_REST_DIST){
      return true;
    }
    return false;
  }
}
