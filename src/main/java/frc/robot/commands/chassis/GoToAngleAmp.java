// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Amp;
import frc.robot.utils.Trapezoid;

public class GoToAngleAmp extends Command {
  CommandXboxController xboxController;
  Amp amp;
  double angleRad;
  double velRad;
  double acceleRad;
  Trapezoid trap;

  /** Creates a new GoToAngleAmp. */
  public GoToAngleAmp(CommandXboxController xboxController, Amp amp, double angleRad, double velRad, double acceleRad) {
    this.xboxController = xboxController;
    this.amp = amp;
    this.angleRad = angleRad;
    this.velRad = velRad;
    this.acceleRad = acceleRad;
    this.trap = new Trapezoid(velRad, acceleRad);
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.neonEncoderReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    if(amp.getPoseRad()>=angleRad){
      return true;
    }
    return false;
  }
}
