// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.AmpUdi;
import frc.robot.utils.TrapezoidCalc;
import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.*;

public class GoToAngleAmp extends Command {
  AmpUdi amp;
  double angle;
  double maxVel;
  double accel;
  double currentAngle;
  TrapezoidCalc trap;

  /** Creates a new GoToAngleAmp. */
  public GoToAngleAmp(AmpUdi amp, double angleRad, double maxVelRad, double acceleRad) {
    this.amp = amp;
    this.angle = angleRad;
    this.maxVel = maxVelRad;
    this.accel = acceleRad;
    this.trap = new TrapezoidCalc();
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.setArmBrake();
    amp.unlock();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = amp.getArmAngle();
    boolean atPosition = atPosition();
    if (amp.isUnlocked()) {
      if (angle > currentAngle) { // going up
        double velRad = trap.trapezoid(currentAngle, maxVel, 0, Math.abs(accel), angle - currentAngle);
        amp.setArmVel(velRad);
      } else if (!atPosition) { // going down
        amp.setArmPower(ARM_DOWN_POWER);
      } else {
        amp.setArmVel(0);
      }
    }
    if (atPosition) {
      amp.lock();
    }
  }

  private boolean atPosition() {
    return Math.abs(angle - currentAngle) < ARM_RAD_ERROR;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.setLockPower(0);
    amp.setArmPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition() && amp.isLocked();
  }
}