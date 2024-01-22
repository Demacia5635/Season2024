// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class DriveByGivenVelocity extends Command {
  Chassis chassis;
  double vX;
  double vY;
  double Angle;
  PIDController pid = new PIDController(0.5, 0, 0);
  public DriveByGivenVelocity(Chassis chassis, double vX, double vY, double Angle) {
    this.chassis = chassis;
    
    this.vX = vX;
    this.vY = vY;
    this.Angle = Angle;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    ChassisSpeeds speeds = new ChassisSpeeds(vX, vY, pid.calculate(Angle));
    chassis.setVelocities(speeds);

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
