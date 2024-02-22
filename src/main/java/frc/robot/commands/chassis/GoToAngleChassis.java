// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.utils.Trapezoid;
import frc.robot.utils.TrapezoidNoam;

public class GoToAngleChassis extends Command {
  /** Creates a new GoToAngleChassis. */
  Rotation2d wantedAngle;
  Chassis chassis;
  TrapezoidNoam rotationTrapezoid;
  
  PIDController rotationPidController = new PIDController(0.23, 0.00, 0.006);
  public GoToAngleChassis(Chassis chassis, Rotation2d wantedAngle) {
    this.wantedAngle = wantedAngle;
    this.chassis = chassis;
    //rotationTrapezoid = new TrapezoidNoam(180, 360);
    addRequirements(chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ChassisSpeeds speed =  new ChassisSpeeds(0, 0, 
    Math.toRadians(rotationPidController.calculate(chassis.getAngle().getDegrees(), wantedAngle.getDegrees())));
    chassis.setVelocities(speed); 
  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(wantedAngle.getDegrees() - chassis.getAngle().getDegrees()) <= 3;
  }
}
