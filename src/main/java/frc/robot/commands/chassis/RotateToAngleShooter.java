// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;
import static frc.robot.subsystems.chassis.ChassisConstants.*;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;

public class RotateToAngleShooter extends Command {
  private final Chassis chassis;
  private boolean precisionDrive = false;
  PIDController pid = new PIDController(0.0001, 0.0002,0.000002);
  double angle;
  //double kp = 0.004;

  public RotateToAngleShooter(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    chassis.setGyroAngle(0);
   
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("angle", () -> angle,(double angle)-> {this.angle = angle;});
  }
  @Override
  public void execute() {
    angle = -2;
    
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, pid.calculate(chassis.getAngle().getDegrees(), angle) * 180);
    chassis.setVelocities(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }


    @Override
    public boolean isFinished() {
      return chassis.getAngle().getDegrees() - angle <= 1 && chassis.getAngle().getDegrees() - angle >= -1;
    }
  }
