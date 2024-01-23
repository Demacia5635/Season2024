// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;
import static frc.robot.subsystems.chassis.ChassisConstants.*;
public class DriveWithAnglePID extends Command {
  private final Chassis chassis;
  private final CommandXboxController controller;

  private boolean precisionDrive = false;
 
  public DriveWithAnglePID(Chassis chassis, CommandXboxController controller) {
    this.controller = controller;
    this.chassis = chassis;
    controller.b().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
    addRequirements(chassis);
  }



  @Override
  public void execute() {
    double joyX = deadband(controller.getLeftY(), 0.1);
    double joyY = deadband(controller.getLeftX(), 0.1);
    double rot = -(deadband(controller.getRightTriggerAxis(), 0.1) - deadband(controller.getLeftTriggerAxis(), 0.1));
    double angle = getAngleOfJoystick().getDegrees();
    
    double velX = Math.pow(joyX, 3)* MAX_DRIVE_VELOCITY;
    double velY = Math.pow(joyY, 3) * MAX_DRIVE_VELOCITY;
    double velRot;

    if(angle >= -3 && angle <= 3) velRot = Math.pow(rot, 3) * MAX_OMEGA_VELOCITY;
    else velRot = rotationPid.calculate(chassis.getAngle().getDegrees(), angle) * MAX_OMEGA_VELOCITY;
    System.out.println("angle: " + angle);
    
    
    
    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }


    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    chassis.setVelocities(speeds);
  }

  private Rotation2d getAngleOfJoystick(){
    double angle = new Rotation2d(controller.getRightX(), controller.getRightY()).getDegrees();
    return new Rotation2d(angle);
  }
  private double deadband(double x, double threshold) {
    return (Math.abs(x) < threshold)?0:x;
  }
}