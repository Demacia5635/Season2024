// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.Auto.OpenDay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.chassis.Chassis;

public class MoveBackwards extends Command {
  double distance;
  Chassis chassis;
  double vel = 1;
  public MoveBackwards(double distance) {
   this.distance = distance;
   this.chassis = RobotContainer.robotContainer.chassis;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    chassis.setVelocities(new ChassisSpeeds(0, vel, 0));
    distance -= (vel * 0.02);



  }


  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return distance <= 0.05;
  }
}
