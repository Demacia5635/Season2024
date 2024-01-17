// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import static frc.robot.Constants.ClimbConstants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;
import static frc.robot.Constants.ClimbConstants.*;


public class TestClimbCommand extends Command {
  ClimbSubsystem climb;
  /** Creates a new TestClimbCommand. */
  public TestClimbCommand(ClimbSubsystem climb) {
    this.climb = climb;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climb.getLeftVolteg() < MI_WON_DAY_VOLTEG){
      climb.climbLeft(0.6);
    }
    if (climb.getRoghtVolteg() < MI_WON_DAY_VOLTEG){
      climb.climbRight(0.6);
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
