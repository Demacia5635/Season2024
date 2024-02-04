// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.NeoMotor;

public class NeoTest extends Command {
  NeoMotor neo;
  /** Creates a new NeoTest. */
  public NeoTest() {
    this.neo = new NeoMotor();
    addRequirements(this.neo);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
      public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Encoder", () -> {return getEncoder();}, null);
      }


  public double getEncoder()
  {
      return neo.neo.getEncoder().getPosition();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    neo.neo.setSmartCurrentLimit(25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      neo.neo.set(0.2);

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
