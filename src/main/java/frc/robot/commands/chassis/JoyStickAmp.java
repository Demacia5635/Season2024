// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Amp;

public class JoyStickAmp extends Command {
  private final CommandXboxController xboxController;
  private Amp amp;
  private Translation2d translation2d1;
  private Translation2d translation2d2;
  /** Creates a new JoyStickAmpMove. */
  public JoyStickAmp(CommandXboxController xboxController, Amp amp) {
    this.xboxController = xboxController;
    this.amp = amp;
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void initialize() {
    amp.startRad(amp.deg());
  }

  // Called when the command is initially scheduled.

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    translation2d1 = Amp.getStickRight(xboxController);
    translation2d2 = Amp.getStickLeft(xboxController);
    amp.setPowerArm(translation2d1.getY());
  }

  @Override
    public void end(boolean interrupted) {
        amp.stop();
    }
}