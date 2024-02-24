// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.amp.Amp;

public class climb_amp extends ParallelRaceGroup {
  /** Creates a new climb_amp. */
  public climb_amp(Amp amp) {
    addCommands(new GoToAngleAmp(amp, Math.toRadians(160), 4, 4).andThen(new Brake_Amp(amp, true)));
  }

}
