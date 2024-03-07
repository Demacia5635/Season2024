// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.Auto;
import static frc.robot.commands.chassis.Auto.AutoUtils.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Field;
import frc.robot.PathFollow.Util.pathPoint;

public class DestroyCenter extends Command {
  SequentialCommandGroup cmd = new SequentialCommandGroup(initShooter());
  pathPoint centerNote1 = offset(Field.CenterNotes[0], 0,0,0);
  pathPoint centerNote5 = offset(Field.CenterNotes[4], 0,0,0);




  /** Creates a new CollectTop. */
  public DestroyCenter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    addCommands(goTo(centerNote1).alongWith(shoot()), cmd);
    addCommands(goTo(centerNote5), cmd);


    cmd.schedule();
 

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
