// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.Auto;
import static frc.robot.commands.chassis.Auto.AutoUtils.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Field;
import frc.robot.PathFollow.Util.pathPoint;

public class CollectBottom extends Command {
  SequentialCommandGroup cmd;
  pathPoint escapePoint1 = offset(Field.WingNotes[2], 0,-3, 0);
  pathPoint escapePoint2 = offset(Field.CenterNotes[4], -1,0, 0);

  pathPoint centerNote1 = offset(Field.CenterNotes[0], 0,0,0);
  pathPoint centerNote2 = offset(Field.CenterNotes[1], 0,0,0);
  pathPoint shootPoint = offset(Field.Speaker, 0,0,0);




  /** Creates a new CollectTop. */
  public CollectBottom() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup(shoot(1));
    addCommands(goTo(escapePoint1, 3), cmd);
    addCommands(goTo(escapePoint2, 3), cmd);
    addCommands(takeNote(), cmd);
    addCommands(goTo(escapePoint2, 3), cmd);
    addCommands(goTo(escapePoint1, 3), cmd);
    addCommands(turnToSpeakerAndWaitForReady(), cmd);
    addCommands(shoot(0), cmd);



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
