// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.Auto;
import static frc.robot.commands.chassis.Auto.AutoUtils.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Field;
import frc.robot.PathFollow.Util.pathPoint;

public class CollectWing extends Command {
  SequentialCommandGroup cmd = new SequentialCommandGroup(initShooter());
  pathPoint wingNote1 = offset(Field.WingNotes[0], 0,0, 0);
  pathPoint wingNote2 = offset(Field.WingNotes[1], 0,0, 0);
  pathPoint wingNote3 = offset(Field.WingNotes[2], 0,0, 0);

  pathPoint shootPoint1 = offset(Field.Speaker, 0,0,0);
  pathPoint shootPoint2 = offset(Field.Speaker, 0,0,0);




  /** Creates a new CollectTop. */
  public CollectWing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    /*addCommands(getNote(wingNote1).alongWith(shoot()), cmd);
    addCommands(goTo(shootPoint, 2), cmd);
    addCommands(shoot(), cmd);
    addCommands(getNote(wingNote2), cmd);
    addCommands(goTo(shootPoint, 2), cmd);
    addCommands(shoot(), cmd);
    addCommands(getNote(wingNote3), cmd);
    addCommands(goTo(shootPoint, 2), cmd);
    addCommands(shoot(), cmd);
*/
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
