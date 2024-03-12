// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.Auto;
import static frc.robot.commands.chassis.Auto.AutoUtils.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Field;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.Paths.GoTo;

public class CollectWing extends Command {
  SequentialCommandGroup cmd = new SequentialCommandGroup(initShooter());
  pathPoint wingNote1 = offset(Field.WingNotes[0], 0,0, 0);
  pathPoint wingNote2 = offset(Field.WingNotes[1], 0,0, 0);
  pathPoint wingNote3 = offset(Field.WingNotes[2], 0,0, 0);

  
  //pathPoint shootPoint = offset(Field.Speaker, 0,0,0);
  pathPoint shootPoint = new pathPoint(new Translation2d(1.45, 5.5), Rotation2d.fromDegrees(0));
  pathPoint shootPointAnchor = new pathPoint(new Translation2d(1.55, 6.37), Rotation2d.fromDegrees(0), 0.3);
  pathPoint secondAnchorPoint = new pathPoint(new Translation2d(1.49, 4.75), Rotation2d.fromDegrees(-35), 0.3);
  pathPoint[] firstPoints = {
    dummyPoint, shootPointAnchor, shootPoint
  };


  /** Creates a new CollectTop. */
  public CollectWing() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    addCommands(new WaitCommand(0.2), cmd);

    addCommands((shoot()), cmd);
    addCommands(new WaitCommand(0.6), cmd);
    addCommands(takeNote(), cmd);
    //addCommands(new WaitCommand(1), cmd);
    //addCommands(new IsChassisInAngle(chassis), cmd);
    addCommands(goToMultiple(firstPoints, 1.5), cmd);
    addCommands(new WaitCommand(0.4), cmd);

    addCommands(shoot(), cmd);
    addCommands(new WaitCommand(0.7), cmd);

    addCommands(takeNote(), cmd);
    

    //addCommands(goTo(shootPoint, 2), cmd);

    addCommands(goTo(secondAnchorPoint), cmd);
    addCommands(new WaitCommand(0.5), cmd);

    addCommands(shoot(), cmd);
        
    addCommands(new WaitCommand(1.5), cmd);

    addCommands(takeNote(), cmd);
    addCommands(new WaitCommand(0.5), cmd);

    addCommands(shoot(), cmd);

    //addCommands(getNote(wingNote3), cmd);
    //addCommands(goTo(shootPoint, 2), cmd);
    //addCommands(shoot(), cmd);


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
