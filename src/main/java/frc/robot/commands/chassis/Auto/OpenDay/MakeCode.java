
package frc.robot.commands.chassis.Auto.OpenDay;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.commands.chassis.Auto.AutoUtils.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Field;
import frc.robot.PathFollow.Util.pathPoint;

public class MakeCode extends Command {
  SequentialCommandGroup cmd;
  pathPoint example = offset(Field.WingNotes[0], 0,0, 0);


  /** Creates a new CollectTop. */
  public MakeCode() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup(goTo(example,1.6, true));
    addCommands(new WaitCommand(0.5), cmd);
    addCommands(MoveForward(1), cmd);
    addCommands(MoveBackwards(1), cmd);
    addCommands(MoveLeft(1), cmd);
    addCommands(MoveRight(1), cmd);
    addCommands(setShooterToShoot(), cmd);
    addCommands(shootWhenReady(), cmd);


    
    cmd.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd.isFinished();
  }
  
}

