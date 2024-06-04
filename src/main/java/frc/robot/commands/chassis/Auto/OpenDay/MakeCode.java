
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
    cmd = new SequentialCommandGroup(
        takeNote()

    );



    
    cmd.schedule();
  }

  @Override
  public boolean isFinished() {
    return cmd.isFinished();
  }
  
}

