
package frc.robot.commands.chassis.Auto.leftPick3Close1Mid;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.chassis.Chassis;

public class leftPick3Close1Mid extends SequentialCommandGroup {
  Chassis chassis;
  public leftPick3Close1Mid(Chassis chassis) {
    this.chassis = chassis;
    addCommands(new leftPick3Close1MidGEN(chassis));
  }
}
