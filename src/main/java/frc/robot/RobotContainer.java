package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climb.ClimbManualCommand;
import frc.robot.subsystems.climb.Telescope;


public class RobotContainer {
  private final CommandXboxController commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
  private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);
  private final Telescope telescope = new Telescope();
  private final ClimbManualCommand climbMan = new ClimbManualCommand(telescope, controller);

 
  public RobotContainer() {
    telescope.setDefaultCommand(climbMan);

    configureBindings();
  }

  private void configureBindings() {
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
}
