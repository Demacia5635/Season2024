package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.climb.ClimbManualCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class RobotContainer {
  private final CommandXboxController commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
  private final XboxController controller = new XboxController(Constants.CONTROLLER_PORT);
  private final ClimbSubsystem climb = new ClimbSubsystem();
  private final ClimbManualCommand climbMan = new ClimbManualCommand(climb, controller);
 
  public RobotContainer() {
    climb.setDefaultCommand(climbMan);

    configureBindings();
  }

  private void configureBindings() {
    
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
}
