package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.AngleControl;
import frc.robot.commands.shooter.GoToAngle;
import frc.robot.commands.shooter.GoToDis;
import frc.robot.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer  {
  
    Shooter shooter;
    CommandXboxController controller;
    
    public RobotContainer() {
        shooter = new Shooter();
        controller = new CommandXboxController(0);
        configureBindings();
    }
    
    /**
         * 
         * Use this method to define your trigger->command mappings. Triggers can be created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
         * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
            controller.b().onTrue(new AngleControl(shooter, controller));
            controller.x().onTrue(new GoToDis(shooter, 245, 30000, 20000));
            controller.a().onTrue(new GoToAngle(shooter, 39, 30000, 20000));
            
            controller.y().onTrue(new InstantCommand(()-> shooter.resetDis(), shooter));    
            
            controller.rightBumper().onTrue(new InstantCommand(()-> shooter.stopAll(),shooter).ignoringDisable(true));
        }
        
    /**
     * Use this to pass the autonomous command to the main {@link Robot} cass.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand(()-> shooter.setVel(42550), shooter);
    }
}
