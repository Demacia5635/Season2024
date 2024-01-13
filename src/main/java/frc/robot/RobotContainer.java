package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.subsystems.chassis.Chassis;


public class RobotContainer {
  CommandXboxController commandController;
  Chassis chassis;
  DriveCommand drive;

 
  public RobotContainer() {

    commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
    chassis = new Chassis();
    drive = new DriveCommand(chassis, commandController);

    chassis.setDefaultCommand(drive);
    

    
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
        // code for controller to controll the gripper and the parallelogram

        // safty buttons to stop the arm and/or the gripper
    }
   
  /**
   * Use this to pass the autonomous command to the main {@link Robot} cass.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new RunCommand(()-> chassis.setModulesAngleFromSB(90), chassis);
    // return new InstantCommand(() -> chassis.resetWheels(), chassis)
    // .andThen(new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(-2, 0, 0))).withTimeout(2).andThen(new InstantCommand(() -> chassis.stop())));
    //return new RunCommand(() -> chassis.getModule(2).setAngularVelocity(600));
    // return new RunCommand(() -> chassis.setModulesPower(0.05));
    // return new RunCommand(()-> chassis.setModulesAngularPower(-0.3), chassis);
    //return new RunCommand(()->{chassis.getModule(2).setAngularPower(0.049 + 300*0.0003);},chassis).withTimeout(3)
    //  .andThen(new InstantCommand(()->{SmartDashboard.putNumber("FF TEST",  chassis.getModule(2).getAngularVelocity());
    //chassis.stop();}));
  }
}
