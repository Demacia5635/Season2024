package frc.robot;

import javax.swing.plaf.metal.MetalTheme;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.amp.AmpIntake;
import frc.robot.commands.amp.GoToAngleAmp;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.JoyStickAmp;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.chassis.Chassis;


public class RobotContainer implements Sendable{
  public static CommandXboxController commandController;
  Chassis chassis;
  Amp amp;
  JoyStickAmp moveAmp;
  //DriveCommand drive;
  //double x = 0.2;

 
  public RobotContainer() {

    commandController = new CommandXboxController(Constants.CONTROLLER_PORT);
    amp = new Amp();
    //chassis = new Chassis();
    //drive = new DriveCommand(chassis, commandController);

    //chassis.setDefaultCommand(drive);
    //SmartDashboard.putData("RC", this);
    moveAmp = new JoyStickAmp(commandController, amp);
    amp.setDefaultCommand(moveAmp);

    configureBindings();
  }


  @Override
  public void initSendable(SendableBuilder builder) {

    //builder.addDoubleProperty("spinspeed", () -> x, null);

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
      //commandController.start().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
        // code for controller to controll the gripper and the parallelogram

        // safty buttons to stop the arm and/or the gripper
    }
   
  /**
   * Use this to pass the autonomous command to the main {@link Robot} cass.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return null;
    //return new GoToAngleAmp(amp, Math.PI*0.7, Math.PI*0.4, Math.PI*0.5);
    return new AmpIntake(amp, 2,1).withTimeout(3)
    .andThen(new GoToAngleAmp(amp, Math.PI*0.7, Math.PI*0.4, Math.PI*0.5))
    .andThen(new AmpIntake(amp, 2,1));
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
