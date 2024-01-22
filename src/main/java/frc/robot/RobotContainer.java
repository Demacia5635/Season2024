package frc.robot;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.SetModuleAngle;
import frc.robot.commands.chassis.TestSteerVandA;
import frc.robot.subsystems.chassis.Chassis;


public class RobotContainer implements Sendable{
  CommandXboxController commandController = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  DriveCommand drive = new DriveCommand(chassis, commandController);
  Command test = new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.5, 0, 0));}, chassis).andThen(new WaitCommand(2),
  new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.4  , 0, 0));}, chassis).andThen(new WaitCommand(2)),
  new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(0, 0, 0));}, chassis).andThen(new WaitCommand(2)));
  double x = 0.2;

 
  public RobotContainer() {
    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("RC", this);

    configureBindings();
  }


  @Override
  public void initSendable(SendableBuilder builder) {


  }

    
    private void configureBindings() {
      commandController.a().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
//      commandController.start().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
        // code for controller to controll the gripper and the parallelogram

        // safty buttons to stop the arm and/or the gripper
    }
   
  /**
   * Use this to pass the autonomous command to the main {@link Robot} cass.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0.5, 0, 0)), chassis);
    //    return new RunCommand(()-> chassis.setModulesSteerVelocity(500), chassis);
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
