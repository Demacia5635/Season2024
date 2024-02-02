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
import frc.robot.commands.chassis.RotateToAngleShooter;
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
  double x = 5;



  public RobotContainer() {
    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("RC", this);

    configureBindings();
  }


  @Override
  public void initSendable(SendableBuilder builder) {

    builder.addDoubleProperty("chassis angle",() -> chassis.getAngle().getDegrees(), null);
    SmartDashboard.putData("Reset Odometry", new InstantCommand(() -> chassis.resetOdometry()));



  }

    
    private void configureBindings() {
      commandController.a().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
      commandController.y().onTrue(new InstantCommand(() -> chassis.resetOdometry()));
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
  
    return new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0.5, 0, 0)), chassis).withTimeout(4).
    andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(-1, 0, 0)), chassis).withTimeout(3.5)).
    andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(1.5, 0, 0)), chassis).withTimeout(3)).
    andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(-2, 0, 0)), chassis).withTimeout(3)).
    andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(2.5, 0, 0)), chassis).withTimeout(2.5)).
    andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(-3, 0, 0)), chassis).withTimeout(2)).
    andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(3.5, 0, 0)), chassis).withTimeout(1.5)).
    andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(-4, 0, 0)), chassis).withTimeout(1));
  

    // return new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, 0.5, 0)), chassis).withTimeout(4).
    // andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, -1, 0)), chassis).withTimeout(3.5)).
    // andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, 1.5, 0)), chassis).withTimeout(3)).
    // andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, -2, 0)), chassis).withTimeout(3)).
    // andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, 2.5, 0)), chassis).withTimeout(2.5)).
    // andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, -3, 0)), chassis).withTimeout(2)).
    // andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, 3.5, 0)), chassis).withTimeout(1.5)).
    // andThen(new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, -4, 0)), chassis).withTimeout(1));
  }
}
