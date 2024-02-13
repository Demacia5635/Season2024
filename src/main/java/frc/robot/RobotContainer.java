package frc.robot;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.PathFollow;
import frc.robot.commands.vision.GoToNoteCommand;
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
//      commandController.a().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
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
    //return new RunCommand(()-> chassis.setModulesAngularVelocity(50), chassis);
    // return new GoToNoteCommand(chassis);
    // return new InstantCommand(() -> chassis.resetWheels(), chassis)
    // .andThen(new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(-2, 0, 0))).withTimeout(2).andThen(new InstantCommand(() -> chassis.stop())));
    //return new RunCommand(() -> chassis.getModule(2).setAngularVelocity(600));
    // return new RunCommand(() -> chassis.setModulesPower(0.05));
    // return new RunCommand(()-> chassis.setModulesAngularPower(-0.3), chassis);
    //return new RunCommand(()->{chassis.getModule(2).setAngularPower(0.049 + 300*0.0003);},chassis).withTimeout(3)
    //  .andThen(new InstantCommand(()->{SmartDashboard.putNumber("FF TEST",  chassis.getModule(2).getAngularVelocity());
    //chassis.stop();}));
    pathPoint[] pointArrUp = {new pathPoint(3, 2.5, Rotation2d.fromDegrees(0), 0, false), new pathPoint(3, 3.25, Rotation2d.fromDegrees(0), 0, false), new pathPoint(3, 4, Rotation2d.fromDegrees(0), 0, false)};
    pathPoint[] pointArrDown = {new pathPoint(3, 3, Rotation2d.fromDegrees(0), 0, false), new pathPoint(3, 2.5, Rotation2d.fromDegrees(0), 0, false), new pathPoint(3, 1.75, Rotation2d.fromDegrees(0), 0, false), new pathPoint(3, 0.5, Rotation2d.fromDegrees(0), 0, false)};

    pathPoint[] pointArrBack = {new pathPoint(2.5, 2.2, Rotation2d.fromDegrees(0), 0, false), new pathPoint(3, 2.2, Rotation2d.fromDegrees(0), 0, false), new pathPoint(3.5, 2.2, Rotation2d.fromDegrees(0), 0, false), new pathPoint(4, 2.2, Rotation2d.fromDegrees(0), 0, false)};
    pathPoint[] pointArrForward = {new pathPoint(3.5, 2.2, Rotation2d.fromDegrees(0), 0, false), new pathPoint(3, 2.2, Rotation2d.fromDegrees(0), 0, false), new pathPoint(2.5, 2.2, Rotation2d.fromDegrees(0), 0, false), new pathPoint(2, 2.2, Rotation2d.fromDegrees(0), 0, false)};

    return new PathFollow(chassis,pointArrUp , 1, 1).andThen(new PathFollow(chassis,pointArrDown , 2, 1.5).
    andThen(new PathFollow(chassis,pointArrUp , 2.5, 1.75).andThen(new PathFollow(chassis,pointArrDown , 3, 2).
    andThen(new PathFollow(chassis,pointArrUp , 3.5, 2).andThen(new PathFollow(chassis,pointArrDown , 4, 2.5))))));

    // return new PathFollow(chassis,pointArrBack , 1, 1).andThen(new PathFollow(chassis,pointArrForward , 2, 1.5).
    // andThen(new PathFollow(chassis,pointArrBack , 2.5, 1.75).andThen(new PathFollow(chassis,pointArrForward , 3, 2).
    // andThen(new PathFollow(chassis,pointArrBack , 3.5, 2).andThen(new PathFollow(chassis,pointArrForward , 4, 2.5))))));
  }
}
