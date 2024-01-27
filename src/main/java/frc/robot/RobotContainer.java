package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
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
import frc.robot.subsystems.chassis.Chassis;


public class RobotContainer implements Sendable{
  CommandXboxController commandController = new CommandXboxController(0);
  PS4Controller controller = new PS4Controller(1);  
  Chassis chassis = new Chassis();
  pathPoint[] points = {
    new pathPoint(0, 0, Rotation2d.fromDegrees(-90), 0.5, true),
    
    new pathPoint(-2, -2, Rotation2d.fromDegrees(-90), 0.7, true),
    new pathPoint(0, -4, Rotation2d.fromDegrees(90), 0.5, false)
   };

  pathPoint[] points1 = {
    new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, true),
    new pathPoint(0, -6, Rotation2d.fromDegrees(0), 0, true),
    new pathPoint(1, 0, Rotation2d.fromDegrees(0), 0, true),
  };

  DriveCommand drive = new DriveCommand(chassis, controller, commandController);
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

    builder.addDoubleProperty("spinspeed", () -> Math.toDegrees(chassis.getChassisSpeeds().omegaRadiansPerSecond), null);

  }
 private void configureBindings() {
      if(controller.getCrossButton()) new InstantCommand(()->{chassis.setOdometryToForward();});
    }
   
  public Command getAutonomousCommand() {
    return new PathFollow(chassis, points, 4, 14);
    
  }
}
