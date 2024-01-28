package frc.robot;


import java.util.Optional;

import com.fasterxml.jackson.core.StreamReadConstraints.Builder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.chassis.RotateToAngleShooter;
import frc.robot.commands.chassis.SetModuleAngle;
import frc.robot.commands.chassis.TestSteerVandA;
import frc.robot.subsystems.chassis.Chassis;


public class RobotContainer implements Sendable{
  Boolean isRed = false;
  DriverStation.Alliance alliance;
  CommandXboxController commandController = new CommandXboxController(0);
  PS4Controller controller = new PS4Controller(1);  
  Chassis chassis = new Chassis();
  pathPoint[] points = {
    new pathPoint(0, 0, Rotation2d.fromDegrees(-90), 0.5, false),
    new pathPoint(1, 1, Rotation2d.fromDegrees(-90), .65, false),
    new pathPoint(2, 0, Rotation2d.fromDegrees(90), 0.5, false),

    new pathPoint(3, 1, Rotation2d.fromDegrees(90), 0.5, false),
    new pathPoint(4, 0, Rotation2d.fromDegrees(90), 0.5, false)

   };

  pathPoint[] points1 = {
    new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, true),
    new pathPoint(0, -6, Rotation2d.fromDegrees(0), 0, true),
    new pathPoint(1, 0, Rotation2d.fromDegrees(0), 0, true),
  };

 
  Command test = new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.5, 0, 0));}, chassis).andThen(new WaitCommand(2),
  new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.4  , 0, 0));}, chassis).andThen(new WaitCommand(2)),
  new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(0, 0, 0));}, chassis).andThen(new WaitCommand(2)));
  double x = 5;



  public RobotContainer() {

    alliance = DriverStation.getAlliance().get();
    if(alliance == Alliance.Red) isRed = true;
    else isRed = false;
    DriveCommand drive = new DriveCommand(chassis, controller, commandController, isRed);
    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("RC", this);
    

    configureBindings();
  }


  @Override
  public void initSendable(SendableBuilder builder) {

    builder.addDoubleProperty("chassis angle",() -> chassis.getAngle().getDegrees(), null);
    builder.addStringProperty("Alliance",() -> alliance.toString(), null);

  }
 private void configureBindings() {
      if(controller.getCrossButton()) new InstantCommand(()->{chassis.setOdometryToForward();});
    }
   
  public Command getAutonomousCommand() {
    return new PathFollow(chassis, points, 3, 6, isRed);
    
  }
}
