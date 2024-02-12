package frc.robot;


import java.util.Optional;

import com.fasterxml.jackson.core.StreamReadConstraints.Builder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import static frc.robot.subsystems.chassis.ChassisConstants.*;


public class RobotContainer implements Sendable{
  Boolean isRed = false;
  //DriverStation.Alliance alliance;
  CommandXboxController commandController = new CommandXboxController(0);
  PS4Controller controller = new PS4Controller(1);  
  Chassis chassis = new Chassis();

  pathPoint[] points1 = {
    new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
    new pathPoint(2, 0, Rotation2d.fromDegrees(0), 0, false)
  };
  pathPoint[] points2 = {
    new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
    new pathPoint(2, 2, Rotation2d.fromDegrees(0), 0, false)
  };



  public RobotContainer() {

    //alliance = DriverStation.getAlliance().get();
    //isRed = (alliance == Alliance.Red);
    DriveCommand drive = new DriveCommand(chassis, controller, commandController, isRed);
    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("RC", this);
    

    configureBindings();
  }


  @Override
  public void initSendable(SendableBuilder builder) {

    builder.addDoubleProperty("chassis angle",() -> chassis.getAngle().getDegrees(), null);
    //builder.addStringProperty("Alliance",() -> alliance.toString(), null);

  }
 private void configureBindings() {
      if(controller.getCrossButton()) new InstantCommand(()->{chassis.setOdometryToForward();});
    commandController.x().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
    
    }
   
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(new PathFollow(chassis, points1, 2, 4, 1, DriverStation.getAlliance().get() == Alliance.Red)
    .andThen(new PathFollow(chassis, points2, 2, 1, 0, DriverStation.getAlliance().get() == Alliance.Red)));
   
  }
}
