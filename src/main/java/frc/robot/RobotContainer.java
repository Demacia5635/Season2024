package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.commands.shooter.AngleGoToAngle;
import frc.robot.commands.shooter.AngleQuel;
import frc.robot.commands.shooter.ShooterPowering;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.amp.AmpIntake2;

import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.Auto.StartTOP;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.DispenseCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeToShooter;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LedControll;
import frc.robot.subsystems.leds.SubStrip;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;
  private Boolean isRed = false;
  CommandXboxController commandController;
  CommandXboxController commandController2;
  PS4Controller controller = new PS4Controller(1);

  // pathPoint[] points = {
  // new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
  // new pathPoint(2, 3, Rotation2d.fromDegrees(0), 0, false),
  // };
  // Command test = new RunCommand(() -> {chassis.setVelocities(new
  // ChassisSpeeds(-0.5, 0, 0));}, chassis).andThen(new WaitCommand(2),
  // new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.4 , 0,
  // 0));}, chassis).andThen(new WaitCommand(2)),
  // new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(0, 0, 0));},
  // chassis).andThen(new WaitCommand(2)));
  // double x = 5;

  public Shooter shooter;
  public Amp amp;
  public Intake intake;
  // public Chassis chassis;
  // Vision vision;
  SubStrip leds;
  LedControll led;

  public Command shoot; // shoot to amp or to speaker
  public Command activateShooter; // prepare to shoot
  public Command autonomousRight;
  public Command driveToNote;
  public Command manualIntake;
  public Command activateAmp;
  public Command disableCommand;
  public Command resetOdometry;

  public RobotContainer() {
    robotContainer = this;
    
    // chassis = new Chassis();
    // vision = new Vision(chassis, chassis.getSwerveDrivePoseEstimator());
    intake = new Intake();
    // amp = new Amp();
    commandController2 = new CommandXboxController(1);
    commandController = new CommandXboxController(0);
    shooter = new Shooter();
    // chassis.setDefaultCommand(new DriveCommand(chassis, commandController));

    createCommands();

    led = new LedControll(9, 110);
    
    SmartDashboard.putData("RobotContainer", this);
    configureBindings();
  }

  public void stopAll() {
    shooter.stopAll();
    intake.stop();
  }
  public void createCommands() {
    double vel = 18;
    double angle = 35;

    // driveToNote = new DriveToNote(chassis).raceWith(new IntakeCommand(intake));
    shoot = new IntakeToShooter(intake, shooter, vel);
    // shoot = new InstantCommand(()->shooter.isShooting(true));
    activateShooter = new ShooterPowering(shooter, vel).alongWith(new AngleGoToAngle(shooter, angle));
    // activateShooter = new ActivateShooter(shooter, intake, chassis, false).
    //     alongWith(new InstantCommand(()->shooter.isShootingAmp(false)));
    manualIntake = new IntakeCommand(intake);
    // activateAmp = new ActivateShooter(shooter, intake, chassis, false).
        // alongWith(new InstantCommand(()->shooter.isShootingAmp(true)));
    // resetOdometry = new InstantCommand(()-> {chassis.setOdometryToForward();}, chassis);
    disableCommand = new InstantCommand(()-> stopAll(),intake, shooter).andThen(new AngleQuel(shooter));
}

  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }
  public boolean isRed() {
    return isRed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red Alliance", this::isRed, this::isRed);
  }

  private void configureBindings() {
    // Buttons:
    // Y - Auto Intake
    // X - Activate Shooter
    // B - Drive precision mode
    // A - Manual Intake
    // POV-UP - Shoot
    // POV-Down - Amp
    // RightBumper - disable all
    // Back - chassie gyro set

    // Trigger overrideAuto = new Trigger(() -> Utils.joystickOutOfDeadband(commandController));

    // commandController.y().onTrue(driveToNote);
    commandController.x().onTrue(activateShooter);
    // B - defined in Drive
    commandController.a().onTrue(manualIntake);
    commandController.pov(0).whileTrue(shoot);
    // commandController.pov(180).onTrue(activateAmp);
    commandController.rightBumper().onTrue(disableCommand);
    // commandController.back().onTrue(resetOdometry);
    // overrideAuto.onTrue(chassis.getDefaultCommand());
}

  public void calibrate() {
    disableCommand.schedule();
  }
   
  public Command getAutonomousCommand() {
    return null;
    // return new StartTOP(chassis, shooter, intake, isRed);
   //return new PathFollow(chassis, points, 4, 12, 0, false);
  }
}
