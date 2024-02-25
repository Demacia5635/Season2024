package frc.robot;

import static frc.robot.utils.Utils.speakerPosition;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.AngleQuel;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Auto.StartTOP;
import frc.robot.commands.chassis.Paths.GoToAMP;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LedControll;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;
  private Boolean isRed = true;
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
  public Chassis chassis;
  Vision vision;
  // SubStrip leds;
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
    
    chassis = new Chassis();
    vision = new Vision(chassis, chassis.getSwerveDrivePoseEstimator());
    intake = new Intake();
    // amp = new Amp();
    commandController2 = new CommandXboxController(1);
    commandController = new CommandXboxController(0);
    shooter = new Shooter();
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController));

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

    SmartDashboard.putNumber("wanted vel", 0);
    SmartDashboard.putNumber("wanted angle", 20);

    driveToNote = new DriveToNote(chassis, 1).raceWith(new IntakeCommand(intake));
    // shoot = new IntakeToShooter(intake, shooter, vel);
    shoot = shooter.getShootCommand();
    // activateShooter = new ShooterPowering(shooter, vel).alongWith(new AngleGoToAngle(shooter, angle));
    activateShooter = shooter.getActivateShooterToSpeaker();
    manualIntake = new IntakeCommand(intake);
    activateAmp = shooter.getActivateShooterToAmp();
    resetOdometry = new InstantCommand(()-> {chassis.setOdometryToForward();}, chassis);
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
    // POV-Left - Go to Amp and shoot
    // RightBumper - disable all
    // Back - chassie gyro set

    Trigger overrideAuto = new Trigger(() -> Utils.joystickOutOfDeadband(commandController));



    commandController.y().onTrue(driveToNote);
    commandController.x().onTrue(activateShooter);
    // B - in Drive Command
    commandController.a().onTrue(manualIntake);
    commandController.x().onTrue(activateShooter.alongWith(new GoToAngleChassis(chassis, Utils.speakerPosition())));


    //commandController.x().onTrue(activateShooter);
    // commandController.x().onTrue(new AngleGoToAngle(shooter, Utils.getShootingAngleVelocity(
    //speakerPosition().getDistance(chassis.getPose().getTranslation())).getFirst())
    // .alongWith(new ShooterPowering(shooter, Utils.getShootingAngleVelocity(
    //   speakerPosition().getDistance(chassis.getPose().getTranslation())).getSecond())));
   // commandController.b().onTrue(new AngleGoToAngle(shooter, ShooterConstants.AmpPera.ANGLE).alongWith(new RunCommand(()->shooter.setVel(ShooterConstants.AmpPera.UP, ShooterConstants.AmpPera.DOWN), shooter)));
   commandController.b().onTrue(activateAmp); 
   commandController.pov(0).whileTrue(new IntakeToShooter(intake, shooter, 15));
    commandController.pov(180).whileTrue(new IntakeToShooter(intake, shooter, ShooterConstants.AmpPera.UP, ShooterConstants.AmpPera.DOWN));
    commandController.back().onTrue(resetOdometry);
    commandController.leftBumper().onTrue(disableCommand);

    commandController.pov(270).onTrue(new GoToAMP(shooter, chassis, intake, true).raceWith(new WaitCommand(2)));
    
    /*commandController.x().onTrue(activateShooter);
    commandController.povRight().onTrue(new AngleControl(shooter, commandController));
    // B - defined in Drive

    
    commandController.pov(0).whileTrue(shoot);
    commandController.pov(180).onTrue();
    commandController.x().onTrue();
    commandController.pov(270).onTrue(new 
    GoToAMP(shooter, chassis, intake, false));
    
    commandController.pov(90).onTrue(new InstantCommand(()-> chassis.setPose(new Pose2d(new Translation2d(1.75, 5.5), new Rotation2d())))); */
    overrideAuto.onTrue(chassis.getDefaultCommand());
}

  public void calibrate() {
    disableCommand.schedule();
  }

   
  public Command getAutonomousCommand() {
    // return null;
    //return new RunCommand(()->shooter.angleSetPow(.1), shooter);
    return new StartTOP(chassis, shooter, intake, isRed);
   //return new PathFollow(chassis, points, 4, 12, 0, false);
  }
}
