package frc.robot;

import static frc.robot.utils.Utils.angelErrorInRadians;
import static frc.robot.utils.Utils.speakerPosition;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.StringIdGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.commands.shooter.AngleCalibrate;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionLimelight;
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Auto.StartTOP;
import frc.robot.commands.chassis.Auto.StartTOP1;
import frc.robot.commands.chassis.Auto.AutoChooser;
import frc.robot.commands.chassis.Auto.StartBottom1;
import frc.robot.commands.chassis.Auto.StartMiddle1;
import frc.robot.commands.chassis.Paths.GoToAMP;
import frc.robot.commands.chassis.Paths.GoToAMP1;
import frc.robot.commands.chassis.Paths.PathUtils.TestForPathPlannerGUI;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeToShooter;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LedControll;

public class RobotContainer implements Sendable {

  TestForPathPlannerGUI testForPathPlannerGUI = new TestForPathPlannerGUI("C:/Users/User/Season2024-2/src/main/deploy/pathplanner/paths/New Path.path");



  public static RobotContainer robotContainer;
  private Boolean isRed = true;
  CommandXboxController commandController;
  public CommandXboxController commandController2;
 
  public Shooter shooter;
  public Amp amp;
  public Intake intake;
  public Chassis chassis;
  public VisionLimelight vision;
  public LedControll led;

  public Command shoot; // shoot to amp or to speaker
  public Command activateShooter; // prepare to shoot
  public Command autonomousRight;
  public Command driveToNote;
  public Command manualIntake;
  public Command activateAmp;
  public Command disableCommand;
  public Command resetOdometry;

  private static final String Forword_oton = "rorword";
  private static final String Smale_oton = "smal";
  private static final String Big_oton = "big";

  private SendableChooser<Command> autoChoose;

  public RobotContainer() {
    robotContainer = this;
    
    chassis = new Chassis();
    vision = new VisionLimelight(chassis, chassis.getSwerveDrivePoseEstimator());
    intake = new Intake();
    // amp = new Amp();
    commandController2 = new CommandXboxController(1);
    commandController = new CommandXboxController(0);
    shooter = new Shooter();
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController));
    led = new LedControll(9, 110);
  
    createCommands();
    
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
    shoot = shooter.shootCommand();
    activateShooter = shooter.activateShooterToSpeaker();
    manualIntake = new IntakeCommand(intake);
    activateAmp = shooter.activateShooterToAmp();
    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward()).ignoringDisable(true);
    disableCommand = new InstantCommand(()-> stopAll(),intake, shooter).andThen(new AngleCalibrate(shooter));

    Command startTop = new ActivateShooter(shooter, intake, chassis, true).alongWith(new StartTOP1());
    Command StartMiddle = new ActivateShooter(shooter, intake, chassis, true).alongWith(new StartMiddle1());
    Command startBottom = new ActivateShooter(shooter, intake, chassis, true).alongWith(new StartBottom1());
    autoChoose = new SendableChooser<Command>();
    autoChoose.setDefaultOption("Top",startTop);
    autoChoose.addOption("Middle", StartMiddle);
    autoChoose.addOption("Bottom", startBottom);
    SmartDashboard.putData("Auto Chooser", autoChoose);

}

  public void isRed(boolean isRed) {
    this.isRed = isRed;
  }
  public boolean isRed() {
    return isRed;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is Red",this::isRed, this::isRed);
  }

  private void configureBindings() {
    // Buttons:
    //Driver controller
    // Y - Auto Intake
    // X - Activate Shooter
    // B - Drive precision mode
    // A - Manual Intake.
    // POV-UP - Shoot
    // POV-Down -
    // POV-Left - Go to Amp and shoot
    // POV-Right - Active shooter to shoot from subwaffer
    // LeftBumber - disable all
    //RightBumper - speaker from subwoofer
    // Back - chassie gyro set
    //Start - Amp

    //Operator Controller



    Trigger overrideAuto = new Trigger(() -> Utils.joystickOutOfDeadband(commandController));

    commandController.y().onTrue(driveToNote);
    commandController.x().onTrue(activateShooter);
    // B - in Drive Command
    commandController.a().onTrue(manualIntake);
    commandController.pov(0).whileTrue(shoot);
    commandController.start().onTrue(activateAmp);
    commandController.back().onTrue(resetOdometry);
    commandController.leftBumper().onTrue(disableCommand);
    commandController.pov(270).onTrue(new GoToAMP1());
    commandController.rightBumper().onTrue(shooter.activateShooterToSpeakerFromSub());
    overrideAuto.onTrue(chassis.getDefaultCommand());


    commandController2.b().onTrue(new InstantCommand((()->vision.setResetOdo(true))).ignoringDisable(true));
    commandController2.y().onTrue(shooter.activateShooterToSpeakerFromSub());
    commandController2.x().onTrue(activateShooter);
    commandController2.a().onTrue(activateAmp);
    commandController2.pov(180).onTrue(
      (new RunCommand(()->shooter.feedingSetPow(-0.5), intake).withTimeout(0.2))
      .andThen(new InstantCommand(()->shooter.feedingSetPow(0), intake)));
}

  public void calibrate() {
    disableCommand.schedule();
  }

   
  public Command getAutonomousCommand() {
    System.out.println(testForPathPlannerGUI.getPointsNoRadius()[1]);
    return new WaitCommand(15);
    //return new StartTOP1().alongWith(new ActivateShooter(shooter, intake, chassis, true));
    //return new RunCommand(()->shooter.setVel(10), shooter);
  }
}
