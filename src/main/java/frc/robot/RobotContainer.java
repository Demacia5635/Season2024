package frc.robot;

import static frc.robot.utils.Utils.speakerPosition;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.AngleQuel;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Auto.StartTOP;
import frc.robot.commands.chassis.Auto.StartTOP1;
import frc.robot.commands.chassis.Paths.GoToAMP;
import frc.robot.commands.chassis.Paths.GoToAMP1;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeToShooter;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LedControll;

public class RobotContainer implements Sendable {
  public static RobotContainer robotContainer;
  private Boolean isRed = true;
  CommandXboxController commandController;
  CommandXboxController commandController2;
 
  public Shooter shooter;
  public Amp amp;
  public Intake intake;
  public Chassis chassis;
  public Vision vision;
  public LedControll led;

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
    builder.addBooleanProperty("is Red",this::isRed, this::isRed);
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
    // POV-Right - Active shooter to shoot from subwaffer
    // RightBumper - disable all
    // Back - chassie gyro set

    Trigger overrideAuto = new Trigger(() -> Utils.joystickOutOfDeadband(commandController));

    commandController.y().onTrue(driveToNote);
    commandController.x().onTrue(activateShooter);
    // B - in Drive Command
    commandController.a().onTrue(manualIntake);
     commandController.pov(0).onTrue(shoot);
    commandController.pov(180).onTrue(activateAmp);
    commandController.back().onTrue(resetOdometry);
    commandController.leftBumper().onTrue(disableCommand);
    commandController.pov(270).onTrue(new GoToAMP1());
    commandController.pov(90).onTrue(shooter.activateShooterToSpeakerFromSub());
    
    overrideAuto.onTrue(chassis.getDefaultCommand());
}

  public void calibrate() {
    disableCommand.schedule();
  }

   
  public Command getAutonomousCommand() {
    return new StartTOP1();
  }
}
