package frc.robot;

import static frc.robot.utils.Utils.angelErrorInRadians;
import static frc.robot.utils.Utils.speakerPosition;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.StringIdGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
import frc.robot.commands.chassis.Auto.StartTOP1;
import frc.robot.commands.chassis.Auto.StartTOP2;
import frc.robot.commands.chassis.Auto.AutoChooser;
import frc.robot.commands.chassis.Auto.Fowrard;
import frc.robot.commands.chassis.Auto.MidPlayoffTEST;
import frc.robot.commands.chassis.Auto.Shoot;
import frc.robot.commands.chassis.Auto.StartBottom1;
import frc.robot.commands.chassis.Auto.StartBottom2;
import frc.robot.commands.chassis.Auto.StartBottomEscape;
import frc.robot.commands.chassis.Auto.StartBottomPlayoffs;
import frc.robot.commands.chassis.Auto.StartMiddle1;
import frc.robot.commands.chassis.Auto.StartMiddle2;
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
  public CommandXboxController commandController2;
 
  public Shooter shooter;
  public Amp amp;
  public Intake intake;
  public Chassis chassis;
  public VisionLimelight vision;
  public LedControll led;
  public Command shoot; // shoot to amp or to speaker
  public Command 
  activateShooter; // prepare to shoot
  public Command autonomousRight;
  public Command driveToNote;
  public Command manualIntake;
  public Command activateAmp;
  public Command disableCommand;
  public Command resetOdometry;
  public Command activatePodium;



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
    SmartDashboard.putNumber("UP", 0);
    SmartDashboard.putNumber("DOWN", 0);
    SmartDashboard.putNumber("ANGLE SHOOTER", 0);



    driveToNote = new DriveToNote(chassis, 1.6, true).raceWith(new IntakeCommand(intake));
    driveToNote.setName("Drive to Note");
    shoot = shooter.getShootCommand();
    shoot.setName("Shoot");
    activateShooter = shooter.getActivateShooterToSpeaker();
    activateShooter.setName("activateShooter");
    manualIntake = new IntakeCommand(intake);
    manualIntake.setName("manualIntake");
    activateAmp = shooter.getActivateShooterToAmp();
    activateAmp.setName("Activate Amp");
    activatePodium = shooter.getActivateShooterToPodium();
    activatePodium.setName("activate podium");

    resetOdometry = new InstantCommand(()-> chassis.setOdometryToForward()).ignoringDisable(true);
    resetOdometry.setName("reset odometry");
    disableCommand = new InstantCommand(()-> stopAll(),intake, shooter).andThen(new AngleCalibrate(shooter));
    disableCommand.setName("disable command");

    Command startTop = (new AngleCalibrate(shooter))
      .andThen((new ActivateShooter(shooter, intake, chassis, true))
      .alongWith(new StartTOP1()));
    startTop.setName("TOP");
    Command StartMiddle = new ActivateShooter(shooter, intake, chassis, true).alongWith(new StartMiddle1());
    StartMiddle.setName("Middle");
    Command startBottom = new ActivateShooter(shooter, intake, chassis, true).alongWith(new StartBottom1());
    startBottom.setName("Buttom");
    Command autoShoot = (new AngleCalibrate(shooter))
      .andThen(new ActivateShooter(shooter, intake, chassis, true)
      .alongWith((new WaitUntilCommand(()->shooter.getIsShootingReady())).andThen(shooter.getShootCommand())));
      autoShoot.setName("auto shoot");
    Command justShootFromSubWuffer = (new AngleCalibrate(shooter))
      .andThen(shooter.getActivateShooterToSpeakerFromSub()
      .alongWith((new WaitUntilCommand(()->shooter.getIsShootingReady())).andThen(shooter.getShootCommand())));

    Command shootFromSubAndLeave = (new AngleCalibrate(shooter)
      .andThen(shooter.getActivateShooterToSpeakerFromSub()
      .alongWith((new WaitUntilCommand(()->shooter.getIsShootingReady())).andThen(shooter.getShootCommand())))).raceWith(new WaitCommand(5)).andThen(
      new RunCommand(
        () -> chassis.setVelocities(new ChassisSpeeds((isRed) ? -1 : 1,
         0, 0))).raceWith(new WaitCommand(5)));
  
    Command StartBottomEscape = new ActivateShooter(shooter, intake, chassis, true).alongWith(new StartBottomEscape());
    StartBottomEscape.setName("start b escape");
    // Command PlayOffs = (new AngleCalibrate(shooter)
    //   .andThen(shooter.getActivateShooterToSpeakerFromSub()
    //   .alongWith((new WaitUntilCommand(()->shooter.getIsShootingReady())).andThen(shooter.getShootCommand()))))
    //   .andThen(new ActivateShooter(shooter, intake, chassis, true).alongWith(new StartBottomPlayoffs()));
    // PlayOffs.setName("playpoff");

    Command PlayOffs = new ActivateShooter(shooter, intake, chassis, true).alongWith(new StartBottomPlayoffs());
    Command tryForDis3TOP = (new AngleCalibrate(shooter)
      .andThen(shooter.getActivateShooterToSpeakerFromSub()
      .alongWith((new WaitUntilCommand(()->shooter.getIsShootingReady())).andThen(shooter.getShootCommand())))).andThen(new StartTOP2().alongWith(new ActivateShooter(shooter, intake, chassis, true)));
    Command tryForDis3MIDDLE = (new AngleCalibrate(shooter)
      .andThen(shooter.getActivateShooterToSpeakerFromSub()
      .alongWith((new WaitUntilCommand(()->shooter.getIsShootingReady())).andThen(shooter.getShootCommand())))).andThen(new StartMiddle2().alongWith(new ActivateShooter(shooter, intake, chassis, true)));
    Command tryForDis3BOTTOM = (new AngleCalibrate(shooter)
      .andThen(shooter.getActivateShooterToSpeakerFromSub()
      .alongWith((new WaitUntilCommand(()->shooter.getIsShootingReady())).andThen(shooter.getShootCommand())))).andThen(new StartBottom2().alongWith(new ActivateShooter(shooter, intake, chassis, true)));


    autoChoose = new SendableChooser<Command>();
    autoChoose.setDefaultOption("Playoffs", PlayOffs);
    autoChoose.addOption("Top",startTop);
    autoChoose.addOption("Middle", StartMiddle);
    autoChoose.addOption("Bottom", startBottom);
    autoChoose.addOption("shoot", autoShoot);
    autoChoose.addOption("StartBottomEscape", StartBottomEscape);
    autoChoose.addOption("just shoot from subwuffer", justShootFromSubWuffer);
    autoChoose.addOption("Middle try", tryForDis3MIDDLE);
    autoChoose.addOption("Top try", tryForDis3TOP);
    autoChoose.addOption("Bottom try", tryForDis3BOTTOM);
    autoChoose.addOption("shot + leave", shootFromSubAndLeave);






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

    




    Trigger overrideAuto = new Trigger(() -> Utils.joystickOutOfDeadband(commandController));

    commandController.y().onTrue(driveToNote);
    commandController.x().onTrue(activateShooter);
    // B - in Drive Command
    commandController.a().onTrue(manualIntake);
    commandController.pov(0).onTrue(shoot);
    commandController.start().onTrue(activateAmp);
    commandController.back().onTrue(resetOdometry);
    commandController.leftBumper().onTrue(disableCommand);
   // commandController.pov(270).onTrue(new GoToAMP1());
    commandController.rightBumper().onTrue(shooter.getActivateShooterToSpeakerFromSub());
    overrideAuto.onTrue(chassis.getDefaultCommand());

    //Operator Controller
    //b -> set odometry to vision
    //y -> Shooter from Subwofer
    //x -> Shooter from Look up table
    //a -> amp
    //rightBumper -> Shooter from Podium
    //leftBumper -> Take note bakewords
    //pov(down) -> puke from intake(remove stack note)
    //pov(up) -> puke from Shooter(remove stack note)
    //pov(right) -> exposer+1
    //pov(left) -> exposer-1
    Command driverB = new InstantCommand((()->vision.setResetOdo(true))).ignoringDisable(true);
    driverB.setName("Perlman B");
    commandController2.b().onTrue(driverB);
    commandController2.y().onTrue(shooter.getActivateShooterToSpeakerFromSub());
    commandController2.x().onTrue(activateShooter);
    commandController2.a().onTrue(activateAmp);
    commandController2.rightBumper().onTrue(activatePodium);
    Command PerlmanleftBumper = (new RunCommand(()->shooter.feedingSetPow(-0.5), intake).withTimeout(0.2))
      .andThen(new InstantCommand(()->shooter.feedingSetPow(0), intake));
    PerlmanleftBumper.setName("Perlman l Bumber");
    commandController2.leftBumper().onTrue(PerlmanleftBumper);
    Command Ppov0 = new InstantCommand(()-> {intake.setPower(1); shooter.feedingSetPow(1); shooter.setVel(10);}, shooter, intake);
    Ppov0.setName("P Pov0");
    commandController2.pov(0).whileTrue(Ppov0);
    Command Ppov180 = new RunCommand(()-> intake.setPower(-1), intake).withTimeout(0.3);
    Ppov180.setName("P Pov 180");
    commandController2.pov(180).onTrue(Ppov180);

    Command Ppov90 = new InstantCommand(() -> {Utils.setPipeline(Math.min(Utils.getPipeline() + 1, 6)); }).ignoringDisable(true);
    Ppov180.setName("P Pov 90");
    commandController2.pov(90).onTrue(Ppov90);

  
    commandController2.pov(270).onTrue(new InstantCommand(() -> {Utils.setPipeline(Math.max(Utils.getPipeline() - 1, 0)); }).ignoringDisable(true));

}


  public void calibrate() {
    disableCommand.schedule();
  }

 
   
  public Command getAutonomousCommand() {
    Command cmd = autoChoose.getSelected();
    System.out.println(" -------------------------------------------");
    System.out.println(" Auto command = " + cmd);
    System.out.println(" -------------------------------------------");
    System.out.println(" -------------------------------------------");
    System.out.println(" -------------------------------------------");
    return cmd;
    //return new StartTOP1().alongWith(new ActivateShooter(shooter, intake, chassis, true));
    //return new RunCommand(()->shooter.setVel(10), shooter);
  }
}
