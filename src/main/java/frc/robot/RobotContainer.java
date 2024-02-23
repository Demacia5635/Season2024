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
  Boolean isRed = false;
  DriverStation.Alliance alliance;
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
  SubStrip leds;
  LedControll led;

  Command intake2shooter;
  Command intake2amp;
  Command shoot;
  Command amplify;
  Command autonomousRight;
  public DriveToNote driveToNote;

  Command amp2Angle;
  Command shootAmp;
  Command closeAmp;

  public double wantedAngle;
  public double wantedShootingVel;
  double wantedAmpVel;
  double wantedAmpAngle;

  double wantedAmpShootingAngle;
  double wantedAmpShootingVelUp;
  double wantedAmpShootingVelDown;

  pathPoint[] points = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), //doesnt matter because it gets fixed in the command
    new pathPoint(6.6, 6.7, Rotation2d.fromDegrees(0), 0, false)};
    
  /*
   * pathPoint[] points2 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0,
   * false),
   * new pathPoint(1.77, -0.6, Rotation2d.fromDegrees(180), 0, false)};
   */

  public RobotContainer() {
    robotContainer = this;
    
    chassis = new Chassis();
    vision = new Vision(chassis, chassis.getSwerveDrivePoseEstimator());
    intake = new Intake();
    // amp = new Amp();
    commandController2 = new CommandXboxController(1);

    // alliance = DriverStation.getAlliance().get();
    // isRed = (alliance == Alliance.Red) ;
    // DriveCommand drive = new DriveCommand(chassis, controller, commandController,
    // isRed);
    // chassis.setDefaultCommand(drive);

    amp = new Amp();
    commandController = new CommandXboxController(0);
    shooter = new Shooter();
    // shooter.setDefaultCommand(new AngleControl(shooter, commandController));
    chassis.setDefaultCommand(
        new DriveCommand(chassis, commandController, isRed));

    createCommands();

    // leds = new SubStrip(120);
    led = new LedControll(9, 110);
    
    SmartDashboard.putData("RC", this);
    configureBindings();
  }

  public void createCommands() {
    // intake2amp = (new DispenseCommand(intake).raceWith(new AmpIntake2(amp)));
    // amp2Angle = amp.getReadyCommand(intake);
    // shootAmp = amp.getShootCommand();
    // closeAmp = amp.getCancelCommand();
    driveToNote = new DriveToNote(chassis);

    // for check, can add wait command between and then

    // intake2shooter = (new AngleGoToAngle(shooter,
    // ShooterConstants.CommandParams.ANGLE_COLLECT,
    // ShooterConstants.CommandParams.MAX_VEL,
    // ShooterConstants.CommandParams.MAX_ACCCEL))
    // .andThen(new IntakeCommand(intake).raceWith(new GoToNote(chassis)), new
    // ShootCommand(intake).alongWith(new ShooterFeeding(shooter,
    // ShooterConstants.CommandParams.FEED_POWER)),
    // new AngleGoToAngle(shooter, ShooterConstants.CommandParams.ANGLE_DEFAULT,
    // ShooterConstants.CommandParams.MAX_VEL,
    // ShooterConstants.CommandParams.MAX_ACCCEL));

    // intake2amp = (new IntakeCommand(intake).raceWith(new GoToNote(chassis)))
    // .andThen(new DispenseCommand(intake)
    // .alongWith(new AmpIntake(amp, AmpConstants.CommandParams.v1,
    // AmpConstants.CommandParams.v2)));

    // amplify = new GoToAngleAmp(amp, AmpConstants.CommandParams.ANGLE_RADIANS_AMP,
    // AmpConstants.CommandParams.MAX_VEL_RAD,
    // AmpConstants.CommandParams.MAX_ACCEL_RAD).andThen(
    // new AmpIntake(amp, AmpConstants.CommandParams.v1,
    // AmpConstants.CommandParams.v2)
    // );

    // //I prefer driver will close the amp
    // amplify = new OpenAmp(amp).andThen(new AmpIntake2(amp));

    // shoot = new AngleGoToAngle(shooter,
    // ShooterConstants.CommandParams.ANGLE_SHOOT,
    // ShooterConstants.CommandParams.MAX_VEL,
    // ShooterConstants.CommandParams.MAX_ACCCEL).alongWith(
    // new RunCommand(() ->
    // shooter.setPow(ShooterConstants.CommandParams.SHOOT_POWER), shooter)
    // ).andThen(new ShooterFeeding(shooter,
    // ShooterConstants.CommandParams.FEED_POWER));

    // Command intake2ampnoshooter = new IntakeCommand(intake).andThen(new
    // WaitCommand(1.5), new DispenseCommand(intake).raceWith(
    // new AmpIntake2(amp)
    // ));

  }

  @Override
  public void initSendable(SendableBuilder builder) {

    // builder.addDoubleProperty("chassis angle",() ->
    // chassis.getAngle().getDegrees(), null);
    // builder.addStringProperty("Alliance",() -> alliance.toString(), null);

  }

  private void configureBindings() {

    Trigger overrideAuto = new Trigger(() -> Utils.joystickOutOfDeadband(commandController));

    // wanted angle = 56 angle for close to speaker
    wantedAngle = 56;
    // SmartDashboard.putNumber("wanted angle", 60);
    // SmartDashboard.putNumber("wanted shooting vel for noga", 0);
    // wantedAngle = SmartDashboard.getNumber("wanted angle", 60);
    wantedShootingVel = 16.5;

    wantedAmpShootingAngle = ShooterConstants.AmpPera.ANGLE;
    wantedAmpShootingVelUp = ShooterConstants.AmpPera.UP;
    wantedAmpShootingVelDown = ShooterConstants.AmpPera.DOWN;
    

    // wantedShootingVel = SmartDashboard.getNumber("wanted shooting vel for noga",
    // 0);
    // commandController.b().onTrue(new AngleQuel(shooter));
    commandController.a().onTrue(new IntakeCommand(intake));
    //commandController.a().onTrue(new RunCommand(()->intake.setPower(1), intake));
    commandController.pov(180).onTrue(new AngleGoToAngle(shooter, wantedAmpShootingAngle).alongWith(new ShooterPowering(shooter, wantedAmpShootingVelUp, wantedAmpShootingVelDown)));
    commandController.leftBumper().whileTrue(new IntakeToShooter(intake, shooter, wantedAmpShootingVelUp, wantedAmpShootingVelDown));
    commandController.pov(0).whileTrue(new IntakeToShooter(intake, shooter, wantedShootingVel));
    commandController.x()
        .onTrue(new AngleGoToAngle(shooter, wantedAngle).alongWith(new ShooterPowering(shooter, wantedShootingVel)));

    commandController.rightBumper().onTrue(new InstantCommand(() -> {
      shooter.stopAll();
      intake.stop();
    }, intake, shooter).andThen(new AngleQuel(shooter)));
    commandController.y().onTrue(driveToNote.raceWith(new IntakeCommand(intake)));
    // commandController.pov(180).onTrue(new InstantCommand(() -> {
    //   chassis.setOdometryToForward();
    // }));
    overrideAuto.onTrue(chassis.getDefaultCommand());

    // Amp commands Buttons
    /*
     * commandController.b().onTrue(intake2amp.andThen(amp2Angle));
     * commandController.pov(90).onTrue(shootAmp);
     * commandController.pov(270).onTrue(closeAmp);
     */

  }

  public void calibrate() {
    new AngleQuel(shooter).schedule();
    // (new CalibrateArm(amp).andThen(new RunBrakeArm(amp,
    // Parameters.ARM_BRAKE_POW))).schedule();
  }

  public void resetOd() {
    new InstantCommand(() -> chassis.setPose(new Pose2d(2.896, 8.54 - 1.21, Rotation2d.fromDegrees(0)))).schedule();
    ;
  }

    public void disable(){
        new InstantCommand(()-> {
            shooter.stopAll();
            intake.stop();
        }, 
        intake, shooter
        ).ignoringDisable(true).schedule();
    }
   
   
  public Command getAutonomousCommand() {
    return new StartTOP(chassis, shooter, intake, isRed);
   //return new PathFollow(chassis, points, 4, 12, 0, false);
  }
}
