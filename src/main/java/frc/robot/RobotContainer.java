package frc.robot;

import javax.swing.plaf.metal.MetalTheme;


import java.util.Optional;

import com.fasterxml.jackson.core.StreamReadConstraints.Builder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.AngleControl;
import frc.robot.commands.shooter.AngleGoToAngle;
import frc.robot.commands.shooter.AngleGoToDis;
import frc.robot.commands.shooter.AngleQuel;
import frc.robot.commands.shooter.ShooterFeeding;
import frc.robot.commands.shooter.ShooterPowering;
import frc.robot.commands.shooter.ShooterSending;
import frc.robot.commands.shooter.ShooterShoot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.amp.AmpIntake;
import frc.robot.commands.amp.AmpIntake2;
import frc.robot.commands.amp.GoToAngleAmp;
import frc.robot.commands.amp.JoyStickAmp;
import frc.robot.commands.amp.GoToAngleAmp;
import frc.robot.commands.chassis.DriveAndPickNote;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.SetModuleAngle;
import frc.robot.commands.chassis.TestSteerVandA;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.DispenseCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeToShooter;
import frc.robot.commands.intake.ShootCommand;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LedControll;
import frc.robot.subsystems.led.SubStrip;


public class RobotContainer implements Sendable{
  Boolean isRed = false;
  DriverStation.Alliance alliance;
  CommandXboxController commandController;
  PS4Controller controller = new PS4Controller(1);  

  // pathPoint[] points = {
  //   new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
  //   new pathPoint(2, 3, Rotation2d.fromDegrees(0), 0, false),
  // };
  // Command test = new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.5, 0, 0));}, chassis).andThen(new WaitCommand(2),
  // new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.4  , 0, 0));}, chassis).andThen(new WaitCommand(2)),
  // new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(0, 0, 0));}, chassis).andThen(new WaitCommand(2)));
  // double x = 5;

  Shooter shooter;
  Amp amp;
  Intake intake;
  Chassis chassis;
  LedControll ledControll;
  SubStrip leds;

  Command intake2shooter;
  Command intake2amp;
  Command shoot;
  Command amplify;
  Command autonomousRight;

  double wantedAngle;
  double wantedShootingVel;
  double wantedAmpVel;
  double wantedAmpAngle;

  pathPoint[] points = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
    new pathPoint(1.35, -0.6, Rotation2d.fromDegrees(180), 0, false)
  };
  /*pathPoint[] points2 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
  new pathPoint(1.77, -0.6, Rotation2d.fromDegrees(180), 0, false)};*/



  public RobotContainer() {

    intake = new Intake();

    // ledControll = new LedControll(0, 5);
    leds = new SubStrip(60);
    // alliance = DriverStation.getAlliance().get();
    // isRed = (alliance == Alliance.Red)   ;
    // DriveCommand drive = new DriveCommand(chassis, controller, commandController, isRed);
    // chassis.setDefaultCommand(drive);

    amp = new Amp();
    commandController = new CommandXboxController(0);
    shooter = new Shooter();
    //shooter.setDefaultCommand(new AngleControl(shooter, commandController));
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController, DriverStation.getAlliance().get() == Alliance.Red));
    

    SmartDashboard.putData("RC", this);
    configureBindings();
  }

  public void createCommands() {



    //for check, can add wait command between and then

    // intake2shooter = (new AngleGoToAngle(shooter, ShooterConstants.CommandParams.ANGLE_COLLECT,
    //  ShooterConstants.CommandParams.MAX_VEL, ShooterConstants.CommandParams.MAX_ACCCEL))
    // .andThen(new IntakeCommand(intake).raceWith(new GoToNote(chassis)), new ShootCommand(intake).alongWith(new ShooterFeeding(shooter, ShooterConstants.CommandParams.FEED_POWER)), 
    //  new AngleGoToAngle(shooter, ShooterConstants.CommandParams.ANGLE_DEFAULT,
    //  ShooterConstants.CommandParams.MAX_VEL, ShooterConstants.CommandParams.MAX_ACCCEL));

    // intake2amp = (new IntakeCommand(intake).raceWith(new GoToNote(chassis)))
    // .andThen(new DispenseCommand(intake)
    // .alongWith(new AmpIntake(amp, AmpConstants.CommandParams.v1, AmpConstants.CommandParams.v2)));

    //  amplify = new GoToAngleAmp(amp, AmpConstants.CommandParams.ANGLE_RADIANS_AMP,
    //  AmpConstants.CommandParams.MAX_VEL_RAD, AmpConstants.CommandParams.MAX_ACCEL_RAD).andThen(
    //   new AmpIntake(amp, AmpConstants.CommandParams.v1, AmpConstants.CommandParams.v2)
    //  );

    //  //I prefer driver will close the amp
    //  amplify = new OpenAmp(amp).andThen(new AmpIntake2(amp));

    //  shoot = new AngleGoToAngle(shooter, ShooterConstants.CommandParams.ANGLE_SHOOT,
    //  ShooterConstants.CommandParams.MAX_VEL, ShooterConstants.CommandParams.MAX_ACCCEL).alongWith(
    //   new RunCommand(() -> shooter.setPow(ShooterConstants.CommandParams.SHOOT_POWER), shooter)
    //  ).andThen(new ShooterFeeding(shooter, ShooterConstants.CommandParams.FEED_POWER));


    // Command intake2ampnoshooter =  new IntakeCommand(intake).andThen(new WaitCommand(1.5), new DispenseCommand(intake).raceWith(
    //   new AmpIntake2(amp)
    // ));



  }


  @Override
  public void initSendable(SendableBuilder builder) {

    // SmartDashboard.putData("Red", new InstantCommand(()-> ledControll.setColor(255, 0, 0), ledControll).ignoringDisable(true));
    // SmartDashboard.putData("Green", new InstantCommand(()-> ledControll.setColor(0, 255, 0), ledControll).ignoringDisable(true));
    // SmartDashboard.putData("Blue", new InstantCommand(()-> ledControll.setColor(0, 0, 255), ledControll).ignoringDisable(true));

    // builder.addDoubleProperty("chassis angle",() -> chassis.getAngle().getDegrees(), null);
    // builder.addStringProperty("Alliance",() -> alliance.toString(), null);

  }
    private void configureBindings() {

      //wanted angle = 54 angle for close to speaker
        wantedAngle = 36;
        wantedAmpAngle = 110/360*2*Math.PI;
    //   SmartDashboard.putNumber("wanted angle", 60);
    //   SmartDashboard.putNumber("wanted shooting vel for noga", 0);
    //   wantedAngle = SmartDashboard.getNumber("wanted angle", 60);
        wantedShootingVel = 12;
        wantedAmpVel = Math.PI/2;
    //   wantedShootingVel = SmartDashboard.getNumber("wanted shooting vel for noga", 0);
    //   commandController.b().onTrue(new AngleQuel(shooter));
        commandController.a().onTrue(new IntakeCommand(intake));
        
        commandController.pov(0).onTrue(new AngleGoToAngle(shooter, wantedAngle).alongWith( new ShooterPowering(shooter, wantedShootingVel)));
        commandController.x().whileTrue(new IntakeToShooter(intake, shooter, wantedShootingVel));
        commandController.rightBumper().onTrue(new InstantCommand(()-> {shooter.stopAll();intake.stop();}, intake, shooter).ignoringDisable(true));
        commandController.y().onTrue(new DriveToNote(chassis).raceWith(new IntakeCommand(intake)).alongWith(new InstantCommand(()-> leds.setBlink(Color.kOrange))));
        //commandController.b().whileTrue(new AmpIntake2(amp));
        //commandController.rightTrigger().onTrue(new GoToAngleAmp(amp, wantedAngle, wantedAmpVel, wantedAmpAngle));
        // if(controller.getCrossButton()) new InstantCommand(()->{chassis.setOdometryToForward();});
    // commandController.x().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
    
    }


    public void calibrate() {
        new AngleQuel(shooter).schedule();
    }

    public void disable(){
        new InstantCommand(()-> {
            shooter.stopAll();
            intake.stop();
        }, 
        intake, shooter
        ).ignoringDisable(true).schedule();
        leds.turnOff().schedule();
    }
   
  public Command getAutonomousCommand() {
    //return new PathFollow(chassis, points, 3, 6, DriverStation.getAlliance().get() == Alliance.Red);
   
    return new InstantCommand(() -> ledControll.setColor(55, 55, 0));
    /*.andThen(new PathFollow(chassis, points2, 1, 2, 0, DriverStation.getAlliance().get() == Alliance.Red)
    .alongWith(new IntakeCommand(intake)));*/
    //.alongWith(new AmpIntake(amp, AmpConstants.CommandParams.v1, AmpConstants.CommandParams.v2))

    //return new RunCommand(()-> chassis.setVelocities(new ChassisSpeeds(0, -1.5, 0)), chassis);

    // return new PathFollow(chassis, points1, 2, 4, 1, true).andThen
    // (new PathFollow(chassis, points2, 2, 4, 0, true));
    //return new RunCommand(()->amp.neo1.set(0.5), amp);

  }
}