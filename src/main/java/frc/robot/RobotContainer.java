package frc.robot;

import javax.swing.plaf.metal.MetalTheme;


import java.util.Optional;

import com.fasterxml.jackson.core.StreamReadConstraints.Builder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.Utils;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.amp.AmpIntake;
import frc.robot.commands.amp.AmpIntake2;
import frc.robot.commands.amp.AmpIntakeShoot;
import frc.robot.commands.amp.CalibrateArm;
import frc.robot.commands.amp.GoToAngleAmp;
import frc.robot.commands.amp.JoyStickAmp;
import frc.robot.commands.amp.RunBrakeArm;
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
import frc.robot.subsystems.amp.AmpConstants.Parameters;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.SubStrip;


public class RobotContainer implements Sendable{
  Boolean isRed = false;
  DriverStation.Alliance alliance;
  CommandXboxController commandController;
  CommandXboxController commandController2;
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
  Vision vision;
  SubStrip leds;

  Command intake2shooter;
  Command intake2amp;
  Command shoot;
  Command amplify;
  Command autonomousRight;

  Command amp2Angle;
  Command shootAmp;
  Command closeAmp;

  double wantedAngle;
  double wantedShootingVel;
  double wantedAmpVel;
  double wantedAmpAngle;

  

  pathPoint[] points = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
    new pathPoint(1.35, -0.6, Rotation2d.fromDegrees(0), 0, false)
  };
  /*pathPoint[] points2 = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
  new pathPoint(1.77, -0.6, Rotation2d.fromDegrees(0), 0, false)};*/



  public RobotContainer() {
    
    chassis = new Chassis();
    vision = new Vision(chassis, chassis.getSwerveDrivePoseEstimator());
    intake = new Intake();
    //leds = new SubStrip(60);
    commandController2 = new CommandXboxController(1);

    // alliance = DriverStation.getAlliance().get();
    // isRed = (alliance == Alliance.Red)   ;
    // DriveCommand drive = new DriveCommand(chassis, controller, commandController, isRed);
    // chassis.setDefaultCommand(drive);

    //amp = new Amp();
    commandController = new CommandXboxController(0);
    shooter = new Shooter();
    //shooter.setDefaultCommand(new AngleControl(shooter, commandController));
    chassis.setDefaultCommand(new DriveCommand(chassis, commandController, DriverStation.getAlliance().get() == Alliance.Red));
    
    createCommands();
    
    SmartDashboard.putData("RC", this);
    configureBindings();
  }

  public void createCommands() {
     //intake2amp = (new DispenseCommand(intake).alongWith(new AmpIntake2(amp)));
    
    //amp2Angle = (new RunBrakeArm(amp,Parameters.ARM_RELEASE_POW).andThen(new GoToAngleAmp(amp, Math.toRadians(55), wantedAmpVel,Math.PI*2)).andThen(new RunBrakeArm(amp, Parameters.ARM_BRAKE_POW)));

    //shootAmp = (new AmpIntakeShoot(amp));

    //closeAmp = (new RunBrakeArm(amp,Parameters.ARM_RELEASE_POW).andThen((new GoToAngleAmp(amp, Math.toRadians(-55), wantedAmpVel/2,Math.PI*2)).andThen(new RunBrakeArm(amp, Parameters.ARM_BRAKE_POW))));


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

    // builder.addDoubleProperty("chassis angle",() -> chassis.getAngle().getDegrees(), null);
    // builder.addStringProperty("Alliance",() -> alliance.toString(), null);

  }
    private void configureBindings() {

      Trigger overrideAuto = new Trigger(()->Utils.joystickOutOfDeadband(commandController));

      //wanted angle = 56 angle for close to speaker
        wantedAngle = 56
        ;
        wantedAmpAngle = 110/360*2*Math.PI;
    //   SmartDashboard.putNumber("wanted angle", 60);
    //   SmartDashboard.putNumber("wanted shooting vel for noga", 0);
    //   wantedAngle = SmartDashboard.getNumber("wanted angle", 60);
        wantedShootingVel = 14;
        wantedAmpVel = Math.PI/2;
    //   wantedShootingVel = SmartDashboard.getNumber("wanted shooting vel for noga", 0);
    //   commandController.b().onTrue(new AngleQuel(shooter));
        commandController.a().onTrue(new IntakeCommand(intake));
        
        commandController.pov(0).whileTrue(new IntakeToShooter(intake, shooter, wantedShootingVel));
        commandController.x().onTrue(new AngleGoToAngle(shooter, wantedAngle).alongWith( new ShooterPowering(shooter, wantedShootingVel)));
  
        commandController.rightBumper().onTrue(new InstantCommand(()-> {shooter.stopAll();intake.stop();}, intake, shooter).andThen(new AngleQuel(shooter)));
        commandController.y().onTrue(new DriveToNote(chassis).raceWith(new IntakeCommand(intake)));
        //commandController.b().whileTrue(new AmpIntake2(amp));
        //commandController.rightTrigger().onTrue(new GoToAngleAmp(amp, wantedAngle, wantedAmpVel, wantedAmpAngle));
        // if(controller.getCrossButton()) new InstantCommand(()->{chassis.setOdometryToForward();});
        commandController.pov(0).onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
        overrideAuto.onTrue(chassis.getDefaultCommand());
        
        //Amp commands Buttons
       /* commandController2.pov(90).onTrue(intake2amp);
        commandController2.pov(270).onTrue(amp2Angle);
        commandController2.leftBumper().onTrue(shootAmp);
        commandController2.a().onTrue(closeAmp); */

    
    }


   
    public void calibrate() {
        new AngleQuel(shooter).schedule();
       // (new CalibrateArm(amp).andThen(new RunBrakeArm(amp, Parameters.ARM_BRAKE_POW))).schedule();
    }

    public void disable(){
        new InstantCommand(()-> {
            shooter.stopAll();
            intake.stop();
        }, 
        intake, shooter
        ).ignoringDisable(true).schedule();
        //leds.turnOff().schedule();
    }
   
   
  public Command getAutonomousCommand() {
  
    Pose2d pose = chassis.getPose();
    pathPoint[] pointArrForX = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false),
       new pathPoint(pose.getX() + 2, pose.getY(), Rotation2d.fromDegrees(0), 0, false), 
       new pathPoint(pose.getX() - 2, pose.getY(), Rotation2d.fromDegrees(0), 0, false),
       new pathPoint(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0), 0, false)};

    pathPoint[] pointArrForY = {new pathPoint(0, 0, Rotation2d.fromDegrees(0), 0, false), 
      new pathPoint(pose.getX(), pose.getY() + 2, Rotation2d.fromDegrees(0), 0, false), 
      new pathPoint(pose.getX(), pose.getY() - 2, Rotation2d.fromDegrees(0), 0, false),
       new pathPoint(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0), 0, false)};

    // return new PathFollow(chassis,pointArrForX , 1, 1, 0, false).andThen(new PathFollow(chassis,pointArrForX , 2, 1.5, 0, false).
    // andThen(new PathFollow(chassis,pointArrForX , 2.5, 1.75, 0, false).andThen(new PathFollow(chassis,pointArrForX , 3, 2, 0, false).
    // andThen(new PathFollow(chassis,pointArrForX , 3.5, 2, 0, false).andThen(new PathFollow(chassis,pointArrForX , 4, 2.5, 0, false))))));

    return new PathFollow(chassis, pointArrForY, 0.5, 0.5, 0, false).andThen(
      new PathFollow(chassis, pointArrForY, 2, 2, 0, false)).andThen(
        new PathFollow(chassis, pointArrForY, 3, 6, 0, false)
    );

    /*return new PathFollow(chassis,pointArrForY , 1, 1, 0, false).andThen(new PathFollow(chassis,pointArrForY , 2, 1.5, 0, false).
    andThen(new PathFollow(chassis,pointArrForY , 2.5, 1.75, 0, false).andThen(new PathFollow(chassis,pointArrForY , 3, 2, 0, false).
    andThen(new PathFollow(chassis,pointArrForY , 3.5, 2, 0, false).andThen(new PathFollow(chassis,pointArrForY , 4, 2.5, 0, false)))))); */
  }
}
