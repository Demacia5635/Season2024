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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.AngleControl;
import frc.robot.commands.shooter.AngleGoToAngle;
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


public class RobotContainer implements Sendable{
  Boolean isRed = false;
  DriverStation.Alliance alliance;
  CommandXboxController commandController = new CommandXboxController(0);
  PS4Controller controller = new PS4Controller(1);  
  // Chassis chassis = new Chassis();

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

  Command intake2shooter;
  Command intake2amp;
  Command shoot;
  Command amplify;
  Command autonomousRight;



  public RobotContainer() {

    //chassis = new Chassis();

    // alliance = DriverStation.getAlliance().get();
    // isRed = (alliance == Alliance.Red);
    // DriveCommand drive = new DriveCommand(chassis, controller, commandController, isRed);
    // chassis.setDefaultCommand(drive);

    //SmartDashboard.putData("RC", this);
    // shooter = new Shooter();
    // amp = new Amp();
    intake = new Intake();
    

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
    // cmd still wasn't tested
    double wantedAngle = SmartDashboard.getNumber("wanted angle", 45);
    SmartDashboard.putData("go to angle", new AngleGoToAngle(shooter, wantedAngle, 1000, 500));

    // builder.addDoubleProperty("chassis angle",() -> chassis.getAngle().getDegrees(), null);
    // builder.addStringProperty("Alliance",() -> alliance.toString(), null);

  }
 private void configureBindings() {
      // if(controller.getCrossButton()) new InstantCommand(()->{chassis.setOdometryToForward();});
    // commandController.x().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
    
    }
   
  public Command getAutonomousCommand() {
    //return new PathFollow(chassis, points, 3, 6, DriverStation.getAlliance().get() == Alliance.Red);
   
    //return new IntakeCommand(intake).andThen(new AmpIntake2(amp));
    //.alongWith(new AmpIntake(amp, AmpConstants.CommandParams.v1, AmpConstants.CommandParams.v2))
   return intake2shooter;
    //return new AmpIntake(amp, AmpConstants.CommandParams.v1, AmpConstants.CommandParams.v2);
    //return new RunCommand(()->amp.neo1.set(0.5), amp);

  }
}