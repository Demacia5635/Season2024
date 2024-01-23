package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveCommand;
import frc.robot.commands.chassis.PathFollow;
import frc.robot.commands.chassis.SetModuleAngle;
import frc.robot.commands.chassis.TestSteerVandA;
import frc.robot.subsystems.chassis.Chassis;


public class RobotContainer implements Sendable{
  CommandXboxController commandController = new CommandXboxController(0);
  Chassis chassis = new Chassis();
  pathPoint[] points = {
    new pathPoint(0, 0, new Rotation2d(0), 0.2, false),
    
    new pathPoint(1, 1, new Rotation2d(0), 0.35, false),
    
    new pathPoint(0, 2, new Rotation2d(0), 0.2, false),
    
    new pathPoint(1, 3, Rotation2d.fromDegrees(0), 0.2, false)
  };
  PathFollow path = new PathFollow(chassis, points, 2, 4);
  DriveCommand drive = new DriveCommand(chassis, commandController);
  Command test = new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.5, 0, 0));}, chassis).andThen(new WaitCommand(2),
  new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(-0.4  , 0, 0));}, chassis).andThen(new WaitCommand(2)),
  new RunCommand(() -> {chassis.setVelocities(new ChassisSpeeds(0, 0, 0));}, chassis).andThen(new WaitCommand(2)));
  double x = 0.2;

 
  public RobotContainer() { 
    chassis.setDefaultCommand(drive);

    SmartDashboard.putData("RC", this);

    configureBindings();
  }


  @Override
  public void initSendable(SendableBuilder builder) {

    builder.addDoubleProperty("spinspeed", () -> Math.toDegrees(chassis.getChassisSpeeds().omegaRadiansPerSecond), null);

  }

    /**
     * 
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
      commandController.a().onTrue(new InstantCommand(()->{chassis.setOdometryToForward();}));
        // code for controller to controll the gripper and the parallelogram

        // safty buttons to stop the arm and/or the gripper
    }
   
  /**
   * Use this to pass the autonomous command to the main {@link Robot} cass.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return null;
    return path;
    // return new InstantCommand(() -> chassis.resetWheels(), chassis)
    // .andThen(new RunCommand(() -> chassis.setVelocities(new ChassisSpeeds(-2, 0, 0))).withTimeout(2).andThen(new InstantCommand(() -> chassis.stop())));
    //return new RunCommand(() -> chassis.getModule(2).setAngularVelocity(600));
    // return new RunCommand(() -> chassis.setModulesPower(0.05));
    // return new RunCommand(()-> chassis.setModulesAngularPower(-0.3), chassis);
    //return new RunCommand(()->{chassis.getModule(2).setAngularPower(0.049 + 300*0.0003);},chassis).withTimeout(3)
    //  .andThen(new InstantCommand(()->{SmartDashboard.putNumber("FF TEST",  chassis.getModule(2).getAngularVelocity());
    //chassis.stop();}));
  }
}
