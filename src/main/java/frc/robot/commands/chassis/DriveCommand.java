package frc.robot.commands.chassis;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.leds.SubStrip;
import frc.robot.utils.Utils;

import static frc.robot.utils.Utils.*;

import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final CommandXboxController commandXboxController;

  private double direction;

  private boolean isRed;
  private boolean precisionDrive = false;

  private Translation2d speaker;
  // Rotation2d wantedAngleApriltag = new Rotation2d();
  // boolean rotateToApriltag = false;
  // PIDController rotationPidController = new PIDController(0.03, 0, 0.0008);

  public DriveCommand(Chassis chassis, CommandXboxController commandXboxController) {
    this.chassis = chassis;
    this.commandXboxController = commandXboxController;
    addRequirements(chassis);
    commandXboxController.rightBumper().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
    // commandXboxController.y().onTrue(new InstantCommand((() -> this.wantedAngleApriltag = chassis.getClosetAngleApriltag())).andThen(() -> rotateToApriltag = true));
  }

  @Override
  public void initialize() {
    isRed = chassis.isRed();
    speaker = (isRed) ? new Translation2d(16.54 - (-0.04), 5.5) : new Translation2d(-0.04, 5.55);
    direction = isRed ? 1 : -1;
  }

  @Override
  public void execute() {
    double joyX = deadband(commandXboxController.getLeftY(), 0.1) * direction;
    double joyY = deadband(commandXboxController.getLeftX(), 0.1) * direction;
    double rot = -(deadband(commandXboxController.getRightTriggerAxis(), 0.1)
        - deadband(commandXboxController.getLeftTriggerAxis(), 0.1));

    double velX = Math.pow(joyX, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyY);
    double velRot = Math.pow(rot, 2) * MAX_OMEGA_VELOCITY * Math.signum(rot);

    /*
    if (rotateToApriltag) {
      if (Math.abs(
          (wantedAngleApriltag).minus(chassis.getAngle()).getDegrees()) >= -1 &&
          Math.abs((wantedAngleApriltag).minus(chassis.getAngle()).getDegrees()) <= 1) {
        velRot = 0;
        rotateToApriltag = false;
      } else {
        velRot = rotationPidController.calculate(chassis.getAngle().getDegrees(), wantedAngleApriltag.getDegrees())
            * Math.toRadians(90);
      }
    } */
    if (rot == 0 && RobotContainer.robotContainer.shooter.isActiveForSpeaker()) { // rotate to speaker
      Translation2d vec = speaker.minus(chassis.getPose().getTranslation()); // vector from speaker to robot
      Rotation2d tgtAngle = vec.getAngle(); // angle we need to rotate to
      double angleError = Utils.angelErrorInRadians(tgtAngle, chassis.getAngle(), Math.toRadians(1));
      velRot = angleError * 0.3;
    }
    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }
    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    chassis.setVelocities(speeds);
  }

}
