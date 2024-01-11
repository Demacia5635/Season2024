package frc.robot.commands.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;

import static frc.robot.Constants.ChassisConstants.*;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final CommandXboxController controller;

  private boolean precisionDrive = false;

  public DriveCommand(Chassis chassis, CommandXboxController controller) {
    this.chassis = chassis;
    this.controller = controller;

    addRequirements(chassis);

    controller.pov(0).onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
  }

  @Override
  public void initialize() {
    chassis.stop();
  }

  @Override
  public void execute() {
    double joyX = -deadband(controller.getLeftX(), 0.1);
    double joyY = -deadband(controller.getLeftY(), 0.1);
    double rot = -(deadband(controller.getRightTriggerAxis(), 0.1) - deadband(controller.getLeftTriggerAxis(), 0.1));
    
    double velX = Math.pow(joyX, 3) * MAX_DRIVE_VELOCITY;
    double velY = Math.pow(joyY, 3) * MAX_DRIVE_VELOCITY;
    double velRot = Math.pow(rot, 3) * MAX_ANGULAR_VELOCITY;
    
    if (precisionDrive) {
      velX /= 2;
      velY /= 2;
      velRot /= 2;
    }

    ChassisSpeeds speeds = new ChassisSpeeds(velY, velX, Math.toRadians(velRot));
    chassis.setVelocities(speeds);
  }

  private double deadband(double x, double threshold) {
    if (Math.abs(x) < threshold) return 0;
    else return x;
  }
}
