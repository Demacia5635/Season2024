package frc.robot.commands.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.chassis.Chassis;

import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final PS4Controller controller;
  private final CommandXboxController commandXboxController;

  private double direction;

  private boolean precisionDrive = false;

  public DriveCommand(Chassis chassis, PS4Controller controller, CommandXboxController commandXboxController, boolean isRed) {
    this.chassis = chassis;
    this.commandXboxController = commandXboxController;
    this.controller = controller;
    if(isRed) direction = -1;
    else direction = 1;

    addRequirements(chassis);

    commandXboxController.b().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double joyX = deadband(commandXboxController.getLeftY(), 0.1) * direction;
    double joyY = deadband(commandXboxController.getLeftX(), 0.1) * direction;
    double rot = -(deadband(commandXboxController.getRightTriggerAxis(), 0.1) - deadband(commandXboxController.getLeftTriggerAxis(), 0.1));
    
    double velX = Math.pow(joyX, 3)* MAX_DRIVE_VELOCITY;
    double velY = Math.pow(joyY, 3) * MAX_DRIVE_VELOCITY;
    double velRot = Math.pow(rot, 3) * MAX_OMEGA_VELOCITY;
    
    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }

    System.out.println("target velocity= " + velRot);


    ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
    chassis.setVelocities(speeds);
  }

  private double deadband(double x, double threshold) {
    return (Math.abs(x) < threshold)?0:x;
  }
}
