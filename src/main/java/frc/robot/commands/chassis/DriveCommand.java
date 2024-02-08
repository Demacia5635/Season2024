package frc.robot.commands.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LedController;
import frc.robot.subsystems.chassis.Chassis;

import static frc.robot.Constants.ChassisConstants.*;
import static frc.robot.subsystems.chassis.ChassisConstants.MAX_OMEGA_VELOCITY;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final CommandXboxController controller;

  private boolean precisionDrive = false;

  private double[] llpython;

  private LedController Leds;
  public DriveCommand(Chassis chassis, CommandXboxController controller) {
    this.chassis = chassis;
    this.controller = controller;

    addRequirements(chassis);

    controller.b().onTrue(new InstantCommand(() -> precisionDrive = !precisionDrive));
    Leds = new LedController(9, 200);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double joyX = deadband(controller.getLeftY(), 0.1);
    double joyY = deadband(controller.getLeftX(), 0.1);
    double rot = -(deadband(controller.getRightTriggerAxis(), 0.1) - deadband(controller.getLeftTriggerAxis(), 0.1));
    
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

    llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
    if(llpython[0] != 0 ){
      Leds.changeColor(0, 255, 0);
    }
    else{
      Leds.changeColor(75, 0, 130);
    }
  }

  private double deadband(double x, double threshold) {
    return (Math.abs(x) < threshold)?0:x;
  }
}
