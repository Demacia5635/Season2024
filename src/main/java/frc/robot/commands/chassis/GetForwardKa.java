package frc.robot.commands.chassis;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.chassis.Chassis;
import static frc.robot.Constants.ChassisConstants.SwerveModuleConstants.*;

public class GetForwardKa extends CommandBase {
  Chassis chassis;
  double power;
  double velocity;
  double lastV;
  double a;
  double kA;
  double kS = FORWORD_MOVE_KS;
  double kV = FORWORD_MOVE_KV;

  public GetForwardKa(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    SmartDashboard.putData(this);
  }

  @Override
  public void initialize() {
    lastV = chassis.getVelocity().getNorm();

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Velocity", ()-> velocity, null);
    builder.addDoubleProperty("Power", () ->  power, (double p) -> {this.power = p;});
    builder.addDoubleProperty("Accel",() -> a, null);
    builder.addDoubleProperty("Ka", ()->kA, null);
    builder.addDoubleProperty("Ks", ()->kS, null);
    builder.addDoubleProperty("Kv", ()->kV, null);
  }

  
  @Override
  public void execute() {
    chassis.setModulesPower(power);
    velocity = chassis.getVelocity().getNorm();
    a = velocity - lastV;
    lastV = velocity;
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.stop();
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.getPose().getX() >= 3;
  }
}
