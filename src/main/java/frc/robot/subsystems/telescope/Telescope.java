package frc.robot.subsystems.telescope;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class Telescope extends SubsystemBase {
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;
  private final TalonSRX rightWormMotor;
  private final TalonSRX leftWormMotor;
  private final DigitalInput rightLimitSwitch;
  private final DigitalInput leftLimitSwitch;

  public Telescope() {
    rightMotor = new TalonFX(RIGHT_CLIMB_MOTOR);
    leftMotor = new TalonFX(LEFT_CLIMB_MOTOR);
    rightWormMotor = new TalonSRX(RIGHT_WORM_MOTOR);
    leftWormMotor = new TalonSRX(LEFT_WORM_MOTOR);
    rightLimitSwitch = new DigitalInput(RIGHT_LIMIT_SWITCH);
    leftLimitSwitch = new DigitalInput(LEFT_LIMIT_SWITCH);
  }

  public void setRightPower(double p) {
    rightMotor.set(ControlMode.PercentOutput, p); 
  }

  public void setLeftPower(double p) {
    leftMotor.set(ControlMode.PercentOutput, p); 
  }

  public boolean getRightLimSwitch() {
    return rightLimitSwitch.get();
  }

  public boolean getLeftLimSwitch() {
    return leftLimitSwitch.get();
  }

  public void stopRight() {
    setRightPower(0);
  }

  public void stopLeft() {
    setLeftPower(0);
  }

  public void stop() {
    stopRight();
    stopLeft();
  }
}
