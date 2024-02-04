package frc.robot.subsystems.telescope;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climb.BrakeTelescopeCommand;

import static frc.robot.Constants.ClimbConstants.*;

public class Telescope extends SubsystemBase {
  private final TalonFX rightMotor;
  private final TalonFX leftMotor;
  private final TalonSRX brakeMotor;
  private final DigitalInput rightLimitSwitch;
  private final DigitalInput leftLimitSwitch;

  public Telescope() {
    rightMotor = new TalonFX(RIGHT_CLIMB_MOTOR);
    leftMotor = new TalonFX(LEFT_CLIMB_MOTOR);
    brakeMotor = new TalonSRX(BRAKE_MOTOR);
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

  public void setBrakePower(int p) {
    brakeMotor.set(ControlMode.PercentOutput, p);
  }

  public double getBrakeCurrent() {
    return brakeMotor.getStatorCurrent();
  }

  public void brake() {
    new BrakeTelescopeCommand(this).schedule();
  }

  public void release() {
    new RunCommand(() -> setBrakePower(-1), this).withTimeout(0.5).andThen(() -> setBrakePower(0), this);
  }

  public enum TelescopeType {
    RIGHT,
    LEFT,
    BOTH
  }
}
