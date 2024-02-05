package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climb.BrakeClimbCommand;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbSubsystem extends SubsystemBase {
  public final ClimbBrakerSubsystem s_Braker;

  private final TalonFX s_RightMotor;
  private final TalonFX s_LeftMotor;
  private final DigitalInput s_RightLimitSwitch;
  private final DigitalInput s_LeftLimitSwitch;
  // TODO - add left/right encoder at low position 

  public ClimbSubsystem() {
    s_Braker = new ClimbBrakerSubsystem();

    s_RightMotor = new TalonFX(RIGHT_CLIMB_MOTOR);
    s_LeftMotor = new TalonFX(LEFT_CLIMB_MOTOR);
    s_RightLimitSwitch = new DigitalInput(RIGHT_LIMIT_SWITCH);
    s_LeftLimitSwitch = new DigitalInput(LEFT_LIMIT_SWITCH);

    s_RightMotor.configFactoryDefault();
    s_LeftMotor.configFactoryDefault();

    s_RightMotor.setInverted(false);
    s_LeftMotor.setInverted(false);
  }

  public void setRightPower(double p) {
    s_RightMotor.set(ControlMode.PercentOutput, p); 
  }

  public void setLeftPower(double p) {
    s_LeftMotor.set(ControlMode.PercentOutput, p); 
  }

  public boolean getRightLimSwitch() {
    return s_RightLimitSwitch.get();
  }

  public boolean getLeftLimSwitch() {
    return s_LeftLimitSwitch.get();
  }

  public double getRightPosition() {
    return s_RightMotor.getSelectedSensorPosition();
  }

  public double getLeftPosition() {
    return s_LeftMotor.getSelectedSensorPosition();
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

  public void resetEncoders() {
    s_RightMotor.setSelectedSensorPosition(0);
    s_LeftMotor.setSelectedSensorPosition(0);
  }
}
