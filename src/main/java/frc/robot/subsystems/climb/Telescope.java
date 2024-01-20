// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import static frc.robot.Constants.ClimbConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
  private TalonFX rightMotor;
  private TalonFX leftMotor;
  /** Creates a new ClimbSubsystem. */
  public Telescope() {
    rightMotor = new TalonFX(RIGHT_CLIMB_MOTOR);
    leftMotor = new TalonFX(LEFT_CLIMB_MOTOR); 
  }
  public void setRightPower(double p){
    // Control motor speed until it reaches normal voltage
    rightMotor.set(ControlMode.PercentOutput, p);
    
  }
  public void setLeftPower(double p){
    // Control motor speed until it reaches normal voltage
    leftMotor.set(ControlMode.PercentOutput, p); 
  }

  public void stop() {
    setRightPower(0);
    setLeftPower(0);
  }
}
