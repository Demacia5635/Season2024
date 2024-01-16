// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import static frc.robot.Constants.ClimbConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX Right_Climb_Motor;
  private TalonFX Left_Climb_Motor;
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    Right_Climb_Motor = new TalonFX(RIGHT_CLIMB_MOTOR);
    Left_Climb_Motor = new TalonFX(LEFT_CLIMB_MOTOR); 
  }
  public void climbRight(double climbSpeed){
    // Control motor speed until it reaches normal voltage
    Right_Climb_Motor.set(ControlMode.PercentOutput,climbSpeed);
    
  }
  public void climbLeft(double climbSpeed){
    // Control motor speed until it reaches normal voltage
    Left_Climb_Motor.set(ControlMode.PercentOutput,climbSpeed);
    
  }
  public double getLeftVolteg(){
    return Left_Climb_Motor.getMotorOutputVoltage();
  }
  public double getRoghtVolteg(){
    return Right_Climb_Motor.getMotorOutputVoltage();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
