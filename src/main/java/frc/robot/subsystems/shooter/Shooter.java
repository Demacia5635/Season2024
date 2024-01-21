// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  public CANSparkMax neo;
  public TalonFX motor1;
  public TalonFX motor2;
  
  /** Creates a new Shooter. */
  public Shooter() {
    neo = new CANSparkMax(NEO_ID, MotorType.kBrushless);
    neo.getEncoder().setPosition(0);
    
    motor1 = new TalonFX(MOTOR_1_ID);
    motor2 = new TalonFX(MOTOR_2_ID);
    
    motor1.config_kP(0, KP);
    motor1.config_kI(0, KI);
    motor2.config_kP(0, KP);
    motor2.config_kI(0, KI);
    
    SmartDashboard.putData(this);
  }
  
  /**
   * @param vel must be from -1 to 1
   */
  public void neoSetVel(double vel){
    neo.set(vel);
  }
  
  public void neoEncoderSet(double postion){
    neo.getEncoder().setPosition(postion);
  }
  
  public void neoEncoderReset(){
    neoEncoderSet(0);
  }
  
  public double getNEORev(){
    return neo.getEncoder().getPosition()/NEO_PULES_PER_REV;
  }
  
  public void neoMoveByRev(double vel, double rev){
    double startPos = getNEORev();
    while (Math.abs(getNEORev()-startPos+rev)==1){
      neoSetVel(vel);
      System.out.println("another"+(Math.abs(getNEORev()-startPos+rev))+"rev");
    }
  }
  
  public void falconSetVel(double vel){
    double pow = KS + KV * vel;
    System.out.println("Vel = "+ vel);
    motor1.set(ControlMode.PercentOutput, pow);
    motor2.set(ControlMode.PercentOutput, pow);
  }

  public void falconStop(){
    motor1.set(ControlMode.PercentOutput, 0);
    motor2.set(ControlMode.PercentOutput, 0);
  }
  
  
  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);

      builder.addDoubleProperty("neo encoder", this::getNEORev, null);
      builder.addDoubleProperty("motor 1 speed", ()-> motor1.getSelectedSensorVelocity()*10/(FALCON_PULES_PER_REV/360), null);
      builder.addDoubleProperty("motor 2 speed", ()-> motor2.getSelectedSensorVelocity()*10/(FALCON_PULES_PER_REV/360), null);
      builder.addDoubleProperty("current amper motor 1", ()-> motor1.getSupplyCurrent(), null);
      builder.addDoubleProperty("current amper motor 2", ()-> motor2.getSupplyCurrent(), null);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
