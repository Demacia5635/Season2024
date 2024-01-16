// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  public CANSparkMax neon;
  public TalonFX motor1;
  public TalonFX motor2;
  
  double wantedVel = 0;
  
  /** Creates a new Shooter. */
  public Shooter() {
    neon = new CANSparkMax(NEON_ID, MotorType.kBrushless);
    neon.getEncoder().setPosition(0);
    
    motor1 = new TalonFX(MOTOR_1_ID);
    motor2 = new TalonFX(MOTOR_2_ID);
    motor2.setInverted(true);
    
    motor1.config_kP(0, KP);
    motor1.config_kI(0, KI);
    motor2.config_kP(0, KP);
    motor2.config_kI(0, KI);
    
    SmartDashboard.putData(this);
  }
  
  /**
   * @param vel must be from -1 to 1
   */
  public void neonSetVel(double vel){
    neon.set(vel);
  }
  
  public void neonEncoderSet(double postion){
    neon.getEncoder().setPosition(postion);
  }
  
  public void neonEncoderReset(){
    neonEncoderSet(0);
  }
  
  public double getNeonRev(){
    return neon.getEncoder().getPosition()/NEON_PULES_PER_REV;
  }
  
  public void neonMoveByRev(double vel, double rev){
    double startPos = getNeonRev();
    while (Math.abs(getNeonRev()-startPos+rev)==1){
      neonSetVel(vel);
      System.out.println("another"+(Math.abs(getNeonRev()-startPos+rev))+"rev");
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

      builder.addDoubleProperty("neon encoder", this::getNeonRev, null);
      builder.addDoubleProperty("motor 1 speed", ()-> motor1.getSelectedSensorPosition()*10/(FALCON_PULES_PER_REV/360), null);
      builder.addDoubleProperty("motor 2 speed", ()-> motor2.getSelectedSensorPosition()*10/(FALCON_PULES_PER_REV/360), null);
  
      SmartDashboard.putData("set wanted vel", new InstantCommand(()-> wantedVel = SmartDashboard.getNumber("wanted vel", 10000)));
      SmartDashboard.putData("RUN", new InstantCommand(()-> falconSetVel(wantedVel)));
      SmartDashboard.putData("STOP", new InstantCommand(()-> falconStop()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
