// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  public CANSparkMax neon = new CANSparkMax(NEON_ID, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    neon.getEncoder().setPosition(0);
  }

  /**
   * @param vel must be from -1 to 1
   */
  public void setVel(double vel){
    neon.set(vel);
  }

  public void encoderSet(double postion){
    neon.getEncoder().setPosition(postion);
  }

  public void encoderReset(){
    encoderSet(0);
  }

  public double getRev(){
    return neon.getEncoder().getPosition()/PULES_PER_ROTATION;
  }

  public void moveByRev(double vel, double rev){
    double startPos = getRev();
    while (Math.abs(getRev()-startPos+rev)==1){
      setVel(vel);
      System.out.println("another"+(Math.abs(getRev()-startPos+rev))+"rev");
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
      builder.addDoubleProperty("encoder", this::getRev, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
