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

public class Shooter extends SubsystemBase {
    public final TalonFX motorAngle;
    public final TalonFX motor1;
    public final TalonFX motor2;

    public double basePos = 0;
    
    /** Creates a new Shooter. */
    public Shooter() {

        motor1 = new TalonFX(MOTOR_1_ID);
        motor2 = new TalonFX(MOTOR_2_ID);
        
        motor1.config_kP(0, KP);
        motor1.config_kI(0, KI);
        motor2.config_kP(0, KP);
        motor2.config_kI(0, KI);

        motorAngle = new TalonFX(MOTOR_ANGLE_ID);

        basePos = motorAngle.getSelectedSensorPosition()/ PULES_PER_REV; 
        
        SmartDashboard.putData(this);
    }

    public void setAngleVel(double vel){
        motorAngle.set(ControlMode.Velocity, vel);
    }

    public void setVel(double vel){
        double pow = KS + KV * vel;
        System.out.println("power = "+ pow);
        motor1.set(ControlMode.PercentOutput, pow);
        motor2.set(ControlMode.PercentOutput, pow);
    }

    public void stop(){
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }
    
    public void stopAngle() {
        motorAngle.set(ControlMode.PercentOutput, 0);
    }
    
    public double getAngleEncoder(){
        return motorAngle.getSelectedSensorPosition()/ PULES_PER_REV - basePos;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("motor 1 speed", ()-> motor1.getSelectedSensorVelocity()*10/(PULES_PER_REV/360), null);
        builder.addDoubleProperty("motor 2 speed", ()-> motor2.getSelectedSensorVelocity()*10/(PULES_PER_REV/360), null);
        builder.addDoubleProperty("current amper motor 1", ()-> motor1.getSupplyCurrent(), null);
        builder.addDoubleProperty("current amper motor 2", ()-> motor2.getSupplyCurrent(), null);
        builder.addDoubleProperty("motor angle", this::getAngleEncoder, null);
        builder.addDoubleProperty("motor angle speed", ()->motorAngle.getSelectedSensorVelocity(), null);
    
        SmartDashboard.putData("reset base position", new InstantCommand(()-> basePos = motorAngle.getSelectedSensorPosition()/PULES_PER_REV).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
