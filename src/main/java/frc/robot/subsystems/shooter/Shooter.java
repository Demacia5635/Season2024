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
    // public TalonFX motorAngle;

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

        neon = new CANSparkMax(NEON_ID, MotorType.kBrushless);
        neon.getEncoder().setPosition(0);
        // motorAngle = new TalonFX(ANLGE_MOTOR_ID);

        SmartDashboard.putData(this);
    }

    /**
     * @param pow must be from -1 to 1
     */
    public void neonSetPow(double pow){
        neon.set(pow);
    }
    
    // public void anlgeSetPow(double pow){
    //     motorAngle.set(ControlMode.PercentOutput, pow);
    // }

    public void neonStop(){
        neon.stopMotor();
    }

    // public void anlgeStop(){
    //     motorAngle.set(ControlMode.PercentOutput, 0);
    // }

    public void neonEncoderSet(double postion){
        neon.getEncoder().setPosition(postion);
    }
    
    public void neonEncoderReset(){
        neonEncoderSet(0);
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
    
    public void stopAll(){
        neonStop();
        // stop();
        // anlgeStop();
    }
    
    // public double getAnlgeEncoder(){
    //     return (motorAngle.getSelectedSensorPosition()/PULES_PER_REV) - basePos;
    // }

    // public void angleResetPos(){
    //     basePos = motorAngle.getSelectedSensorPosition()/PULES_PER_REV;
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty("motor 1 speed", ()-> motor1.getSelectedSensorVelocity()*10/(PULES_PER_REV/360), null);
        builder.addDoubleProperty("motor 2 speed", ()-> motor2.getSelectedSensorVelocity()*10/(PULES_PER_REV/360), null);
        builder.addDoubleProperty("current amper motor 1", ()-> motor1.getSupplyCurrent(), null);
        builder.addDoubleProperty("current amper motor 2", ()-> motor2.getSupplyCurrent(), null);
        builder.addDoubleProperty("neon encoder", ()->neon.getEncoder().getPosition(), null);
        // builder.addDoubleProperty("angle motor encoder", this::getAnlgeEncoder, null);
        builder.addDoubleProperty("base angle", ()->basePos, null);
    
        SmartDashboard.putData("Reset neon", new InstantCommand(()-> neonEncoderReset()));
        // SmartDashboard.putData("Reset angle motor", new InstantCommand(()-> angleResetPos()));
    }

}
