// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Shooter extends SubsystemBase {

    public final TalonFX motorAngle;

    public final TalonFX motor1;
    public final TalonFX motor2;
    public final TalonSRX motorFeeding;

    public AnalogInput limitInput;

    double baseDis = -322;
    ArmFeedforward elevationFF = new ArmFeedforward(KS, KG, KV);
    
    /** Creates a new Shooter. */
    public Shooter() {

        motor1 = new TalonFX(MOTOR_1_ID);
        motor2 = new TalonFX(MOTOR_2_ID);
        motorFeeding = new TalonSRX(MOTOR_FEEDING_ID);
        
        motorAngle = new TalonFX(MOTOR_ID);
        motorAngle.setInverted(true);
        
        motorAngle.config_kP(0, KP);
        motorAngle.config_kD(0, KD);

        limitInput = new AnalogInput(LIMIT_INPUT_ID);
        limitInput.setAccumulatorInitialValue(0);

        SmartDashboard.putData(this);
        SmartDashboard.putData(null);
    }
    
    public void angleMotionMagic(double dis, double maxVel, double acc) {
        motorAngle.configMotionCruiseVelocity(maxVel);
        motorAngle.configMotionAcceleration(acc);
        motorAngle.set(ControlMode.MotionMagic, (PULES_PER_MM * (dis+baseDis)));
    }

    public void angleSetVel(double vel){
        double ff = elevationFF.calculate(Math.toRadians(getAngle()), vel);
        // System.out.println("ff = "+ ff);
        motorAngle.set(ControlMode.Velocity, vel, DemandType.ArbitraryFeedForward, ff);
    }

    public void angleSetPow(double pow){
        motorAngle.set(ControlMode.PercentOutput, pow);
    }

    public void anlgeStop(){
        motorAngle.set(ControlMode.PercentOutput, 0);
    }

    public void setPow(double pow){
        motor1.set(ControlMode.PercentOutput, pow);
        motor2.set(ControlMode.PercentOutput, pow);
    }
    
    public void feedingSetPow(double pow){
        motorFeeding.set(ControlMode.PercentOutput, pow);
    }

    public void feedingStop(){
        motorFeeding.set(ControlMode.PercentOutput, 0);
    }

    public void stop(){
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }
    
    public void stopAll(){
        stop();
        anlgeStop();
        feedingStop();
    }

    public void angleBrake(){ 
        motorAngle.setNeutralMode(NeutralMode.Brake);
    }
    public void angleCoast(){ 
        motorAngle.setNeutralMode(NeutralMode.Coast);
    }
    
    public void resetDis(){
        motorAngle.setSelectedSensorPosition(0);
        baseDis = -322;
    }
    
    public double getDis(){
        return motorAngle.getSelectedSensorPosition()/PULES_PER_MM - baseDis;
    }

    public boolean limits(boolean isUpDirection){
        // if (isUpDirection){
        //     return getDis() >= 322;
        // } else {
        //     return getDis() <= 98;
        // }
        return isUpDirection ? getDis() >= 322 : getDis() <= 98;
    }

    public double getAngleVel(){ 
        return motorAngle.getSelectedSensorVelocity()*10/(PULES_PER_REV * GEAR_RATIO / 360); 
    }

     public double getLimitVolt(){
        return limitInput.getVoltage();
    }
    public boolean didNotePass(){
        return getLimitVolt()<4.55;
    }

    public double getAngle(){
        double angle = (
        Math.acos(-1 * ((Math.pow(KB, 2) - 
        Math.pow(KA, 2) - 
        Math.pow(getDis(), 2)) / 
        (2 * KA * getDis()))) * 
        180 / Math.PI
        );
        
        return angle;
    }

    /** for future use still needs to set var */
    public boolean isRunDone(){
        double regularAmper = 0;
        double deltaAmper = 0;
        return Math.abs(regularAmper - motor1.getSupplyCurrent()) > deltaAmper;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty("motor 1 speed", ()-> motor1.getSelectedSensorVelocity()*10/(PULES_PER_REV/360), null);
        builder.addDoubleProperty("motor 2 speed", ()-> motor2.getSelectedSensorVelocity()*10/(PULES_PER_REV/360), null);
        builder.addDoubleProperty("current amper motor 1", ()-> motor1.getSupplyCurrent(), null);
        builder.addDoubleProperty("current amper motor 2", ()-> motor2.getSupplyCurrent(), null);
        builder.addDoubleProperty("angle vel", this::getAngleVel, null);
        builder.addDoubleProperty("Distance", this::getDis, null);
        builder.addDoubleProperty("base dis", ()-> baseDis, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("encoder", ()->motorAngle.getSelectedSensorPosition(), null);
    
        SmartDashboard.putData("Dis reset", new InstantCommand(()-> resetDis()).ignoringDisable(true));
        SmartDashboard.putData("Angle Brake", new InstantCommand(()-> angleBrake()).ignoringDisable(true));
        SmartDashboard.putData("Angle Coast", new InstantCommand(()-> angleCoast()).ignoringDisable(true));
    }
}
