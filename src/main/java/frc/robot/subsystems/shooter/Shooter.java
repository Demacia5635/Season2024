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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.subsystems.shooter.ShooterConstants;

/**subsystem shooter and angle changing */
public class Shooter extends SubsystemBase {

    /**the motor that controls the angle of the shooter */
    public final TalonFX motorAngle;

    /**the motors that move the rollers */
    public final TalonFX motor1; 
    /**the motors that move the rollers */
    public final TalonFX motor2;
    
    /**the motor that feeds the notes to the shooter */
    public final TalonSRX motorFeeding;

    /**the sensor that detect if there is a note in the shooter */
    public AnalogInput limitInput;

    /**a var that calculate the distance of the angle changer*/
    double baseDis = -322;

    /** @deprecated Unused bcz switch to motionMagic from velocity*/
    ArmFeedforward elevationFF = new ArmFeedforward(ShooterConstants.KS, ShooterConstants.KG, ShooterConstants.KV);
    
    /**creates a new shooter and angle changer*/
    public Shooter() {

        motor1 = new TalonFX(ShooterConstants.MOTOR_1_ID);
        motor2 = new TalonFX(ShooterConstants.MOTOR_2_ID);
        motorFeeding = new TalonSRX(ShooterConstants.MOTOR_FEEDING_ID);
        
        motorAngle = new TalonFX(ShooterConstants.MOTOR_ID);
        motorAngle.setInverted(true);
        
        motorAngle.config_kP(0, ShooterConstants.KP);
        motorAngle.config_kD(0, ShooterConstants.KD);

        limitInput = new AnalogInput(ShooterConstants.LIMIT_INPUT_ID);
        limitInput.setAccumulatorInitialValue(0);

        SmartDashboard.putData(this);
        SmartDashboard.putData(null);
    }
    
    /**
     * creates trapezoid algo 
     * @param dis the wanted dis in mm on the screw (absolute)
     * @param maxVel the max velocity of the trapezoid in pules per 1/10 sec
     * @param acc the acc of the trapezoid in pules per 1/10 sec
     */
    public void angleMotionMagic(double dis, double maxVel, double acc) {
        motorAngle.configMotionCruiseVelocity(maxVel);
        motorAngle.configMotionAcceleration(acc);
        motorAngle.set(ControlMode.MotionMagic, (ShooterConstants.PULES_PER_MM * (dis+baseDis)));
    }

    /**
     * @deprecated
     * Unues bcz switch to motionMagic from velocity
     * @param vel the wanted vel in pules per 1/10 sec
     * @see also using feedforward from line 39
     */
    public void angleSetVel(double vel){
        double ff = elevationFF.calculate(Math.toRadians(getAngle()), vel);
        // System.out.println("ff = "+ ff);
        motorAngle.set(ControlMode.Velocity, vel, DemandType.ArbitraryFeedForward, ff);
    }

    /**
     * set the pow of the amgle motor
     * @param pow the wanted pow in -1 to 1
     */
    public void angleSetPow(double pow){
        motorAngle.set(ControlMode.PercentOutput, pow);
    }

    /**stop the angle motor */
    public void anlgeStop(){
        motorAngle.set(ControlMode.PercentOutput, 0);
    }

    /**
     * set the pow of the shooters motors
     * @param pow the wanted pow from -1 to 1
     */
    public void setPow(double pow){
        motor1.set(ControlMode.PercentOutput, pow);
        motor2.set(ControlMode.PercentOutput, pow);
    }
    
    public void stop(){
        motor1.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);
    }
    
    /**
     * set the pow of the motor that feeding the shooters motors
     * @param pow the wanted pow from -1 to 1
     */
    public void feedingSetPow(double pow){
        motorFeeding.set(ControlMode.PercentOutput, pow);
    }

    /**stop the feeding motor */
    public void feedingStop(){
        motorFeeding.set(ControlMode.PercentOutput, 0);
    }

    /**important saftey function that will stop all the motors in this subsystem */
    public void stopAll(){
        stop();
        anlgeStop();
        feedingStop();
    }

    /**make the angle motor on brake */
    public void angleBrake(){ 
        motorAngle.setNeutralMode(NeutralMode.Brake);
    }
    /**make the angle motor on coast */
    public void angleCoast(){ 
        motorAngle.setNeutralMode(NeutralMode.Coast);
    }
    
    /**reset the base dis of the angle motor also reset the encoder of the angle motor */
    public void resetDis(){
        motorAngle.setSelectedSensorPosition(0);
        baseDis = -322;
    }
    
    /**
     * caculate the dis the angle motor at
     * @return the dis in mm
     */
    public double getDis(){
        return motorAngle.getSelectedSensorPosition()/ShooterConstants.PULES_PER_MM - baseDis;
    }

    /**
     * for saftey checks if the angle motor passed its limits
     * @param isUpDirection if the velocity the motor moves is positive or negative
     * @return if the limits have passed (false means you are fine)
     */
    public boolean limits(boolean isUpDirection){
        // if (isUpDirection){
        //     return getDis() >= 322;
        // } else {
        //     return getDis() <= 98;
        // }
        return isUpDirection ? getDis() >= 322 : getDis() <= 98;
    }

    /**
     * get the current velocity of the angle motor
     * @return the angle motor velocity angles per sec
     */
    public double getAngleVel(){ 
        return motorAngle.getSelectedSensorVelocity()*10/(ShooterConstants.PULES_PER_REV * ShooterConstants.GEAR_RATIO / 360); 
    }

    /**
     * get the limit input voltage
     * @return the limit input voltage
     * @author Adar
     */ 
    public double getLimitVolt(){
        return limitInput.getVoltage();
    }

    /**
     * check if the not had pass
     * @return if the limit volt is smaller than 4.55
     * @author Adar
     */
    public boolean didNotePass(){
        return getLimitVolt()<4.55;
    }

    /**
     * get the angle of the angle changer
     * @return the angle in degrees
     * @see also use the g(x) from desmos {@link https://www.desmos.com/calculator/4ja9zotx82}
     */
    public double getAngle(){
        double angle = (
        Math.acos(-1 * ((Math.pow(ShooterConstants.KB, 2) - 
        Math.pow(ShooterConstants.KA, 2) - 
        Math.pow(getDis(), 2)) / 
        (2 * ShooterConstants.KA * getDis()))) * 
        180 / Math.PI
        );
        
        return angle;
    }

    /**
     * @deprecated
     * for future use needs to set up var
     * @return if a run been done
     */
    public boolean isRunDone(){
        double regularAmper = 0;
        double deltaAmper = 0;
        return Math.abs(regularAmper - motor1.getSupplyCurrent()) > deltaAmper;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty("motor 1 speed", ()-> motor1.getSelectedSensorVelocity()*10/(ShooterConstants.PULES_PER_REV/360), null);
        builder.addDoubleProperty("motor 2 speed", ()-> motor2.getSelectedSensorVelocity()*10/(ShooterConstants.PULES_PER_REV/360), null);
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
