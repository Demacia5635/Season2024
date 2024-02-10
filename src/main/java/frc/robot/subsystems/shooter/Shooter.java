// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
    ArmFeedforward elevationFF = new ArmFeedforward(KS, KG, KV);
    
    DigitalInput limitSwitch;

    /**creates a new shooter and angle changer*/
    public Shooter() {

        motor1 = new TalonFX(MOTOR_1_ID);
        motor2 = new TalonFX(MOTOR_2_ID);
        motorFeeding = new TalonSRX(MOTOR_FEEDING_ID);
        
        motorAngle = new TalonFX(MOTOR_ANGLE_ID);
        motorAngle.setInverted(true);
        
        motorAngle.config_kP(0, KP);
        motorAngle.config_kD(0, KD);

        limitInput = new AnalogInput(LIMIT_INPUT_ID);
        limitInput.setAccumulatorInitialValue(0);
        limitSwitch = new DigitalInput(LIMIT_SWITCH_ID);

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
        motorAngle.set(ControlMode.MotionMagic, (PULES_PER_MM * (dis+baseDis)));
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
        return motorAngle.getSelectedSensorPosition()/PULES_PER_MM - baseDis;
    }

    /**
     * for saftey checks if the angle motor passed its limits
     * @param isUpDirection if the velocity the motor moves is positive or negative
     * @return if the limits have passed (false means you are fine)
     */
    public boolean isAtLimits(boolean isUpDirection){
        // if (isUpDirection){
        //     return getDis() >= 322;
        // } else {
        //     return getDis() <= 98;
        // }
        return isUpDirection ? getDis() >= 322 : getDis() <= 98;
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
     * get the vel of every motor
     * @param motor the wanted motor 
     * <pre>
     * capable param:
     * 1 - motor 1
     * 2 - motor 2
     * 3 - feeding motor
     * 4 - angle motor
     * </pre>
     * @return the wanted motor velocity in degree per sec
     * @exception 3 - that is a snowblower so return in pules per sec
     * @exception default - if the param motor is not listed above the function will return 0
     */
    public double getMotorVel(int motor){
        switch (motor) {

            case 1:
                return motor1.getSelectedSensorVelocity()*10/(PULES_PER_REV/360);

            case 2:
                return motor2.getSelectedSensorVelocity()*10/(PULES_PER_REV/360);

            case 3:
                return motorFeeding.getSelectedSensorVelocity()*10;

            case 4:
                return motorAngle.getSelectedSensorVelocity()*10/(PULES_PER_REV/360);
            
            default:
                return 0;
        }
    }

    /**
     * get the amper of every motor
     * @param motor the wanted motor 
     * <pre>
     * capable param:
     * 1 - motor 1
     * 2 - motor 2
     * 3 - feeding motor
     * 4 - angle motor
     * </pre>
     * @return the wanted motor amper
     * @exception default - if the param motor is not listed above the function will return 0
     */
    public double getSupplyCurrent(int motor){
        switch (motor) {

            case 1:
                return motor1.getSupplyCurrent();

            case 2:
                return motor2.getSupplyCurrent();

            case 3:
                return motorFeeding.getSupplyCurrent();

            case 4:
                return motorAngle.getSupplyCurrent();

            default:
                return 0;
        }
    }

    /**
     * check if the not had pass
     * @return if the limit volt is smaller than 4.55
     * @author Adar
     */
    public boolean isNote(){
        return getLimitVolt()<4.55;
    /**
     * get if the angle is at the end
     * @return if the limit switch is pressed
     */
    public boolean isLimit(){
        /*can change if see that limit switch return inverted */
        return limitSwitch.get();
    }

    /**
     * get the angle of the angle changer
     * @return the angle in degrees
     * @see also use the g(x) from desmos {@link https://www.desmos.com/calculator/4ja9zotx82}
     */
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
        
        /*put on ShuffleBoard all the builders */
        builder.addDoubleProperty("motor 1 speed", ()-> getMotorVel(1), null);
        builder.addDoubleProperty("motor 2 speed", ()-> getMotorVel(2), null);
        builder.addDoubleProperty("current amper motor 1", ()-> getSupplyCurrent(1), null);
        builder.addDoubleProperty("current amper motor 2", ()-> getSupplyCurrent(2), null);
        builder.addDoubleProperty("angle vel", ()-> getMotorVel(4), null);
        builder.addDoubleProperty("Distance", this::getDis, null);
        builder.addDoubleProperty("base dis", ()-> baseDis, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("encoder", ()->motorAngle.getSelectedSensorPosition(), null);
        builder.addBooleanProperty("Limit switch", ()->isLimit(), null);
    
        /*put on ShuffleBoard all the cmds */
        SmartDashboard.putData("Dis reset", new InstantCommand(()-> resetDis()).ignoringDisable(true));
        SmartDashboard.putData("Angle Brake", new InstantCommand(()-> angleBrake()).ignoringDisable(true));
        SmartDashboard.putData("Angle Coast", new InstantCommand(()-> angleCoast()).ignoringDisable(true));
    @Override
    public void periodic() {
        super.periodic();
        
        /*if the angle is at the end it reset the dis */
        if (isLimit()){
            resetDis();
        }
    }
}
