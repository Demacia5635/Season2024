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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    public final TalonFX motorUP; 
    /**the motors that move the rollers */
    public final TalonFX motorDown;
    
    /**the motor that feeds the notes to the shooter */
    public final TalonSRX motorFeeding;

    /**the sensor that detect if there is a note in the shooter */
    AnalogInput analogInput;

    /**a var that calculate the distance of the angle changer*/
    double baseDis;

    /** @deprecated Unused bcz switch to motionMagic from velocity*/
    ArmFeedforward elevationFF;
    
    /**the limit switch on the angle changer machanism */
    DigitalInput limitSwitch;

    /**
     * <pre>
     * enum SHOOTER_MOTOR that contains all motors of the shooter
     * UP - the shooting motor thats up
     * DOWN - the shooting motor thats down
     * FEEDING - the feeding motor
     * ANGLE - the motor of the angle changer
     * </pre>
     */
    public enum SHOOTER_MOTOR {
        UP,
        DOWN,
        FEEDING,
        ANGLE,
    }

    /**creates a new shooter and angle changer*/
    public Shooter() {
        /*set up vars */

        motorUP = new TalonFX(ShooterConstants.MOTOR_UP_ID);

        motorDown = new TalonFX(ShooterConstants.MOTOR_DOWN_ID);

        motorFeeding = new TalonSRX(ShooterConstants.MOTOR_FEEDING_ID);
        
        motorAngle = new TalonFX(ShooterConstants.MOTOR_ANGLE_ID);
        motorAngle.config_kP(0, ShooterConstants.KP);
        motorAngle.config_kD(0, ShooterConstants.KD);

        analogInput = new AnalogInput(ShooterConstants.ANALOG_INPUT_ID);
        analogInput.setAccumulatorInitialValue(0);

        baseDis = -ShooterConstants.MAX_DIS;

        elevationFF = new ArmFeedforward(ShooterConstants.KS, ShooterConstants.KG, ShooterConstants.KV);

        limitSwitch = new DigitalInput(ShooterConstants.LIMIT_SWITCH_ID);

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
    public void angleStop(){
        motorAngle.set(ControlMode.PercentOutput, 0);
    }

    /**
     * set the pow of the shooters motors
     * @param pow the wanted pow from -1 to 1
     */
    public void setPow(double pow){
        motorUP.set(ControlMode.PercentOutput, pow);
        motorDown.set(ControlMode.PercentOutput, pow);
    }
    
    public void stop(){
        motorUP.set(ControlMode.PercentOutput, 0);
        motorDown.set(ControlMode.PercentOutput, 0);
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

    public double getFeedingEncoderPose() {
        return motorFeeding.getSelectedSensorPosition();
    }

    /**important saftey function that will stop all the motors in this subsystem */
    public void stopAll(){
        stop();
        angleStop();
        feedingStop();
    }

    /**
     * set brake mode to motors
     * @param motor the wanted motors
     */
    public void brake(SHOOTER_MOTOR... motor){ 
        for (SHOOTER_MOTOR i : motor) {
            switch (i) {
                case UP:
                    motorUP.setNeutralMode(NeutralMode.Brake);
                    break;
    
                case DOWN:
                    motorDown.setNeutralMode(NeutralMode.Brake);
                    break;
    
                case FEEDING:
                    motorFeeding.setNeutralMode(NeutralMode.Brake);
                    break;
    
                case ANGLE:
                    motorUP.setNeutralMode(NeutralMode.Brake);
                    break;
    
                default:
                    break;
            }
            
        }
    }
    
    /**
     * set up coast to motors 
     * @param motor the wanted motors
     */
    public void coast(SHOOTER_MOTOR... motor){ 
        for (SHOOTER_MOTOR i : motor) {
            switch (i) {
    
                case UP:
                    motorUP.setNeutralMode(NeutralMode.Coast);
                    break;
    
                case DOWN:
                    motorDown.setNeutralMode(NeutralMode.Coast);
                    break;
    
                case FEEDING:
                    motorFeeding.setNeutralMode(NeutralMode.Coast);
                    break;
    
                case ANGLE:
                    motorUP.setNeutralMode(NeutralMode.Coast);
                    break;
    
                default:
                    break;
            }
            
        }
    }
    
    /**reset the base dis of the angle motor also reset the encoder of the angle motor */
    public void resetDis(){
        motorAngle.setSelectedSensorPosition(0);
        baseDis = -ShooterConstants.MAX_DIS;
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
    public boolean isDisLimits(boolean isUpDirection){
        return isUpDirection ? getDis() >= ShooterConstants.MAX_DIS : getDis() <= ShooterConstants.MIN_DIS;
    }

    public boolean isSupplyLimit(SHOOTER_MOTOR motor){
        return getSupplyCurrent(motor) >= 25;
    }

    /**
     * get the limit input voltage
     * @return the limit input voltage
     * @author Adar
     */ 
    public double getAnalogVolt(){
        return analogInput.getVoltage();
    }

    /**
     * get the vel of every motor
     * @param motor the wanted motor 
     * @return the wanted motor velocity in degree per sec
     * @exception FEEDING - that is a snowblower so return in pules per sec
     * @exception default - if the param motor is not listed above the function will return 0
     */
    public double getMotorVel(SHOOTER_MOTOR motor){
        switch (motor) {

            case UP:
                return motorUP.getSelectedSensorVelocity()*10/(ShooterConstants.PULES_PER_REV/360);

            case DOWN:
                return motorDown.getSelectedSensorVelocity()*10/(ShooterConstants.PULES_PER_REV/360);

            case FEEDING:
                return motorFeeding.getSelectedSensorVelocity()*10;

            case ANGLE:
                return motorAngle.getSelectedSensorVelocity()*10/(ShooterConstants.PULES_PER_REV/360);
            
            default:
                return 0;
        }
    }

    /**
     * get the amper of every motor
     * @param motor the wanted motor 
     * @return the wanted motor amper
     * @exception default - if the param motor is not listed above the function will return 0
     */
    public double getSupplyCurrent(SHOOTER_MOTOR motor){
        switch (motor) {

            case UP:
                return motorUP.getSupplyCurrent();

            case DOWN:
                return motorDown.getSupplyCurrent();

            case FEEDING:
                return motorFeeding.getSupplyCurrent();

            case ANGLE:
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
        return getAnalogVolt()<ShooterConstants.VOLT_NOTE_PRESENT;
    }

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
        return Math.abs(regularAmper - getSupplyCurrent(SHOOTER_MOTOR.UP)) > deltaAmper;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        SmartDashboard.putData("feeding", new RunCommand(()->feedingSetPow(0.5), this));
        SmartDashboard.putData("shooter", new RunCommand(()->setPow(0.5), this));
        SmartDashboard.putData("angle shooter", new RunCommand(()->angleSetPow(0.2), this));

        SmartDashboard.putNumber("analog input", getAnalogVolt());
        
        // /*put on ShuffleBoard all the builders */
        // builder.addDoubleProperty("motor up speed", ()-> getMotorVel(SHOOTER_MOTOR.UP), null);
        // builder.addDoubleProperty("motor down speed", ()-> getMotorVel(SHOOTER_MOTOR.DOWN), null);
        // builder.addDoubleProperty("current amper motor 1", ()-> getSupplyCurrent(SHOOTER_MOTOR.UP), null);
        // builder.addDoubleProperty("current amper motor 2", ()-> getSupplyCurrent(SHOOTER_MOTOR.DOWN), null);
        // builder.addDoubleProperty("angle vel", ()-> getMotorVel(SHOOTER_MOTOR.ANGLE), null);
        // builder.addDoubleProperty("Distance", this::getDis, null);
        // builder.addDoubleProperty("base dis", ()-> baseDis, null);
        // builder.addDoubleProperty("Angle", this::getAngle, null);
        // builder.addDoubleProperty("encoder", ()->motorAngle.getSelectedSensorPosition(), null);
        // builder.addBooleanProperty("Limit switch", ()->isLimit(), null);
    
        // /*put on ShuffleBoard all the cmds */
        // SmartDashboard.putData("Dis reset", new InstantCommand(()-> resetDis()).ignoringDisable(true));
        // SmartDashboard.putData("motor up Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.UP)).ignoringDisable(true));
        // SmartDashboard.putData("motor up Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.UP)).ignoringDisable(true));
        // SmartDashboard.putData("motor down Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.DOWN)).ignoringDisable(true));
        // SmartDashboard.putData("motor down Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.DOWN)).ignoringDisable(true));
        // SmartDashboard.putData("motor feeding Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.FEEDING)).ignoringDisable(true));
        // SmartDashboard.putData("motor feeding Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.FEEDING)).ignoringDisable(true));
        // SmartDashboard.putData("motor angle Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.ANGLE)).ignoringDisable(true));
        // SmartDashboard.putData("motor angle Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.ANGLE)).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        super.periodic();
        
        /*if the angle is at the end it reset the dis */
        if (isLimit()){
            resetDis();
        }
    }
}
