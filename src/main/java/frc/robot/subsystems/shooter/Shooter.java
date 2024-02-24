// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static frc.robot.subsystems.shooter.ShooterConstants.PULES_PER_REV;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotContainer;
import frc.robot.Sysid.Sysid;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.commands.shooter.AngleQuel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.Shooting;
import frc.robot.subsystems.shooter.utils.LookUpTable;
import frc.robot.subsystems.shooter.ShooterConstants.AngleChanger;;

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

    /** @deprecated Unused bcz switch to motionMagic from velocity*/
    ArmFeedforward elevationFF;
    
    /**the limit switch on the angle changer machanism */
    DigitalInput limitSwitch;

    /**the lookup table */
    LookUpTable lookUpTable;

    private boolean isShooting  = false;
    private boolean isShootingAmp = false;
    private boolean isShootingReady = false;
    private boolean isActive = false;
    
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
        motorUP.configFactoryDefault();
        motorUP.setInverted(true);
        motorUP.config_kP(0, ShooterConstants.Shooting.KP);

        motorDown = new TalonFX(ShooterConstants.MOTOR_DOWN_ID);
        motorDown.configFactoryDefault();
        motorDown.config_kP(0, ShooterConstants.Shooting.KP);

        motorFeeding = new TalonSRX(ShooterConstants.MOTOR_FEEDING_ID);
        motorFeeding.setInverted(true);
        motorFeeding.enableCurrentLimit(true);
        motorFeeding.configContinuousCurrentLimit(20);
        
        motorAngle = new TalonFX(ShooterConstants.MOTOR_ANGLE_ID);
        motorDown.configFactoryDefault();
        motorAngle.config_kP(0, ShooterConstants.AngleChanger.KP);
        motorAngle.config_kD(0, ShooterConstants.AngleChanger.KD);
        motorAngle.configMotionCruiseVelocity(20000);
        motorAngle.configMotionAcceleration(20000);
        
        analogInput = new AnalogInput(ShooterConstants.ANALOG_INPUT_ID);
        analogInput.setAccumulatorInitialValue(0);

        elevationFF = new ArmFeedforward(ShooterConstants.AngleChanger.KS, ShooterConstants.AngleChanger.KG, ShooterConstants.AngleChanger.KV);

        limitSwitch = new DigitalInput(ShooterConstants.LIMIT_SWITCH_ID);

        lookUpTable  = new LookUpTable(ShooterConstants.LookUpTable.lookUpTable);

        brake(SHOOTER_MOTOR.UP, SHOOTER_MOTOR.DOWN, SHOOTER_MOTOR.FEEDING, SHOOTER_MOTOR.ANGLE);
        SmartDashboard.putData(this);

        SmartDashboard.putNumber("shoot angle set", 55);
        SmartDashboard.putNumber("shoot velocity set", 20);
 
    }
    
    /**
     * @deprecated
     * creates trapezoid algo 
     * @param dis the wanted dis in mm on the screw (absolute)
     */
    public void angleMotionMagic(double dis) {
        motorAngle.set(ControlMode.MotionMagic, (ShooterConstants.AngleChanger.PULES_PER_MM * (dis + ShooterConstants.AngleChanger.MAX_DIS)));
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
    
    public double getFF(double vel){
        return (ShooterConstants.Shooting.KS * Math.signum(vel)+
                ShooterConstants.Shooting.KV * vel +
                ShooterConstants.Shooting.KV2 * Math.pow(vel, 2));
    }
    
    public void setVel(double velUp, double velDown) {
        double ffUp = getFF(velUp);
        double wantedVelUp = (velUp/ 10) / ShooterConstants.PEREMITER_OF_WHEEL * PULES_PER_REV;
        motorUP.set(ControlMode.Velocity, wantedVelUp, DemandType.ArbitraryFeedForward, ffUp);
        double ffDown = getFF(velDown);
        double wantedVelDown = (velDown/ 10) / ShooterConstants.PEREMITER_OF_WHEEL * PULES_PER_REV;
        motorDown.set(ControlMode.Velocity, wantedVelDown, DemandType.ArbitraryFeedForward, ffDown);
        
    }

    public void setVel(double vel){
        setVel(vel,vel);
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
        isShooting = false;
        isShootingAmp = false;
        isShootingReady = false;
        isActive = false;
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
                    motorAngle.setNeutralMode(NeutralMode.Brake);
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
                    motorAngle.setNeutralMode(NeutralMode.Coast);
                    break;
    
                default:
                    break;
            }
            
        }
    }
    
    /**reset the base dis of the angle motor also reset the encoder of the angle motor */
    public void resetDis(){
        motorAngle.setSelectedSensorPosition(0);
    }

    public double getDistanceFromAngle(double wantedAngle) {
        double rad = Math.toRadians(wantedAngle);
        double aCos = AngleChanger.KA * Math.cos(rad); 
        return  aCos + 
            Math.sqrt(Math.pow(aCos, 2) - Math.pow(AngleChanger.KA, 2) + Math.pow(AngleChanger.KB, 2));
    }
    
    /**
     * caculate the dis the angle motor at
     * @return the dis in mm
     */
    public double getDis(){
        return motorAngle.getSelectedSensorPosition()/ShooterConstants.AngleChanger.PULES_PER_MM - ShooterConstants.AngleChanger.MAX_DIS;
    }


    /**
     * for saftey checks if the angle motor passed its limits
     * @param isUpDirection if the velocity the motor moves is positive or negative
     * @return if the limits have passed (false means you are fine)
     */
    public boolean isDisLimits(boolean isUpDirection){
        return isUpDirection ? getDis() >= -1*ShooterConstants.AngleChanger.MAX_DIS : getDis() <= ShooterConstants.AngleChanger.MIN_DIS;
    }

    public boolean isSupplyLimit(SHOOTER_MOTOR motor){
        return getSupplyCurrent(motor) >= 25;
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
        return !limitSwitch.get();
    }
    
    /**
     * get the needed angle for a dis
     * @param dis the current dis
     * @return the needed angle
     */
    public double getNeededAngle(double dis){
        return lookUpTable.get(dis)[1];
    }
    
    /**
     * get the needed pow for a dis 
     * @param dis the current dis
     * @return the needed pow
     */
    public double getNeededVel(double dis){
        return lookUpTable.get(dis)[2];
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
     * @return the wanted motor velocity
     * @exception FEEDING - that is a snowblower so return in pules per sec
     * @exception default - if the param motor is not listed above the function will return 0
     */
    public double getMotorVel(SHOOTER_MOTOR motor){
        switch (motor) {

            case UP:
                return motorUP.getSelectedSensorVelocity()*10/(ShooterConstants.PULES_PER_REV)* ShooterConstants.PEREMITER_OF_WHEEL;

            case DOWN:
                return motorDown.getSelectedSensorVelocity()*10/(ShooterConstants.PULES_PER_REV) * ShooterConstants.PEREMITER_OF_WHEEL;

            case FEEDING:
                return motorFeeding.getSelectedSensorVelocity()*10;

            case ANGLE:
                return motorAngle.getSelectedSensorVelocity()*10/((ShooterConstants.PULES_PER_REV * ShooterConstants.AngleChanger.GEAR_RATIO));
            
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
     * get the angle of the angle changer
     * @return the angle in degrees
     * @see also use the g(x) from desmos {@link https://www.desmos.com/calculator/4ja9zotx82}
     */
    public double getAngle(){
        double angle = (
        Math.acos(-1 * ((Math.pow(ShooterConstants.AngleChanger.KB, 2) - 
        Math.pow(ShooterConstants.AngleChanger.KA, 2) - 
        Math.pow(getDis(), 2)) / 
        (2 * ShooterConstants.AngleChanger.KA * getDis()))) * 
        180 / Math.PI
        );
        
        return angle;
    }

    public boolean isShooting() {
        return isShooting;
    }
    public void isShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }
   public boolean isShootingAmp() {
        return isShootingAmp;
    }
    public void isShootingAmp(boolean isShootingAmp) {
        this.isShootingAmp = isShootingAmp;
    }
  public boolean isShootingReady() {
        return isShootingReady;
    }
    public void isShootingReady(boolean isShootingReady) {
        this.isShootingReady = isShootingReady;
    }
    public boolean isActive() {
        return isActive;
    }
    public void isActive(boolean isActive) {
        this.isActive = isActive;
    }

    public boolean isActiveForSpeaker() {
        return isActive && !isShootingAmp;
    }

    public Command getShootCommand() {
        return new InstantCommand(()->isShooting(true)).andThen(new WaitCommand(0.5));
    }
    public Command getShootCommandWhenReady() {
        return new WaitUntilCommand(()->isShootingReady).andThen(getShootCommand());
    }

    public Command getActivateShooterToSpeaker() {
        return new InstantCommand(()->isShootingAmp(false)).andThen(new ActivateShooter(this,RobotContainer.robotContainer.intake, RobotContainer.robotContainer.chassis,false));
    }
    public Command getActivateShooterToAmp() {
        return new InstantCommand(()->isShootingAmp(true)).andThen(new ActivateShooter(this,RobotContainer.robotContainer.intake, RobotContainer.robotContainer.chassis,false));
    }

    public void shoot() {
        isShooting(true);
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

        /*put on ShuffleBoard all the builders */
        builder.addDoubleProperty("motor up speed", ()-> getMotorVel(SHOOTER_MOTOR.UP), null);
        builder.addDoubleProperty("motor down speed", ()-> getMotorVel(SHOOTER_MOTOR.DOWN), null);
        builder.addDoubleProperty("current amper motor up", ()-> getSupplyCurrent(SHOOTER_MOTOR.UP), null);
        builder.addDoubleProperty("current amper motor down", ()-> getSupplyCurrent(SHOOTER_MOTOR.DOWN), null);
        builder.addDoubleProperty("angle vel", ()-> getMotorVel(SHOOTER_MOTOR.ANGLE), null);
        builder.addDoubleProperty("Distance", this::getDis, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("encoder", ()->motorAngle.getSelectedSensorPosition(), null);
        builder.addBooleanProperty("Limit switch", ()->isLimit(), null);
        builder.addDoubleProperty("Analog get Volt", ()->getAnalogVolt(), null);
        builder.addBooleanProperty("is note", ()-> isNote(), null);
    
        /*put on ShuffleBoard all the cmds */
        // SmartDashboard.putData("Dis reset", new InstantCommand(()-> resetDis()).ignoringDisable(true));
        SmartDashboard.putData("motor up Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.UP)).ignoringDisable(true));
        SmartDashboard.putData("motor up Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.UP)).ignoringDisable(true));
        SmartDashboard.putData("motor down Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.DOWN)).ignoringDisable(true));
        SmartDashboard.putData("motor down Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.DOWN)).ignoringDisable(true));
        SmartDashboard.putData("motor feeding Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.FEEDING)).ignoringDisable(true));
        SmartDashboard.putData("motor feeding Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.FEEDING)).ignoringDisable(true));
        SmartDashboard.putData("motor angle Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.ANGLE)).ignoringDisable(true));
        SmartDashboard.putData("motor angle Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.ANGLE)).ignoringDisable(true));

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
