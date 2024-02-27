// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.subsystems.shooter.utils.LookUpTable;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

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

    /**is the shooter shoot */
    private boolean isShooting  = false;
    
    /**is the shooter is shooting to the amp */
    public boolean isShootingAmp = false;

    /**is the shooter is ready to shoot */
    public boolean isShootingReady = false;

    /**is the shooter is active */
    public boolean isActive = false;
    public boolean isShootingFrom = false;

    /** Calibrate */
    public boolean inCalibration = false;
    public double calibrateAngle = 0;
    public double calibrateVelocity = 0;
    
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

        /*cofig motor Up */
        motorUP = new TalonFX(ShooterID.MOTOR_UP_ID);
        motorUP.configFactoryDefault();
        motorUP.setInverted(true);
        motorUP.config_kP(0, Shooting.KP);

        /*config motor Down */
        motorDown = new TalonFX(ShooterID.MOTOR_DOWN_ID);
        motorDown.configFactoryDefault();
        motorDown.config_kP(0, Shooting.KP);

        /*config feeding motor */
        motorFeeding = new TalonSRX(ShooterID.MOTOR_FEEDING_ID);
        motorFeeding.setInverted(true);
        motorFeeding.enableCurrentLimit(true);
        motorFeeding.configContinuousCurrentLimit(20);
        
        /*config angle Motor */
        motorAngle = new TalonFX(ShooterID.MOTOR_ANGLE_ID);
        motorAngle.configFactoryDefault();
        motorAngle.config_kP(0, AngleChanger.KP);
        motorAngle.config_kD(0, AngleChanger.KD);
        motorAngle.configMotionCruiseVelocity(20000);
        motorAngle.configMotionAcceleration(20000);
        
        /*config analog input */
        analogInput = new AnalogInput(ShooterID.ANALOG_INPUT_ID);
        analogInput.setAccumulatorInitialValue(0);

        /*config the feedforward for the angle changer */
        elevationFF = new ArmFeedforward(AngleChanger.KS, AngleChanger.KG, AngleChanger.KV);

        /*cofig limit switch */
        limitSwitch = new DigitalInput(ShooterID.LIMIT_SWITCH_ID);

        /*cofig lookup table */
        lookUpTable  = new LookUpTable(LookUpTableVar.lookUpTable);

        /*put all the motors at brake */
        brake(SHOOTER_MOTOR.UP, SHOOTER_MOTOR.DOWN, SHOOTER_MOTOR.FEEDING, SHOOTER_MOTOR.ANGLE);
        
        /*put all the init sendable into the smart dashboard */
        SmartDashboard.putData(this);
    }
    
    /**
     * @deprecated
     * creates trapezoid algo 
     * @param dis the wanted dis in mm on the screw (absolute)
     */
    public void angleMotionMagic(double dis) {
        motorAngle.set(ControlMode.MotionMagic, (AngleChanger.PULES_PER_MM * (dis + (-1 * AngleChanger.MAX_DIS))));
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
        pow*=0.6;
        motorAngle.set(ControlMode.PercentOutput, pow);
    }

    /**
     * stop the angle motor 
     */
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
    
    /**
     * get the need feed forward for the shooting motors
     * @param vel the wanted velocity
     * @return the needed feedforward
     */
    public double getFF(double vel){
        return (Shooting.KS * Math.signum(vel)+
                Shooting.KV * vel +
                Shooting.KV2 * Math.pow(vel, 2));
    }
    
    /**
     * give velocity to the shooting motors
     * @param velUp the wanted velocity for the up motor
     * @param velDown the wanted velocity for the down motor
     */
    public void setVel(double velUp, double velDown) {
        double ffUp = getFF(velUp);
        double wantedVelUp = (velUp/ 10) / ShooterVar.PEREMITER_OF_WHEEL * ShooterVar.PULES_PER_REV;
        motorUP.set(ControlMode.Velocity, wantedVelUp, DemandType.ArbitraryFeedForward, ffUp);
        double ffDown = getFF(velDown);
        double wantedVelDown = (velDown/ 10) / ShooterVar.PEREMITER_OF_WHEEL * ShooterVar.PULES_PER_REV;
        motorDown.set(ControlMode.Velocity, wantedVelDown, DemandType.ArbitraryFeedForward, ffDown);
        
    }

    /**
     * give the same velocity for both shooting motors
     * @param vel the wanted velocity for both motors
     */
    public void setVel(double vel){
        setVel(vel,vel);
    }

    /**
     * stops the shooting motors
     */
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

    /**
     * stop the feeding motor 
     */
    public void feedingStop(){
        motorFeeding.set(ControlMode.PercentOutput, 0);
    }

    /**
     * important for saftey function that will stop all the motors in this subsystem 
     */
    public void stopAll(){
        stop();
        angleStop();
        feedingStop();
        isActive(false);
        isShooting(false);
        isShootingAmp(false);
        isShootingReady(false);
        isShootingFrom = false;
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
    
    /**
     * reset the base dis of the angle motor also reset the encoder of the angle motor 
     */
    public void resetDis(){
        motorAngle.setSelectedSensorPosition(0);
    }

    /**
     * set isShooting to true
     */
    public void shoot() {
        isShooting(true);
    }

    /**
     * set the isShooting var
     * @param isShooting what isShooting will be
     */
    public void isShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }

    /**
     * set isShootingAmp var
     * @param isShootingAmp what isShootingAmp will be
     */
    public void isShootingAmp(boolean isShootingAmp) {
        this.isShootingAmp = isShootingAmp;
    }

    /**
     * set isShootingReady var
     * @param isShootingReady what isShootingReady will be
     */
    public void isShootingReady(boolean isShootingReady) {
        this.isShootingReady = isShootingReady;
    }

    /**
     * set isActive var
     * @param isActive what isActiveReady will be
     */
    public void isActive(boolean isActive) {
        this.isActive = isActive;
    }


    /**
     * set isShooting to true and then wait for 0.5 sec
     * @return acommand that set shoter to true and wait 0.5 sec
     */
    public Command shootCommand() {
        return new InstantCommand(()-> shoot()).andThen(new WaitCommand(0.5)
            .raceWith(RobotContainer.robotContainer.intake.getActivateIntakeCommand()));
    }

    /**
     * wait until shooter is ready and then shoot the shooter
     * @return a command that will wait until the shooter is ready to shoot
     */
    public Command shootCommandWhenReady() {
        return new WaitUntilCommand(()->isShootingReady).andThen(shootCommand());
    }

    /**
     * active shooter to the speaker
     * @return a command that will set isShootingToAmp false and activate the shooter to shoot at the amp
     */
    public Command activateShooterToSpeaker() {
        return new InstantCommand(()->isShootingAmp(false)).alongWith(new ActivateShooter(this,RobotContainer.robotContainer.intake, RobotContainer.robotContainer.chassis,false));
    }
    public Command activateShooterToSpeakerFromSub() {
        return new InstantCommand(()->isShootingAmp(false)).
        alongWith(new ActivateShooter(this,RobotContainer.robotContainer.intake,
                             RobotContainer.robotContainer.chassis,1.35,false));
    }

    /**
     * active the shooter to the amp
     * @return a command that will set isShootingToAmp true and activate the shooter to shoot at the amp
     */
    public Command activateShooterToAmp() {
        return new InstantCommand(()->isShootingAmp(true)).andThen(new ActivateShooter(this,RobotContainer.robotContainer.intake, RobotContainer.robotContainer.chassis,false));
    }

    /**
     * check if the shooter is active to the speakerk
     * @return isActive and not isShootingAmp
     */
    public boolean isActiveToSpeaker() {
        return isActive && !isShootingAmp && !isShootingFrom  && !inCalibration;
    }

    /**
     * get the distance based on the angle
     * @param wantedAngle the wanted angle to calcalate
     * @return the dis on the screw of the angle changer
     */
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
        return motorAngle.getSelectedSensorPosition() / AngleChanger.PULES_PER_MM + AngleChanger.MAX_DIS;
    }

    /**
     * for saftey checks if the angle motor passed its limits
     * @param isUpDirection if the velocity the motor moves is positive or negative
     * @return if the limits have passed (false means you are fine)
     */
    public boolean isDisLimits(boolean isUpDirection){
        return isUpDirection ? getDis() >= AngleChanger.MAX_DIS - 4 : getDis() <= AngleChanger.MIN_DIS+4;
    }

    /**
     * for saftye checks if the angle motor passed its limtis
     * but without direction so it will stop even if goint to the right direction
     * @return if the angle motor is at the limit (false means you are fine)
     */
    public boolean isDisLimits(){
        return isDisLimits(true) || isDisLimits(false);
    }

    /**
     * for saftey if the motor is at too much amper
     * @param motor the motor that being checked
     * @return true if the motor at the supply limits
     */
    public boolean isSupplyLimit(SHOOTER_MOTOR motor){
        return getSupplyCurrent(motor) >= 25;
    }
    
    /**
     * check if the not had pass
     * @return if the limit volt is smaller than 4.55
     * @author Adar
     */
    public boolean isNote(){
        return getAnalogVolt() < ShooterVar.VOLT_NOTE_PRESENT;
    }

    /**
     * get if the angle is at the end
     * @return if the limit switch is pressed
     */
    public boolean isLimit(){
        return !limitSwitch.get();
    }
    
    /**
     * get the needed angle for a dis
     * @param dis the current dis
     * @return the needed angle
     */
    public double getNeededAngle(double dis){
        return lookUpTable.get(dis)[0];
    }
    
    /**
     * get the needed pow for a dis 
     * @param dis the current dis
     * @return the needed pow
     */
    public double getNeededVel(double dis){
        return lookUpTable.get(dis)[1];
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
     * @exception default - if the param motor is not listed above the function will return 0
     */
    public double getMotorVel(SHOOTER_MOTOR motor){
        switch (motor) {

            case UP:
                return motorUP.getSelectedSensorVelocity()*10/(ShooterVar.PULES_PER_REV)* ShooterVar.PEREMITER_OF_WHEEL;

            case DOWN:
                return motorDown.getSelectedSensorVelocity()*10/(ShooterVar.PULES_PER_REV) * ShooterVar.PEREMITER_OF_WHEEL;

            case FEEDING:
                return motorFeeding.getSelectedSensorVelocity()*10;

            case ANGLE:
                return motorAngle.getSelectedSensorVelocity()*10/((ShooterVar.PULES_PER_REV * AngleChanger.GEAR_RATIO));
            
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
        Math.acos(-1 * ((Math.pow(AngleChanger.KB, 2) - 
        Math.pow(AngleChanger.KA, 2) - 
        Math.pow(getDis(), 2)) / 
        (2 * AngleChanger.KA * getDis()))) * 
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

    /**
     * put var to the shuffle board
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        /*put on the shuffleBoard all the builders */
        builder.addDoubleProperty("motor up speed", ()-> getMotorVel(SHOOTER_MOTOR.UP), null);
        builder.addDoubleProperty("motor down speed", ()-> getMotorVel(SHOOTER_MOTOR.DOWN), null);
        builder.addDoubleProperty("current amper motor up", ()-> getSupplyCurrent(SHOOTER_MOTOR.UP), null);
        builder.addDoubleProperty("current amper motor down", ()-> getSupplyCurrent(SHOOTER_MOTOR.DOWN), null);
        builder.addDoubleProperty("angle vel", ()-> getMotorVel(SHOOTER_MOTOR.ANGLE), null);
        builder.addDoubleProperty("Distance", this::getDis, null);
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addBooleanProperty("Limit switch", ()->isLimit(), null);
        builder.addDoubleProperty("Analog get Volt", ()->getAnalogVolt(), null);
        builder.addBooleanProperty("is note", ()-> isNote(), null);
        builder.addBooleanProperty("Is shooting", ()-> isShooting, null);
        builder.addBooleanProperty("Is shooting to amp", ()-> isShootingAmp, null);
        builder.addBooleanProperty("Is shooting ready", ()-> isShootingReady, null);
        builder.addBooleanProperty("Is shooter active", ()-> isActive, null);
        builder.addBooleanProperty("Is active to speaker", this::isActiveToSpeaker, null);

        builder.addBooleanProperty("Calibrate",this::inCalibration, this::inCalibration);
        builder.addDoubleProperty("calibrate Angle", this::calibrateAngle, this::calibrateAngle);
        builder.addDoubleProperty("calibrate Velocity", this::calibrateVelocity, this::calibrateVelocity);
        
        /*put on the shuffleBoard all the commands */
        SmartDashboard.putData("Dis reset", new InstantCommand(()-> resetDis()).ignoringDisable(true));
        SmartDashboard.putData("motor up Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.UP)).ignoringDisable(true));
        SmartDashboard.putData("motor up Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.UP)).ignoringDisable(true));
        SmartDashboard.putData("motor down Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.DOWN)).ignoringDisable(true));
        SmartDashboard.putData("motor down Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.DOWN)).ignoringDisable(true));
        SmartDashboard.putData("motor feeding Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.FEEDING)).ignoringDisable(true));
        SmartDashboard.putData("motor feeding Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.FEEDING)).ignoringDisable(true));
        SmartDashboard.putData("motor angle Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.ANGLE)).ignoringDisable(true));
        SmartDashboard.putData("motor angle Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.ANGLE)).ignoringDisable(true));

    }

    /** Calibrate */
    boolean inCalibration() {
        return inCalibration;
    }
    void inCalibration(boolean inCalibration) {
        this.inCalibration = inCalibration;
    }
    double calibrateAngle() {
        return calibrateAngle;
    }
    void calibrateAngle(double calibrateAngle) {
        this.calibrateAngle = calibrateAngle;
    }
    double calibrateVelocity() {
        return calibrateVelocity;
    }
    void calibrateVelocity(double calibrateVelocity) {
        this.calibrateVelocity = calibrateVelocity;
    }

    public boolean isShootingReady() {
        return isShootingReady;
    }
    /** if the angle is at the end it reset the dis */
    @Override
    public void periodic() {
        super.periodic();
        
        if (isLimit()){
            resetDis();
        }
    }

    public boolean getIsShooting() {
        return isShooting;
    }

    public void setIsShooting(boolean is) {
        isShooting = is;
    }
}
