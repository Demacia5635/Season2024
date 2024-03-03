// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotContainer;
import frc.robot.Sysid.Sysid;
import frc.robot.commands.shooter.ActivateShooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

/**subsystem shooter and angle changing */
public class Shooter extends SubsystemBase {

    /**the motor that controls the angle of the shooter */
    final TalonFX motorAngle;

    /**the motors that move the rollers */
    final TalonFX motorUP; 
    /**the motors that move the rollers */
    final TalonFX motorDown;
    
    /**the motor that feeds the notes to the shooter */
    final TalonSRX motorFeeding;

    /**the sensor that detect if there is a note in the shooter */
    AnalogInput analogInput;
    
    /**the limit switch on the angle changer machanism */
    DigitalInput limitSwitch;

    /**is the shooter shoot */
    private boolean isShooting  = false;
    
    /**is the shooter is shooting to the amp */
    private boolean isShootingAmp = false;
    
    /**is the shooter is shooting from the podium */
    private boolean isShootingPodium = false;
    private boolean isShootingClose = false;

    /**is the shooter is ready to shoot */
    private boolean isShootingReady = false;

    /**is the shooter is active */
    private boolean isActive = false;

    /**is Shooting from diffrent place than chassis location */
    private boolean isShootingFrom = false;

    /**is the shooter is in calibration */
    private boolean isInCalibration = false;
    
    /**the calibration angle */
    private double calibrateAngle = 0;

    /**the calibration velocity */
    private double calibrateVel = 0;
    
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

        /*cofig limit switch */
        limitSwitch = new DigitalInput(ShooterID.LIMIT_SWITCH_ID);

        /*put all the motors at brake */
        brake(SHOOTER_MOTOR.UP, SHOOTER_MOTOR.DOWN, SHOOTER_MOTOR.FEEDING, SHOOTER_MOTOR.ANGLE);
        
        /*put all the init sendable and useful commands into the smart dashboard */
        SmartDashboard.putNumber("wanted vel", 0);
        SmartDashboard.putData("set vel shooter", new RunCommand(()->setVel(SmartDashboard.getNumber("wanted vel", 0)), this));
        SmartDashboard.putData(this);
        SmartDashboard.putData("Shooter Move Sysid",
        (new Sysid(this::setPow, ()-> getMotorVel(SHOOTER_MOTOR.UP), 0.2, 0.8, this)).getCommand());
    }

    /**
     * set the pow of the amgle motor
     * @param pow the wanted pow in -1 to 1
     */
    public void angleSetPow(double pow) {
        pow*=0.6;
        motorAngle.set(ControlMode.PercentOutput, pow);
    }

    /**
     * stop the angle motor 
     */
    public void angleStop() {
        motorAngle.set(ControlMode.PercentOutput, 0);
    }

    /**
     * set the pow of the shooters motors
     * @param pow the wanted pow from -1 to 1
     */
    public void setPow(double pow) {
        motorUP.set(ControlMode.PercentOutput, pow);
        motorDown.set(ControlMode.PercentOutput, pow);
    }
    
    /**
     * get the need feed forward for the shooting motors
     * @param vel the wanted velocity
     * @return the needed feedforward
     */
    public double getFF(double vel) {
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

        /*set vel for the up motor */
        double ffUp = getFF(velUp);
        double wantedVelUp = (velUp/ 10) / ShooterVar.PEREMITER_OF_WHEEL * ShooterVar.PULES_PER_REV;
        motorUP.set(ControlMode.Velocity, wantedVelUp, DemandType.ArbitraryFeedForward, ffUp);
        
        /*set vel for the down motor */
        double ffDown = getFF(velDown);
        double wantedVelDown = (velDown/ 10) / ShooterVar.PEREMITER_OF_WHEEL * ShooterVar.PULES_PER_REV;
        motorDown.set(ControlMode.Velocity, wantedVelDown, DemandType.ArbitraryFeedForward, ffDown);
    }

    /**
     * give the same velocity for both shooting motors
     * @param vel the wanted velocity for both motors
     */
    public void setVel(double vel) {
        setVel(vel,vel);
    }

    /**
     * stops the shooting motors
     */
    public void stop() {
        motorUP.set(ControlMode.PercentOutput, 0);
        motorDown.set(ControlMode.PercentOutput, 0);
    }
    
    /**
     * set the pow of the motor that feeding the shooters motors
     * @param pow the wanted pow from -1 to 1
     */
    public void feedingSetPow(double pow) {
        motorFeeding.set(ControlMode.PercentOutput, pow);
    }

    /**
     * stop the feeding motor 
     */
    public void feedingStop() {
        motorFeeding.set(ControlMode.PercentOutput, 0);
    }

    /**
     * important for saftey function that will stop all the motors in this subsystem 
     */
    public void stopAll() {
        stop();
        angleStop();
        feedingStop();
        setIsActive(false);
        setIsShooting(false);
        setIsShootingAmp(false);
        setIsShootingReady(false);
        setIsShootingFrom(false);
        setIsInCalibration(false);
    }

    /**
     * set brake mode to motors
     * @param motor the wanted motors
     */
    public void brake(SHOOTER_MOTOR... motor) { 
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
    public void coast(SHOOTER_MOTOR... motor) { 
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
    public void resetDis() {
        motorAngle.setSelectedSensorPosition(0);
    }

    /**
     * set isShooting to true
     */
    public void shoot() {
        setIsShooting(true);
    }

    /**
     * set the isShooting var
     * @param isShooting what isShooting will be
     */
    public void setIsShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }

    /**
     * set the isShootingAmp var
     * @param isShootingAmp what isShootingAmp will be
     */
    public void setIsShootingAmp(boolean isShootingAmp) {
        this.isShootingAmp = isShootingAmp;
    }

    /**
     * set the isShootingPodium var
     * @param isShootingPodium what isShootingPodium will be
     */
    public void setIsShootingPodium(boolean isShootingPodium) {
        isShootingAmp = false;
        isShootingClose = false;
        this.isShootingPodium = isShootingPodium;
    }

    /**
     * set isShootingReady var
     * @param isShootingReady what isShootingReady will be
     */
    public void setIsShootingReady(boolean isShootingReady) {
        this.isShootingReady = isShootingReady;
    }

    /**
     * set isActive var
     * @param isActive what isActiveReady will be
     */
    public void setIsActive(boolean isActive) {
        this.isActive = isActive;
    }

    /**
     * set isShootingFrom var
     * @param isShootingFrom what isShootingFrom will be
     */
    public void setIsShootingFrom(boolean isShootingFrom) {
        this.isShootingFrom = isShootingFrom;
    }

    /**
     * set isIncalibration var
     * @param isInCalibration what isInCalibration will be
     */
    public void setIsInCalibration(boolean isInCalibration) {
        this.isInCalibration = isInCalibration;
    }

    /**
     * set calibration angle
     * @param calibrateAngle what calibrateAngle will be
     */
    public void setCalibrateAngle(double calibrateAngle) {
        this.calibrateAngle = calibrateAngle;
    }

    /**
     * set calibration vel
     * @param calibrateVel what calibrateVel will be
     */
    public void setCalibrateVel(double calibrateVel) {
        this.calibrateVel = calibrateVel;
    }

    /**
     * get the command that will activate the shooter to shoot from the podium
     * @return a command that will shoot from the podium
     */
    public Command getActivateShooterToPodium() {
        return new InstantCommand(()->setIsShootingPodium(true)).andThen(new ActivateShooter(this,RobotContainer.robotContainer.intake, RobotContainer.robotContainer.chassis,false));
    }

    /**
     * set isShooting to true and then wait for 0.5 sec
     * @return acommand that set shoter to true and wait 0.5 sec
     */
    public Command getShootCommand() {
        return new InstantCommand(()-> shoot()).andThen(new WaitCommand(2)
            .raceWith(RobotContainer.robotContainer.intake.getActivateIntakeCommand()));
    }

    /**
     * wait until shooter is ready and then shoot the shooter
     * @return a command that will wait until the shooter is ready to shoot
     */
    public Command getShootCommandWhenReady() {
        return new WaitUntilCommand(()->isShootingReady).andThen(getShootCommand());
    }

    /**
     * activate shooter to the speaker
     * @return a command that will set isShootingToAmp false and activate the shooter to shoot at the amp
     */
    public Command getActivateShooterToSpeaker() {
        return (new InstantCommand((()->{setIsShootingAmp(false); setIsShootingPodium(false);}))).alongWith(new ActivateShooter(this,RobotContainer.robotContainer.intake, RobotContainer.robotContainer.chassis,false));
    }

    /**
     * activate the shooter to shoot to the speaker from the sub offer
     * @return a command that will make the shooter shoot from the sub offer
     */
    public Command getActivateShooterToSpeakerFromSub() {
        Command c = new InstantCommand(()->isShootingClose(true)).
        alongWith(new ActivateShooter(this,RobotContainer.robotContainer.intake,
                             RobotContainer.robotContainer.chassis,1.35,false));
        c.setName("activate from sub");
        return c;
    }

    /**
     * activate the shooter to the amp
     * @return a command that will set isShootingToAmp true and activate the shooter to shoot at the amp
     */
    public Command getActivateShooterToAmp() {
        return new InstantCommand(()->setIsShootingAmp(true)).andThen(new ActivateShooter(this,RobotContainer.robotContainer.intake, RobotContainer.robotContainer.chassis,false));
    }

    /**
     * get isShooting
     * @return isShooting
     */
    public boolean getIsShooting() {
        return isShooting;
    }

    /**
     * get isShootingAmp
     * @return isShootingAmp
     */
    public boolean getIsShootingAmp() {
        return isShootingAmp;
    }

    /**
     * get isShootingPodium
     * @return isShootingPodium
     */
    public boolean getIsShootingPodium() {
        return isShootingPodium;
    }

    public boolean isShootingClose() {
        return isShootingClose;
    }
    public void isShootingClose(boolean isShootingClose) {
        isShootingAmp = false;
        isShootingPodium = false;
        this.isShootingClose = isShootingClose;
    }
    

    /**
     * get isShootingReady
     * @return isShootingReady
     */
    public boolean getIsShootingReady() {
        return isShootingReady;
    }

    /**
     * get isActive
     * @return isActive
     */
    public boolean getIsActive() {
        return isActive;
    }

    /**
     * get isShootingFrom
     * @return isShootingFrom
     */
    public boolean getIsShootingFrom() {
        return isShootingFrom;
    }

    /**
     * get isInCalibration
     * @return isInCalibration
     */
    public boolean getIsInCalibration() {
        return isInCalibration;
    }

    /**
     * get calibrateAngle
     * @return calibrateAngle
     */
    public double getCalibrateAngle() {
        return calibrateAngle;
    }

    /**
     * get calibrateVel
     * @return calibrateVel
     */
    public double getCalibrateVel() {
        return calibrateVel;
    }

    /**
     * check if the shooter is active to the speakerk
     * @return isActive and not isShootingAmp
     */
    public boolean getIsActiveToSpeaker() {
        return isActive && !isShootingAmp && !isShootingFrom  && !isInCalibration;
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
    public double getDis() {
        return motorAngle.getSelectedSensorPosition() / AngleChanger.PULES_PER_MM + AngleChanger.MAX_DIS;
    }

    /**
     * for saftey checks if the angle motor passed its limits
     * @param isUpDirection if the velocity the motor moves is positive or negative
     * @return if the limits have passed (false means you are fine)
     */
    public boolean isDisLimits(boolean isUpDirection) {
        return isUpDirection ? 
        getDis() >= AngleChanger.MAX_DIS - AngleChanger.BOUNDARY_DIS : 
        getDis() <= AngleChanger.MIN_DIS + AngleChanger.BOUNDARY_DIS ;
    }

    /**
     * for saftey if the motor is at too much amper
     * @param motor the motor that being checked
     * @return true if the motor at the supply limits
     */
    public boolean isSupplyLimit(SHOOTER_MOTOR motor) {
        return getSupplyCurrent(motor) >= 25;
    }
    
    /**
     * check if the not had pass
     * @return if the limit volt is smaller than 4.55
     * @author Adar
     */
    public boolean isNote() {
        return getAnalogVolt() < ShooterVar.VOLT_NOTE_PRESENT;
    }

    /**
     * get if the angle is at the end
     * @return if the limit switch is pressed
     */
    public boolean isLimit() {
        return !limitSwitch.get();
    }

    /**
     * get the limit input voltage
     * @return the limit input voltage
     * @author Adar
     */ 
    public double getAnalogVolt() {
        return analogInput.getVoltage();
    }

    /**
     * get the vel of every motor
     * @param motor the wanted motor 
     * @return the wanted motor velocity
     * @exception default - if the param motor is not listed above the function will return 0
     */
    public double getMotorVel(SHOOTER_MOTOR motor) {
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
    public double getSupplyCurrent(SHOOTER_MOTOR motor) {
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
    public double getAngle() {
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
        builder.addBooleanProperty("Limit switch", this::isLimit, null);
        builder.addDoubleProperty("Analog get Volt", this::getAnalogVolt, null);
        builder.addBooleanProperty("is note", this::isNote, null);
        builder.addBooleanProperty("Is active to speaker", this::getIsActiveToSpeaker, null);

        /*put shooter var on shuffleboard and making them able to be changeble */
        builder.addBooleanProperty("is shooting", this::getIsShooting, this::setIsShooting);
        builder.addBooleanProperty("is shooting amp", this::getIsShootingAmp, this::setIsShootingAmp);
        builder.addBooleanProperty("is shooting podium", this::getIsShootingPodium, this::setIsShootingPodium);
        builder.addBooleanProperty("is shooting ready", this::getIsShootingReady, this::setIsShootingReady);
        builder.addBooleanProperty("is active", this::getIsActive, this::setIsActive);
        builder.addBooleanProperty("is shooting from", this::getIsShootingFrom, this::setIsShootingFrom);
        builder.addBooleanProperty("Calibrate",this::getIsInCalibration, this::setIsInCalibration);
        builder.addDoubleProperty("calibrate Angle", this::getCalibrateAngle, this::setCalibrateAngle);
        builder.addDoubleProperty("calibrate Velocity", this::getCalibrateVel, this::setCalibrateVel);
        
        /*put on the shuffleBoard all the commands */
        SmartDashboard.putData("Dis reset", new InstantCommand(this::resetDis).ignoringDisable(true));
        SmartDashboard.putData("motor up Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.UP)).ignoringDisable(true));
        SmartDashboard.putData("motor up Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.UP)).ignoringDisable(true));
        SmartDashboard.putData("motor down Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.DOWN)).ignoringDisable(true));
        SmartDashboard.putData("motor down Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.DOWN)).ignoringDisable(true));
        SmartDashboard.putData("motor feeding Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.FEEDING)).ignoringDisable(true));
        SmartDashboard.putData("motor feeding Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.FEEDING)).ignoringDisable(true));
        SmartDashboard.putData("motor angle Brake", new InstantCommand(()-> brake(SHOOTER_MOTOR.ANGLE)).ignoringDisable(true));
        SmartDashboard.putData("motor angle Coast", new InstantCommand(()-> coast(SHOOTER_MOTOR.ANGLE)).ignoringDisable(true));

    }

    /** 
     * if the angle is at the end it reset the dis 
     */
    @Override
    public void periodic() {
        super.periodic();
        
        if (isLimit()){
            resetDis();
        }
    }
}
