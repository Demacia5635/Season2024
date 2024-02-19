package frc.robot.subsystems.amp;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.amp.AmpConstants.*;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
import frc.robot.commands.amp.AmpIntake2;
public class Amp extends SubsystemBase{
    //public final Pigeon2 gyro;
    public final TalonFX m1;
    public final TalonSRX m2;
    public final CANSparkMax neo1;//small wheel
    public final CANSparkMax neo2;//big wheel
    public SparkMaxAnalogSensor limitInput;
    public int opticCount;
    public SimpleMotorFeedforward openFF;
    public SimpleMotorFeedforward closeFF;
    public DigitalInput magneticSensor;
    private double armPulseOffset;

    
    //ArmFeedforward ff = new ArmFeedforward(Parameters.ks1, Parameters.kg1, Parameters.kv1, Parameters.ka1);
    //SimpleMotorFeedforward ff2 = new SimpleMotorFeedforward(Parameters.ks2, Parameters.kv2, Parameters.ka2);
    public Amp(){
        
        //gyro = new Pigeon2(AmpDeviceID.GYRO);
        
        m1 = new TalonFX(AmpDeviceID.M1);
        m1.setInverted(true);
        m1.setNeutralMode(NeutralMode.Coast);
        m1.configFactoryDefault();

        magneticSensor = new DigitalInput(AmpConstants.AmpDeviceID.MAGNETIC_SENSOR_ID);       
        m2 = new TalonSRX(AmpDeviceID.M2);
        m2.setInverted(true);

        neo1 = new CANSparkMax(AmpDeviceID.NEO1, MotorType.kBrushless);
        neo1.getEncoder().setPosition(0);
        neo2 = new CANSparkMax(AmpDeviceID.NEO2, MotorType.kBrushless);
        neo2.getEncoder().setPosition(0);

        openFF = new SimpleMotorFeedforward(AmpConstants.armStatesParameters.openFF[0], AmpConstants.armStatesParameters.openFF[1], AmpConstants.armStatesParameters.openFF[2]);
        closeFF = new SimpleMotorFeedforward(AmpConstants.armStatesParameters.closeFF[0], AmpConstants.armStatesParameters.closeFF[1], AmpConstants.armStatesParameters.closeFF[2]);

        resetStartPulses();
        setNeosBrake();

        armPulseOffset = m1.getSelectedSensorPosition() - Math.toRadians(-50)*ConvertionParams.PULSE_PER_RAD;

        SmartDashboard.putData("intake amp", new AmpIntake2(this));
        SmartDashboard.putNumber("amp power", 0);
        SmartDashboard.putData("set power amp", new RunCommand(()->setNeosPower(0.5), this));


        limitInput = neo2.getAnalog(AnalogMode.kAbsolute);
        
        opticCount = 0;
        SmartDashboard.putData(this);

        SmartDashboard.putData("BrakeDiff", new InstantCommand(
            ()->this.setBrake(),this).ignoringDisable(true));
        SmartDashboard.putData("CoastDiff", new InstantCommand(
            ()->this.setCoast(),this).ignoringDisable(true));


            SmartDashboard.putData("Amp Move Sysid",
        (new Sysid(this::setPowerArm, this::getVelRadArm, 0.3, 0.7, this)).getCommand());
        SmartDashboard.putData("run snowBlower F",new InstantCommand(()->setPowerSnowblower(0.5)));
        SmartDashboard.putData("run snowBlower B",new InstantCommand(()->setPowerSnowblower(-0.2)));


    }

    public void resetStartPulses() {
        m1.setSelectedSensorPosition(0);
    }
    
    public void configDevices() {
        m1.config_kP(0, Parameters.KP1);
        m1.config_kI(0, Parameters.KI1);
        m1.config_kD(0, Parameters.KD1);
    }

    public void neosSetVel(double vel1, double vel2){
        neo1.set(vel1);
        neo2.set(vel2);
    }
      
    public void neoEncoderSet(double postion1, double postion2){
        neo1.getEncoder().setPosition(postion1);
        neo2.getEncoder().setPosition(postion2);

    }
      
    public void neoEncoderReset(){
        neoEncoderSet(0,0);
        neo1.setSmartCurrentLimit(25);
        neo2.setSmartCurrentLimit(25);
    }
      


    public void neosSetInverted(boolean isInvert) {

        neo1.setInverted(isInvert);
        neo2.setInverted(isInvert);

    }

    public double[] getNeosRev(){
        double[] neos = new double[2];
        neos[0] = neo1.getEncoder().getPosition()/ConvertionParams.NEO_PULES_PER_REV*ConvertionParams.NEO1GearRatio;
        neos[1] = neo2.getEncoder().getPosition()/ConvertionParams.NEO_PULES_PER_REV*ConvertionParams.NEO2GearRatio;
        return neos;
    }
    public void neoMoveByRev(double vel1 ,double vel2, double rev1, double rev2){
        double startPos1 = getNeosRev()[0];
        double startPos2 = getNeosRev()[1];

        while ((vel1!=0)&&(vel2!=0)){
            if(Math.abs(getNeosRev()[0]-startPos1+rev1)<=0.05)
                vel1 = 0;
            if(Math.abs(getNeosRev()[1]-startPos2+rev2)<=0.05)
                vel2 = 0;
            neosSetVel(vel1,vel2);
            
            System.out.println("another neo1"+(Math.abs(getNeosRev()[0]-startPos1+rev1))+"rev");
            System.out.println("another neo2"+(Math.abs(getNeosRev()[1]-startPos2+rev2))+"rev");
        }
    }

    public void setPowerSnowblower(double power) {
        m2.set(TalonSRXControlMode.PercentOutput, power);
    }
    public double getSnowblowerA(){
        return m2.getStatorCurrent();
    }
    public void runSnowblower(double pow){
        if (getSnowblowerA()>Parameters.ARM_BRAKE_MAX_A){
            setPowerSnowblower(0);
        } else{
            setPowerSnowblower(pow);
        }
    }


    public boolean getMagneticSensor() {
        return magneticSensor.get();
    }

    public void setPowerArm(double p1){
        m1.set(ControlMode.PercentOutput, p1);
    }
    public double getpower(){
        double pMotor = m1.getMotorOutputPercent();
        return pMotor;
    }

    /**
     * 
     * @return true if the arm is fully closed
     */
    public boolean isClose() {
        if(getMagneticSensor()){
            return false;
        }
        return true;
    }
    
    /**
     * 
     * @return true if the arm is fully open
     */
    public boolean isOpen() {
        if(getPoseByPulses()>= Math.toRadians(55) ){
            return true;
        }
        return false;
    }


    public double getLimitVolt(){
        return limitInput.getPosition();
    }
    public boolean didNotePass(){
        return getLimitVolt()<2.5;
    }
    public void resetOpticCounts(){
        opticCount = 0;
    }
    
    /**
     * 
     * @param last the last output of the didNotePass function 
     * @return true if the note is inside and false if not
     */
    public boolean isNoteThere(boolean last){
        if((didNotePass() == true)&&(last == false)){
            opticCount+=1;
        }
        if(opticCount%2==0){
            return false;
        }
        return true;
    }



    public boolean isNote() {
        return limitInput.getVoltage() <= Parameters.NOTE_VOLTAGE;
    }

    public void setBrake(){
        System.out.println("brake Diff");
        m1.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoast(){
        System.out.println("coast Diff");
        m1.setNeutralMode(NeutralMode.Coast);
    }
    public void stop(){
        m1.set(ControlMode.PercentOutput, 0);
    }

    public static double deadband(double value) {
        if (Math.abs(value)<Parameters.Deadband) {
            return 0;
        }
        return value;
    }

    public static Translation2d getStickLeft(CommandXboxController controller) {
            return new Translation2d(deadband(controller.getLeftX()), deadband(controller.getLeftY()));
    }

    public static Translation2d getStickRight(CommandXboxController controller) {
        return new Translation2d(deadband(controller.getRightX()), deadband(controller.getRightY()));
    }

    /**public double deg(){
        double fixedDeg = gyro.getYaw();
        if(fixedDeg>0){
            while (fixedDeg >=360){
                fixedDeg -= 360;
            }
        }else{
            while (fixedDeg <0){
                fixedDeg += 360;
            }
        }
        return fixedDeg;
    }
    public double fixedDeg(){
        return startDeg - deg() + 23.556;
    }
    public double getPoseRad(){  
        return fixedDeg()/360*Math.PI*2;
    }**/
    public double getPoseByPulses(){
        return (m1.getSelectedSensorPosition()-armPulseOffset)/ConvertionParams.PULSE_PER_RAD;
    }


    public double getNeoPoseByPulses(){
        return (neo1.getEncoder().getPosition());
    }


    /**public void velFFArm(double posRad, double velRad, double acceleRad) {
        double ff = acceleRad*Parameters.KA1 + velRad*Parameters.KV1 + Parameters.KG1*Math.sin(posRad) + Math.signum(velRad)*Parameters.KS1;
        double velMotor = (velRad/ConvertionParams.M1GearRatio)/100;
        m1.set(ControlMode.Velocity, velMotor, DemandType.ArbitraryFeedForward, ff);
    }**/

    public double getVelRadArm(){
        return (m1.getSelectedSensorVelocity()*10)/ConvertionParams.PULSE_PER_RAD;
    }

    public double getArmAngle() {
         return (m1.getSelectedSensorPosition())/ConvertionParams.PULSE_PER_RAD;
    }

    public void setArmVelocityOpen(double velRad) {
        m1.set(ControlMode.Velocity, velRad/10*AmpConstants.ConvertionParams.PULSE_PER_RAD,
        DemandType.ArbitraryFeedForward, openFF.calculate(velRad));
    }

     public void setArmVelocityClose(double velRad) {
        m1.set(ControlMode.Velocity, velRad/10*AmpConstants.ConvertionParams.PULSE_PER_RAD,
        DemandType.ArbitraryFeedForward, closeFF.calculate(velRad));
    }

    public int state;
    public double FF(double wantedAnglerVel){
        double rad = getPoseByPulses();
        
        if (wantedAnglerVel > 0){
            state = 0;
        } else {
           state = 1;
        }

        return (
            armStatesParameters.KS[state] + 
            wantedAnglerVel * armStatesParameters.KV[state] + 
            (wantedAnglerVel-getVelRadArm()) * armStatesParameters.KA[state] + 
            armStatesParameters.Kcos[state] * Math.cos(rad)
        );
    }

    public void setVel(double wantedAnglerVel){
        double ff = FF(wantedAnglerVel);
        System.out.println(" ff = " + ff);
        m1.set(ControlMode.Velocity, wantedAnglerVel*ConvertionParams.PULSE_PER_RAD/10, 
        DemandType.ArbitraryFeedForward, ff);
    }

    @Override
    public void periodic() {
        super.periodic();
        // SmartDashboard.putNumber("Motor1 Power", getpower());
        // SmartDashboard.putNumber("Motor1 Velocity", getVelRadArm());
        // //SmartDashboard.putNumber("Arm poseRad", getPoseRad());
        // SmartDashboard.putNumber("Start angle", startDeg);

        // SmartDashboard.putData("snow blow", new RunCommand(()->setPowerSnowblower(0.1)));
        // SmartDashboard.putData("arm power", new RunCommand(()->setPowerArm(0.5)));
        // SmartDashboard.putData("neo power", new RunCommand(()->setNeosPower(0.3)));




        SmartDashboard.putNumber("neo1 encoder",getNeosRev()[0]);
        SmartDashboard.putNumber("neo2 encoder",getNeosRev()[1]);

        SmartDashboard.putBoolean("Optic Limit switch state", didNotePass());
        SmartDashboard.putNumber("Optic Limit switch Voltage", getLimitVolt());
        SmartDashboard.putNumber("Get SnowBlower a",getSnowblowerA());
        double a = getSnowblowerA();
        if(a != 0)
            System.out.println(" current = " + a);

        SmartDashboard.putBoolean("Lower Limit switch state", isClose());
        SmartDashboard.putBoolean("Upper Limit switch state", isOpen());

    }

    public boolean isCriticalCurrent() {
        // TODO Auto-generated method stub
        return neo1.getOutputCurrent() >= Parameters.CRITICAL_CURRENT;
    }

    public void setNeosPower(double p1, double p2) {
        neo1.set(p1);
        neo2.set(p2);
    }

     public void setNeosPower(double p) {
        neo1.set(p);
        neo2.set(p);
    }

    public double getMotorCurrent() {
        // TODO Auto-generated method stub
        return neo1.getOutputCurrent();
    }

    public void setCoastArm() {
       m1.setNeutralMode(NeutralMode.Coast);
    }

    public void setBrakeArm() {
        // TODO Auto-generated method stub
         m1.setNeutralMode(NeutralMode.Brake);
    }

    public void setNeosBrake() {
        neo1.setIdleMode(IdleMode.kBrake);
        neo2.setIdleMode(IdleMode.kBrake);
    }

    public void setNeosCoast() {
        neo1.setIdleMode(IdleMode.kCoast);
        neo2.setIdleMode(IdleMode.kCoast);
    }

    

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        Command cmd = new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KCos}, this::setPowerArm,
         this::getVelRadArm, this::getPoseByPulses, null, 0.2, 0.35 ,3, 0.7,5, this).getCommandOneWay();
        SmartDashboard.putData("Amp SYSID", cmd);
        builder.addDoubleProperty("Arm Angle", this::getPoseByPulses, null);
    }
    
    
}