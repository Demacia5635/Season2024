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
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.amp.AmpConstants.*;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
public class Amp extends SubsystemBase{
    //public final Pigeon2 gyro;
    public final TalonFX m1;
    public final TalonSRX m2;
    public final CANSparkMax neo1;//small wheel
    public final CANSparkMax neo2;//big wheel
    public double startDeg;
    public AnalogInput limitInput;
    public int opticCount;
    
    //ArmFeedforward ff = new ArmFeedforward(Parameters.ks1, Parameters.kg1, Parameters.kv1, Parameters.ka1);
    //SimpleMotorFeedforward ff2 = new SimpleMotorFeedforward(Parameters.ks2, Parameters.kv2, Parameters.ka2);
    public Amp(){
        
        //gyro = new Pigeon2(AmpDeviceID.GYRO);
        
        m1 = new TalonFX(AmpDeviceID.M1);
        m1.setInverted(true);

        m1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, AmpDeviceID.M1);//lower limit switch
        m1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, AmpDeviceID.M1);//upper limit switch
        
        m2 = new TalonSRX(AmpDeviceID.M2);
        m2.setInverted(false);

        neo1 = new CANSparkMax(AmpDeviceID.NEO1, MotorType.kBrushless);
        neo1.getEncoder().setPosition(0);
        neo2 = new CANSparkMax(AmpDeviceID.NEO2, MotorType.kBrushless);
        neo2.getEncoder().setPosition(0);

        limitInput = new AnalogInput(AmpDeviceID.LIGHT_LIMIT);
        limitInput.setAccumulatorInitialValue(0);
        
        opticCount = 0;
        SmartDashboard.putData(this);

        SmartDashboard.putData("BrakeDiff", new InstantCommand(
            ()->this.setBrake(),this).ignoringDisable(true));
        SmartDashboard.putData("CoastDiff", new InstantCommand(
            ()->this.setCoast(),this).ignoringDisable(true));
    }
    
    public void configDevices() {
        m1.config_kP(0, Parameters.KP1);
        m1.config_kI(0, Parameters.KI1);
        m1.config_kD(0, Parameters.KD1);
    }

    public void startRad(double startDeg){
        this.startDeg = startDeg;
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
        return m2.getSupplyCurrent();
    }
    public void runSnowblower(double pow, double maxA){
        if (getSnowblowerA()>maxA){
            setPowerSnowblower(0);
        } else{
            setPowerSnowblower(pow);
        }
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
        return m1.isFwdLimitSwitchClosed() == 1;
    }
    
    /**
     * 
     * @return true if the arm is fully open
     */
    public boolean isOpen() {
        return m1.isRevLimitSwitchClosed() == 1;
    }


    public double getLimitVolt(){
        return limitInput.getVoltage();
    }
    public boolean didNotePass(){
        return getLimitVolt()<4.55;
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
    public double getPoseByPulses(double startPulses){
        return (m1.getSelectedSensorPosition()-startPulses)/ConvertionParams.MOTOR_PULSES_PER_ANGLE/360*2*Math.PI;
    }


    /**public void velFFArm(double posRad, double velRad, double acceleRad) {
        double ff = acceleRad*Parameters.KA1 + velRad*Parameters.KV1 + Parameters.KG1*Math.sin(posRad) + Math.signum(velRad)*Parameters.KS1;
        double velMotor = (velRad/ConvertionParams.M1GearRatio)/100;
        m1.set(ControlMode.Velocity, velMotor, DemandType.ArbitraryFeedForward, ff);
    }**/

    public double getVelRadArm(){
        return (m1.getSelectedSensorVelocity()*ConvertionParams.M1GearRatio/ConvertionParams.MOTOR_PULSES_PER_SPIN)*2*Math.PI;
    }

    public int state;
    public double FF(double wantedAnglerVel, double startPulses){
        double angle = getPoseByPulses(startPulses);
        double rad = Math.toRadians(angle);
        
        if (wantedAnglerVel > 0){
            if (angle <= 36){
                state = 0;
            } else {
                if(angle>=72){
                    state = 2;
                }
                state = 1;
            }
        } else {
            if (angle >= 55){
                state = 3;
            } else {
                state = 4;
            }
        }

        return (
            armStatesParameters.KS[state] + 
            wantedAnglerVel * armStatesParameters.KV[state] + 
            (wantedAnglerVel-getVelRadArm()) * armStatesParameters.KA[state] + 
            armStatesParameters.Kalpha[state] * angle + 
            armStatesParameters.Ksin[state] * Math.sin(rad) +   
            armStatesParameters.Kcos[state] * Math.cos(rad) + 
            armStatesParameters.Kcossin[state] * Math.cos(rad) * Math.sin(rad)
        );
    }

    public void setVel(double wantedAnglerVel, double startPulses){
        m1.set(ControlMode.Velocity, wantedAnglerVel*ConvertionParams.MOTOR_PULSES_PER_ANGLE/10, DemandType.ArbitraryFeedForward, FF(wantedAnglerVel, startPulses));
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Motor1 Power", getpower());
        SmartDashboard.putNumber("Motor1 Velocity", getVelRadArm());
        //SmartDashboard.putNumber("Arm poseRad", getPoseRad());
        SmartDashboard.putNumber("Start angle", startDeg);

        SmartDashboard.putNumber("SnowBlower ampere", getSnowblowerA());

        SmartDashboard.putNumber("neo1 encoder",getNeosRev()[0]);
        SmartDashboard.putNumber("neo2 encoder",getNeosRev()[0]);

        SmartDashboard.putBoolean("Optic Limit switch state", didNotePass());
        SmartDashboard.putNumber("Optic Limit switch Voltage", getLimitVolt());

        SmartDashboard.putBoolean("Lower Limit switch state", isClose());
        SmartDashboard.putBoolean("Upper Limit switch state", isOpen());

    }

    

    /**@Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        Command cmd = new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KCos}, this::setPowerArm,
         this::getVelRadArm, this::getPoseByPulses(startPulses), null, 0.12, 0.2 ,3, 0.5,0.5, this).getCommand();
        SmartDashboard.putData("Amp SYSID", cmd);
    }**/
    
    
}