package frc.robot.subsystems.chassis;
import static frc.robot.Constants.AmpConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AmpConstants.*;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
import frc.robot.commands.chassis.JoyStickAmp;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class Amp extends SubsystemBase{
    public final Pigeon2 gyro;
    public final TalonFX m1;
    public CANSparkMax neo;
    public double startDeg;
    public AnalogTrigger lightLimit;

    //ArmFeedforward ff = new ArmFeedforward(Parameters.ks1, Parameters.kg1, Parameters.kv1, Parameters.ka1);
    //SimpleMotorFeedforward ff2 = new SimpleMotorFeedforward(Parameters.ks2, Parameters.kv2, Parameters.ka2);
    public Amp(){
        gyro = new Pigeon2(AmpDeviceID.GYRO);
        m1 = new TalonFX(AmpDeviceID.M1);
        m1.setInverted(true);

        neo = new CANSparkMax(AmpDeviceID.M2, MotorType.kBrushless);
        neo.getEncoder().setPosition(0);

        lightLimit = new AnalogTrigger(AmpDeviceID.LIGHT_LIMIT); //light limit switch port
        lightLimit.setLimitsVoltage(4.5, 4.8); // Sets the trigger to enable at a voltage of 4 volts, and disable at a value of 1.5 volts
        
        SmartDashboard.putData(this);

        SmartDashboard.putData("BrakeDiff", new InstantCommand(
            ()->this.setBrake(),this).ignoringDisable(true));
        SmartDashboard.putData("CoastDiff", new InstantCommand(
            ()->this.setCoast(),this).ignoringDisable(true));
    }

    public void configDevices() {
        m1.config_kP(0, Parameters.KP1);
        m1.config_kI(0, Parameters.ki1);
        m1.config_kD(0, Parameters.kd1);
    }

    public void startRad(double startDeg){
        this.startDeg = startDeg;
    }
    public void neoSetVel(double vel){
        neo.set(vel);
    }
      
    public void neoEncoderSet(double postion){
        neo.getEncoder().setPosition(postion);
    }
      
    public void neoEncoderReset(){
        neoEncoderSet(0);
    }
      
    public double getNeoRev(){
        return neo.getEncoder().getPosition()/ConvertionParams.NEO_PULES_PER_REV;
    }
    public void neoMoveByRev(double vel, double rev){
        double startPos = getNeoRev();
        while (Math.abs(getNeoRev()-startPos+rev)==1){
          neoSetVel(vel);
          System.out.println("another"+(Math.abs(getNeoRev()-startPos+rev))+"rev");
        }
    }
    
    public double getpower(){
        double pMotor = m1.getMotorOutputPercent();
        return pMotor;
    }


    public boolean getLimitRun() {
        return lightLimit.getTriggerState();
    }

    public void setPowers(double p1, double p2){
        m1.set(ControlMode.PercentOutput, p1);
        neoSetVel(p2);
    }
    public void setPowerArm(double p1){
        m1.set(ControlMode.PercentOutput, p1);
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
        if (Math.abs(value)<Parameters.deadband) {
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

    public double deg(){
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
    }

    public void velFFArm(double posRad, double velRad, double acceleRad) {
        double ff = acceleRad*Parameters.ka1 + velRad*Parameters.kv1 + Parameters.kg1*Math.sin(posRad) + Math.signum(velRad)*Parameters.ks1;
        double velMotor = (velRad/ConvertionParams.m1GearRatio)/100;
        m1.set(ControlMode.Velocity, velMotor, DemandType.ArbitraryFeedForward, ff);
    }

    public double getVelRadArm(){
        return (m1.getSelectedSensorVelocity()*ConvertionParams.m1GearRatio/ConvertionParams.MOTOR_PULSES_PER_SPIN)*2*Math.PI;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Motor1 Power", getpower());
        SmartDashboard.putNumber("Motor1 Velocity", getVelRadArm());
        SmartDashboard.putNumber("Arm poseRad", getPoseRad());
        SmartDashboard.putNumber("neon encoder",getNeoRev());
        SmartDashboard.putNumber("start angle", startDeg);
        SmartDashboard.putBoolean("Limit switch state", getLimitRun());
    }

    

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        Command cmd = new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KCos}, this::setPowerArm,
         this::getVelRadArm, this::getPoseRad, null, 0.12, 0.2 ,3, 0.5,0.5, this).getCommand();
        SmartDashboard.putData("Amp SYSID", cmd);
    }
    
    
}
