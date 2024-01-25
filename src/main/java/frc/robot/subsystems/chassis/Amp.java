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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AmpConstants.*;
import frc.robot.commands.chassis.JoyStickAmp;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
public class Amp extends SubsystemBase{
    public final Pigeon2 gyro;
    public final TalonFX m1;
    public CANSparkMax neon;
    public double startDeg;

    //ArmFeedforward ff = new ArmFeedforward(Parameters.ks1, Parameters.kg1, Parameters.kv1, Parameters.ka1);
    //SimpleMotorFeedforward ff2 = new SimpleMotorFeedforward(Parameters.ks2, Parameters.kv2, Parameters.ka2);
    public Amp(){
        gyro = new Pigeon2(AmpDeviceID.gyro);
        m1 = new TalonFX(AmpDeviceID.m1);
        neon = new CANSparkMax(AmpDeviceID.m2, MotorType.kBrushless);
        neon.getEncoder().setPosition(0);
        m1.setInverted(false);
        //m2.setInverted(false);

        SmartDashboard.putData(this);

        SmartDashboard.putData("BrakeDiff", new InstantCommand(
            ()->this.setBrake(),this).ignoringDisable(true));
        SmartDashboard.putData("CoastDiff", new InstantCommand(
            ()->this.setCoast(),this).ignoringDisable(true));
    }

    public void configDevices() {
        m1.config_kP(0, Parameters.kp1);
        m1.config_kI(0, Parameters.ki1);
        m1.config_kD(0, Parameters.kd1);
    }

    public void startDeg(double startDeg){
        this.startDeg = startDeg;
    }
    public void neonSetVel(double vel){
        neon.set(vel);
      }
      
      public void neonEncoderSet(double postion){
        neon.getEncoder().setPosition(postion);
      }
      
      public void neonEncoderReset(){
        neonEncoderSet(0);
      }
      
      public double getNeonRev(){
        return neon.getEncoder().getPosition()/ConvertionParams.NEON_PULES_PER_REV;
      }
      public void neonMoveByRev(double vel, double rev){
        double startPos = getNeonRev();
        while (Math.abs(getNeonRev()-startPos+rev)==1){
          neonSetVel(vel);
          System.out.println("another"+(Math.abs(getNeonRev()-startPos+rev))+"rev");
        }
      }
    
    public double getpower(){
        double pMotor = m1.getMotorOutputPercent();
        return pMotor;
    }
    public void setPowers(double p1, double p2){
        m1.set(ControlMode.PercentOutput, p1);
        neonSetVel(p2);
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
        return startDeg - deg();
    }
    public double getPoseRad(){  
        return fixedDeg()/360*Math.PI*2;
    }

    public void velFFArm(double posRad, double velRad, double acceleRad) {
        double ff = acceleRad*Parameters.ka1 + velRad*Parameters.kv1 + Parameters.kg1*Math.sin(posRad) + Math.signum(velRad)*Parameters.ks1;
        double velMotor = (velRad/ConvertionParams.m1GearRatio)/100;
        m1.set(ControlMode.Velocity, velMotor, DemandType.ArbitraryFeedForward, ff);
    }

    public double getVelArm(){
        return m1.getSelectedSensorVelocity()*100*ConvertionParams.m1GearRatio;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Motor1 Power", getpower());
        SmartDashboard.putNumber("Motor1 Velocity", getVelArm());
        SmartDashboard.putNumber("Pitch in radiansS", fixedDeg());
        SmartDashboard.putNumber("neon encoder",getNeonRev());
        SmartDashboard.putNumber("start angle", startDeg);
    }
}
