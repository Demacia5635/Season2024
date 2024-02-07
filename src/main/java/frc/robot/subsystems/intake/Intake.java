package frc.robot.subsystems.intake;
import frc.robot.subsystems.intake.IntakeConstants;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
 
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.DispenseCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ShootCommand;
import frc.robot.subsystems.intake.IntakeConstants.*;
public class Intake extends SubsystemBase{
    public final TalonFX motor;
    public AnalogInput limitInput;
 
    //SimpleMotorFeedforward ff2 = new SimpleMotorFeedforward(Parameters.ks2, Parameters.kv2, Parameters.ka2);
    public Intake(){
        motor = new TalonFX(IntakeDeviceID.MOTOR);
        motor.setInverted(Parameters.IS_INVERTED);
 
        limitInput = new AnalogInput(IntakeDeviceID.LIGHT_LIMIT);
        limitInput.setAccumulatorInitialValue(0);
 
        SmartDashboard.putData(this);
 
        SmartDashboard.putData("Brake", new InstantCommand(
            ()->this.setBrake(),this).ignoringDisable(true));
        SmartDashboard.putData("Coast", new InstantCommand(
            ()->this.setCoast(),this).ignoringDisable(true));
    }
 
    public void configDevices() {
        motor.config_kP(0, Parameters.KP);
        motor.config_kI(0, Parameters.KI);
        motor.config_kD(0, Parameters.KD);
    }
 
    public double getpower(){
        double pMotor = motor.getMotorOutputPercent();
        return pMotor;
    }
 
    public double getLimitVolt(){
        return limitInput.getVoltage();
    }
 
    public boolean isNotePresent(){
        if (getLimitVolt() < Parameters.NOT_PRESENCE_VOLTAGE){
            return true;
        }
        return false;
    }
 
    public void setPower(double p1){
        motor.set(ControlMode.PercentOutput, p1);
    }

    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, velocity);
    }

    public double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    public TalonFX getIntakeMotor() {
        return motor;
    }

    public double getEncoderPos() {
        return motor.getSelectedSensorPosition();
    }
 
    public void setBrake(){
        System.out.println("brake");
        motor.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoast(){
        System.out.println("coast");
        motor.setNeutralMode(NeutralMode.Coast);
    }
    public void stop(){
        motor.set(ControlMode.PercentOutput, 0);
    }
 
    public static double deadband(double value) {
        if (Math.abs(value)<Parameters.DEADBAND) {
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
 
 
    public double getRadVelocity(){
        return (motor.getSelectedSensorVelocity()*ConvertionParams.MOTOR_GEAR_RATIO/ConvertionParams.MOTOR_PULSES_PER_SPIN)*2*Math.PI;
    }
 
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Motor Power", getpower());
        SmartDashboard.putNumber("Motor Current", getMotorCurrent());
        SmartDashboard.putNumber("Motor Velocity", getRadVelocity());
        SmartDashboard.putBoolean("Limit switch state", isNotePresent());
        SmartDashboard.putNumber("Limit switch voltage", getLimitVolt());
        
    }
 
 
    @Override
    public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    SmartDashboard.putNumber("power", 0);
    SmartDashboard.putData("set power", new RunCommand(()-> setPower(0.1), this));
    }

}