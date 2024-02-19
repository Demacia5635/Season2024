package frc.robot.subsystems.amp;

import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.ARM_DOWN_POWER;
import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.CLOSE_ANGLE;
import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.INTAKE_RELEASE_POWER;
import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.MAX_ARM_ACCEL_UP;
import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.MAX_ARM_VEL_UP;
import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.SENSEOR_ANGLE;
import static frc.robot.subsystems.amp.AmpConstantsUdi.Parameters.UP_ANGLE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
import frc.robot.commands.amp.AmpIntake2Udi;
import frc.robot.subsystems.amp.AmpConstantsUdi.AmpDeviceID;
import frc.robot.subsystems.amp.AmpConstantsUdi.ConvertionParams;
import frc.robot.subsystems.amp.AmpConstantsUdi.Parameters;
import frc.robot.subsystems.amp.AmpConstantsUdi.armStatesParameters;
import frc.robot.utils.TrapezoidCalc;

public class AmpUdi extends SubsystemBase {

    private final TalonFX armMotor;
    private final TalonSRX lockMotor;
    private final CANSparkMax smallWheelMotor;
    private final CANSparkMax bigWheelMotor;
    private SparkMaxAnalogSensor noteSensor;
    private int opticCount;
    private DigitalInput armSensor;
    private boolean isLocked = false;
    private boolean isLocking = true;
    private boolean isUnLocking = false;
    private double lockingStartTime = 0;
    private boolean releaseReady = false;
    private double targetAngle = Parameters.CLOSE_ANGLE;
    private TrapezoidCalc armUpTrapezoid = new TrapezoidCalc();

    // ArmFeedforward ff = new ArmFeedforward(Parameters.ks1, Parameters.kg1,
    // Parameters.kv1, Parameters.ka1);
    // SimpleMotorFeedforward ff2 = new SimpleMotorFeedforward(Parameters.ks2,
    // Parameters.kv2, Parameters.ka2);
    public AmpUdi() {
        armMotor = new TalonFX(AmpDeviceID.ArmMotorID);
        armMotor.configFactoryDefault();
        armMotor.setInverted(true);
        armMotor.setNeutralMode(NeutralMode.Coast);
        armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.5));
        armMotor.configOpenloopRamp(0.5);
        setArmAngle(Parameters.CLOSE_ANGLE);
        configArmPIDF();

        armSensor = new DigitalInput(AmpDeviceID.ArmPositionSensorID);
        lockMotor = new TalonSRX(AmpDeviceID.LockMotorID);
        lockMotor.configFactoryDefault();
        lockMotor.setInverted(false);
        lockMotor.setNeutralMode(NeutralMode.Brake);

        smallWheelMotor = new CANSparkMax(AmpDeviceID.SmallWheelMotorID, MotorType.kBrushless);
        bigWheelMotor = new CANSparkMax(AmpDeviceID.BigWheelMotorID, MotorType.kBrushless);
        setWheelsBrake();
        wheelsEncoderReset();
        smallWheelMotor.setSmartCurrentLimit(25);
        bigWheelMotor.setSmartCurrentLimit(25);

        SmartDashboard.putData("intake amp", getIntakeCommand());
        SmartDashboard.putNumber("amp power", 0);
        SmartDashboard.putData("set amp wheels power", new InstantCommand(() -> setWheelsPower(0.5), this));
        SmartDashboard.putData("stop amp wheels power", new InstantCommand(() -> setWheelsPower(0), this));

        noteSensor = bigWheelMotor.getAnalog(AnalogMode.kAbsolute);

        opticCount = 0;
        SmartDashboard.putData(this);

        SmartDashboard.putData("Arm Brake", new InstantCommand(
                () -> this.setArmBrake(), this).ignoringDisable(true));
        SmartDashboard.putData("Arm Coast", new InstantCommand(
                () -> this.setArmCoast(), this).ignoringDisable(true));

        Sysid sid = new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KCos }, this::setArmPower,
                this::getArmRadPerSec, null, null, 0.15, 0.4, 3, 1.0, 3.0, this);
        SmartDashboard.putData("Amp Move Sysid", sid.getCommandOneWay());
    }

    public void setArmAngle(double radians) {
        armMotor.setSelectedSensorPosition(radians * ConvertionParams.PULSE_PER_RAD);
    }

    public void configArmPIDF() {
        armMotor.config_kP(0, Parameters.KP1);
        armMotor.config_kI(0, Parameters.KI1);
        armMotor.config_kD(0, Parameters.KD1);
    }

    public void wheelsSetVel(double vel1, double vel2) {
        smallWheelMotor.set(vel1);
        bigWheelMotor.set(vel2);
    }

    public void wheelsEncoderSet(double postion1, double postion2) {
        smallWheelMotor.getEncoder().setPosition(postion1);
        bigWheelMotor.getEncoder().setPosition(postion2);

    }

    public void wheelsEncoderReset() {
        wheelsEncoderSet(0, 0);
    }

    public void wheelsSetInverted(boolean isInvert) {
        smallWheelMotor.setInverted(isInvert);
        bigWheelMotor.setInverted(isInvert);
    }

    public double[] getWheelsRev() {
        double[] neos = new double[2];
        neos[0] = smallWheelMotor.getEncoder().getPosition() / ConvertionParams.NEO_PULES_PER_REV
                * ConvertionParams.SmallWheelGearRatio;
        neos[1] = bigWheelMotor.getEncoder().getPosition() / ConvertionParams.NEO_PULES_PER_REV
                * ConvertionParams.BigWheelGearRatio;
        return neos;
    }

    public void setLockPower(double power) {
        lockMotor.set(TalonSRXControlMode.PercentOutput, power);
    }

    public double getLockMotorCurrent() {
        return lockMotor.getSupplyCurrent();
    }

    public boolean getArmSensor() {
        return armSensor.get();
    }

    public void setArmPower(double p) {
        armMotor.set(ControlMode.PercentOutput, p);
    }

    public double getpower() {
        return armMotor.getMotorOutputPercent();
    }

    /**
     * 
     * @return true if the arm is fully closed
     */
    public boolean isAtSensor() {
        return getArmSensor();
    }

    /**
     * 
     * @return true if the arm is fully open
     */
    public boolean isOpen() {
        return getArmAngle() >= Parameters.UP_ANGLE;
    }

    public double getNoteSensorVolt() {
        return noteSensor.getPosition();
    }

    public boolean isSensingNote() {
        return getNoteSensorVolt() < Parameters.NOTE_VOLTAGE;
    }

    public void resetOpticCounts() {
        opticCount = 0;
    }

    /**
     * 
     * @param last the last sensing
     * @return true if the note is inside and false if not
     */
    public boolean isNoteInside(boolean last) {
        if (isSensingNote() && (!last)) {
            opticCount += 1;
        }
        if (opticCount % 2 == 0) {
            return false;
        }
        return true;
    }

    public void setArmBrake() {
        armMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setArmCoast() {
        armMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void stopArm() {
        armMotor.set(ControlMode.PercentOutput, 0);
    }

    public static double deadband(double value) {
        return Math.abs(value) < Parameters.Deadband ? 0 : value;
    }

    public double getArmRadPerSec() {
        return (armMotor.getSelectedSensorVelocity() * 10) / ConvertionParams.PULSE_PER_RAD;
    }

    public double getArmAngle() {
        return (armMotor.getSelectedSensorPosition()) / ConvertionParams.PULSE_PER_RAD;
    }

    public double FF(double wantedAnglerVel) {
        int state = wantedAnglerVel > 0 ? 0 : 1;
        double angle = getArmAngle();
        return (armStatesParameters.KS[state] +
                wantedAnglerVel * armStatesParameters.KV[state] +
                (wantedAnglerVel - getArmRadPerSec()) * armStatesParameters.KA[state] +
                armStatesParameters.Kcos[state] * Math.cos(angle));
    }

    public void setArmVel(double wantedAnglerVel) {
        armMotor.set(ControlMode.Velocity, wantedAnglerVel * ConvertionParams.PULSE_PER_RAD / 10,
                DemandType.ArbitraryFeedForward, FF(wantedAnglerVel));
    }

    public boolean isCriticalCurrent() {
        return smallWheelMotor.getOutputCurrent() >= Parameters.CRITICAL_CURRENT;
    }

    public void setWheelsPower(double p1, double p2) {
        smallWheelMotor.set(p1);
        bigWheelMotor.set(p2);
    }

    public void setWheelsPower(double p) {
        smallWheelMotor.set(p);
        bigWheelMotor.set(p);
    }

    public double getSmallWheelsPosition() {
        return (smallWheelMotor.getEncoder().getPosition());
    }

    public double getSmallWheelsMotorCurrent() {
        return smallWheelMotor.getOutputCurrent();
    }

    public void setWheelsBrake() {
        smallWheelMotor.setIdleMode(IdleMode.kBrake);
        bigWheelMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setWheelsCoast() {
        smallWheelMotor.setIdleMode(IdleMode.kCoast);
        bigWheelMotor.setIdleMode(IdleMode.kCoast);
    }

    public void stopAll() {
        setWheelsPower(0);
        setArmTargetAngle(CLOSE_ANGLE);
    }

    // Locking
    public boolean isLocked() {
        return isLocked;
    }

    public boolean isUnlocked() {
        return !isLocked;
    }

    public void lock() {
        if (!isLocked || isUnLocking) {
            isLocked = false;
            isLocking = true;
            isUnLocking = false;
            lockingStartTime = Timer.getFPGATimestamp();
            setLockPower(Parameters.LOCK_POWER);
        }
    }

    public void unlock() {
        if (isLocked || isUnLocking) {
            isLocking = false;
            isUnLocking = true;
            isLocked = true;
            lockingStartTime = Timer.getFPGATimestamp();
            setLockPower(Parameters.UNLOCK_POWER);
        }
    }

    private void lockPeriodic() {
        if (Robot.robot.isDisabled()) {
            return;
        }
        if (isLocking) {
            if (getLockMotorCurrent() > Parameters.LOCK_CURRENT) {
                setLockPower(0);
                isLocked = true;
                isLocking = false;
                isUnLocking = false;
            } else {
                setLockPower(Parameters.LOCK_POWER);
            }
        } else if (isUnLocking) {
            if (Timer.getFPGATimestamp() > lockingStartTime + Parameters.UNLOCK_TIME) {
                setLockPower(0);
                isLocked = false;
                isLocking = false;
                isUnLocking = false;
            }
        }
    }

    // arm periodic
    public void setArmTargetAngle(double radians) {
        targetAngle = radians;
    }

    public boolean isArmAtPosition() {
        double currentAngle = getArmAngle();
        double error = targetAngle - currentAngle;
        if (Math.abs(error) < Parameters.ARM_RAD_ERROR) {
            return targetAngle == SENSEOR_ANGLE || isLocked;
        }
        return false;
    }

    private void armPeriodic() {
        if (isAtSensor()) {
            setArmAngle(Parameters.SENSEOR_ANGLE);
        }
        if (Robot.robot.isEnabled()) {
            double currentAngle = getArmAngle();
            double error = targetAngle - currentAngle;
            if (Math.abs(error) > Parameters.ARM_RAD_ERROR) {
                if (isLocked()) {
                    unlock();
                } else if (error > 0) { // up
                    double vel = armUpTrapezoid.trapezoid(getArmRadPerSec(), MAX_ARM_VEL_UP, 0, MAX_ARM_ACCEL_UP,
                            error);
                    setArmVel(vel);
                } else {
                    setArmPower(ARM_DOWN_POWER);
                }
            } else if (!isLocked && targetAngle != SENSEOR_ANGLE) {
                lock();
            }
        }
    }

    private void doRelease() {
        if (targetAngle == UP_ANGLE && isArmAtPosition()) {
            getDoReleaseCommand().schedule();
        }
    }

    // commands

    // to bind the ready button - use the getIntakeCommand.alongWith(intake command
    // to transfer)).andThen(getReadyPositionCommand)
    public Command getIntakeCommand() {
        return new AmpIntake2Udi(this);
    }

    public Command getReadyPositionCommand() {
        return new InstantCommand(() -> setArmTargetAngle(UP_ANGLE), this)
                .andThen(new WaitUntilCommand(this::isArmAtPosition));
    }

    // to bind the shooting button to getReleaseCommand along with shooter shoot
    // command
    // this will do the release only if ready
    public Command getReleaseCommand() {
        return new InstantCommand(() -> doRelease(), this);
    }

    private Command getDoReleaseCommand() {
        return new InstantCommand(() -> setWheelsPower(INTAKE_RELEASE_POWER), this).andThen(new WaitCommand(0.7),
                new InstantCommand(() -> setWheelsPower(0), this));
    }

    // command to return to home position
    public Command getToHomeCommand() {
        return new InstantCommand(() -> setArmTargetAngle(CLOSE_ANGLE), this)
                .andThen(new WaitUntilCommand(this::isArmAtPosition));
    }

    // command to cancel all and return to home position
    public Command getCancelCommand() {
        return new InstantCommand(() -> stopAll(), this);
    }

    @Override
    public void periodic() {
        super.periodic();
        lockPeriodic();
        armPeriodic();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Lock Current", this::getLockMotorCurrent, null);
        builder.addDoubleProperty("Wheel Encoder", () -> getWheelsRev()[0], null);
        builder.addBooleanProperty("Amp Note Sensor", this::isSensingNote, null);
        builder.addDoubleProperty("Amp Note Volt", this::getNoteSensorVolt, null);
        builder.addBooleanProperty("Amp At Sensor", this::isAtSensor, null);
    }

}