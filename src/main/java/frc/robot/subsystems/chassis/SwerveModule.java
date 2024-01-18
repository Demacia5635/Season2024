package frc.robot.subsystems.chassis;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Sysid.FeedForward_SVA;
import frc.robot.Sysid.Sysid;
import frc.robot.subsystems.chassis.Constants.SwerveModuleConstants;
import frc.robot.utils.Trapezoid;

import static frc.robot.Constants.ChassisConstants.*;
import static frc.robot.Constants.ChassisConstants.SwerveModuleConstants.MAX_STEER_ERROR;
import static frc.robot.subsystems.chassis.Constants.MOTOR_PULSES_PER_ROTATION;

public class SwerveModule implements Sendable {
    private final TalonFX moveMotor;
    private final TalonFX steerMotor;
    private final CANCoder absoluteEncoder;

    private final double angleOffset;
    private double pulsePerDegree;
    private double pulsePerMeter;

    private FeedForward_SVA moveFF;
    private FeedForward_SVA steerFF;
    private Trapezoid steerTrapezoid;

    double targetVelocity = 0;
    Rotation2d targetAngle = new Rotation2d();
    public String name;

    public boolean debug = false;

    Chassis chassis;

    public SwerveModule(SwerveModuleConstants constants, Chassis chassis) {
        this.chassis = chassis;
        moveMotor = new TalonFX(constants.moveMotorId);
        steerMotor = new TalonFX(constants.angleMotorId);
        moveMotor.configFactoryDefault();
        steerMotor.configFactoryDefault();
        absoluteEncoder = new CANCoder(constants.absoluteEncoderId);
        moveFF = new FeedForward_SVA(constants.moveFF.KS, constants.moveFF.KV, constants.moveFF.KA);
        steerFF = new FeedForward_SVA(constants.steerFF.KS, constants.steerFF.KV, constants.steerFF.KA);
        setMovePID(constants.movePID.KP, constants.movePID.KI, constants.movePID.KD);
        setAnglePID(constants.steerPID.KP, constants.steerPID.KI, constants.steerPID.KD);
        steerMotor.setInverted(false);
        pulsePerDegree = constants.pulsePerDegree;
        pulsePerMeter = constants.pulsePerMeter;
        steerTrapezoid = new Trapezoid(MAX_STEER_VELOCITY, STEER_ACCELERATION);
        angleOffset = constants.steerOffset;
        name = (constants.moduleTranslationOffset.getX()>0?"Front":"Back") + 
               (constants.moduleTranslationOffset.getY()>0?"Left":"Right");
        debug = constants.moduleTranslationOffset.getX() < 0 && constants.moduleTranslationOffset.getY() >0;
        
        moveMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setNeutralMode(NeutralMode.Brake);
        moveMotor.setInverted(true);
        steerMotor.setInverted(constants.inverted);
        moveMotor.setSelectedSensorPosition(0);
        System.out.println(" angle degrees = " + getAngleDegrees());
        steerMotor.setSelectedSensorPosition(getAngleDegrees()*pulsePerDegree);
        steerMotor.configClosedloopRamp(0.5);
        steerMotor.configOpenloopRamp(0.5);
        double kp = 0.1*1023/20/pulsePerDegree; // correct 0.1  power at 20 degrees error
        double ki = kp/10;
        System.out.println(" kp = " + kp + " ki = " + ki);

        setAnglePID(kp,ki,0);

        System.out.println(" selected sensor " + steerMotor.getSelectedSensorPosition());
        SmartDashboard.putData(name + " Steer Sysid", (new Sysid(this::setSteerPower, this::getSteerVelocity, 0.15, 0.5, chassis)).getCommand());
    }

    public void setMovePID(double kP, double kI, double kD) {
        moveMotor.config_kP(0, kP);
        moveMotor.config_kI(0, kI);
        moveMotor.config_kD(0, kD);
    }

    public void setAnglePID(double kP, double kI, double kD) {
        steerMotor.config_kP(0, kP);
        steerMotor.config_kI(0, kI);
        steerMotor.config_kD(0, kD);
    }

    public void setInverted(boolean invert) {
        moveMotor.setInverted(invert);
    }


    public double getAbsoluteEncoder() {
        return absoluteEncoder.getAbsolutePosition();
    }



    public double steerTalonAngle() {
        return MathUtil.inputModulus(steerMotor.getSelectedSensorPosition()/pulsePerDegree,-180,180);
    }
    public double steerTalonRawAngle() {
        return steerMotor.getSelectedSensorPosition()*360/MOTOR_PULSES_PER_ROTATION;
    }

    /**
     * Returns the velocity of the module drive motor
     * @return Velocity in m/s
     */
    public double getVelocity() {
        return encoderToMetricSpeed(moveMotor.getSelectedSensorVelocity());
    }

    /**
     * Stops the module completely
     */
    public void stop() {
        setPower(0);
        setSteerPower(0);
    }

    /**
     * Sets the neutral mode of both motors
     */
    public void setNeutralMode(NeutralMode mode) {
        moveMotor.setNeutralMode(mode);
        steerMotor.setNeutralMode(mode);
    }

    /**
     * Sets the velocity of the module
     * @param v Velocity in m/s
     */

    double lastMoveA = 0;
    double lastMoveV = 0;

    public void setVelocity(double v) {
        targetVelocity = v;
        if(Math.abs(v) < 0.03) {
            setPower(0);
            return;
        }
        targetVelocity = v;
        double currentVelocity = getVelocity();
        double cv = currentVelocity;
        if(lastMoveA > 0 && currentVelocity < lastMoveV && currentVelocity < v) {
            currentVelocity = lastMoveV;
        } else if(lastMoveA < 0 && currentVelocity > lastMoveV && currentVelocity < v) {
            currentVelocity = lastMoveV;
        }
        double tgtV = v;
        double maxAccel = DRIVE_ACCELERATION * Constants.CYCLE_DT;
        if (v > currentVelocity + maxAccel) {
            tgtV = currentVelocity + maxAccel;
        } else if (v < currentVelocity-maxAccel) {
            tgtV = currentVelocity - maxAccel;
        }
        double ff = moveFF.calculate(tgtV, cv);
        moveMotor.set(ControlMode.Velocity, metricToEncoderSpeed(tgtV), DemandType.ArbitraryFeedForward, ff);
        lastMoveA = tgtV - currentVelocity;
        lastMoveV = tgtV;
    }

     /**
     * Sets the power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setPower(double p) {
        moveMotor.set(ControlMode.PercentOutput, p);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition() - angleOffset);
    }
    public double getAngleDegrees() {
        double d = getAngle().getDegrees();
        return MathUtil.inputModulus(d,-180,180);
    }

    /**
     * Sets the rotational power of the module
     * @param p Power in precent of the module (0%..100%)
     */
    public void setSteerPower(double p) {
        steerMotor.set(ControlMode.PercentOutput, p);
    }

    /**
     * Returns the rotational velocity of the module
     * @return Velocity in deg/s
     */
    public double getSteerVelocity() {  
        return absoluteEncoder.getVelocity(); // encoderToAngularSpeed(steerMotor.getSelectedSensorVelocity());
    }

    /**
     * Sets the angular velocity of the module
     * @param v Velocity in deg/s
     */
    public void setSteerVelocity(double v, boolean withAcceleration) {
        double tgtV = v;
        double currentVelocity = getSteerVelocity();
        if(withAcceleration) {
            double maxVChange = STEER_ACCELERATION * Constants.CYCLE_DT;
            tgtV = Math.min(tgtV,currentVelocity + maxVChange);
            tgtV = Math.max(tgtV,currentVelocity - maxVChange);
        }
        double ff = steerFF.calculate(tgtV,currentVelocity);
        if(debug) System.out.println(" set v to " + tgtV + " ff=" + ff);
        steerMotor.set(ControlMode.Velocity, angularToEncoderSpeed(tgtV), DemandType.ArbitraryFeedForward, ff);
    }

    public void setAngle(Rotation2d angle) {
        targetAngle = angle;
        double diff = MathUtil.inputModulus(angle.minus(getAngle()).getDegrees(),-180,180);
        if(debug) System.out.println(" set angle - " + angle.getDegrees() + " diff=" + diff + " cur=" + getAngleDegrees());
        if(debug) System.out.println(" error=" + steerMotor.getClosedLoopError() + " power=" + steerMotor.getMotorOutputPercent());
        double cpos = steerMotor.getSelectedSensorPosition();
        double pos = cpos + diff*pulsePerDegree;
        if(debug) System.out.println(" pos=" + pos + " cpos=" + cpos);
        steerMotor.set(ControlMode.Position,pos);

        /* 
        double v = 0;
        if(Math.abs(error) > MAX_STEER_ERROR) {
            double cv = getSteerVelocity();
            v = steerTrapezoid.calculate(error, cv, 0, debug);
            if(debug) System.out.println(name + " error=" + error + " v=" + v + " cv=" + cv + " angle=" + getAngleDegrees() + "/" + 
                steerTalonAngle());
        }
        setSteerVelocity(v, true);
        */
    }

    /**
     * Returns the state of the module
     * @return Velocity in m/s
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Sets the state of the module
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState optimized = SwerveModuleState.optimize(state, getAngle());
        setVelocity(optimized.speedMetersPerSecond);
        setAngle(optimized.angle);
    }

    /**
     * Returns the module position
     * @return Position relative to the field
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(moveMotor.getSelectedSensorPosition() / pulsePerMeter, getAngle());
    }

    public double metricToEncoderSpeed(double speed) {
        return speed * pulsePerMeter / 10;
    }

    public double encoderToMetricSpeed(double speed) {
            return speed / pulsePerMeter * 10;
    }

    public double angularToEncoderSpeed(double speed) {
            return speed * pulsePerDegree / 10;
    }
    
    public double encoderToAngularSpeed(double speed) {
            return speed / pulsePerDegree * 10;
    }

    public double getDistance() {
        return moveMotor.getSelectedSensorPosition()/pulsePerMeter;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("angle", () -> getAngleDegrees(), null);
        builder.addDoubleProperty("steer error", () -> steerMotor.getClosedLoopError(), null);
        builder.addDoubleProperty("velocity", this::getVelocity, null);
        builder.addDoubleProperty("Steer velocity", this::getSteerVelocity, null);
        builder.addDoubleProperty("desired angle", () -> targetAngle.getDegrees(), null);
        builder.addDoubleProperty("desired velocity", () -> targetVelocity, null);
        builder.addDoubleProperty("Steer Talon Angle", this::steerTalonAngle, null);
        builder.addDoubleProperty("Steer power", ()->steerMotor.getMotorOutputPercent(), null);
        builder.addDoubleProperty("distance", this::getDistance, null);
        builder.addDoubleProperty("Steer Power",()->steerMotor.getMotorOutputPercent(), null);
    }
    

}