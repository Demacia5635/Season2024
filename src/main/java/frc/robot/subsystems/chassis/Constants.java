package frc.robot.subsystems.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  public static final int GYRO_ID = 15;

  public static final double CYCLE_DT = 0.02;

  public static final double MAX_DRIVE_VELOCITY = 3;
  public static final double DRIVE_ACCELERATION = 8;
  public static final double MAX_STEER_VELOCITY = 600;
  public static final double STEER_ACCELERATION = 6000;
  public static final double MAX_STEER_ERROR = 5;
  public static final double MAX_OMEGA_VELOCITY = Math.toRadians(180);
  public static final double MAX_OMEGA_ACCELERATION = Math.toRadians(720);

  // Pulse per meter/degrees
  public static final double WHEEL_DIAMETER = 4 * 0.0254; // 4 inch
  public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  public static final double MOVE_GEAR_RATIO = 8.14;
  public static final double MOTOR_PULSES_PER_ROTATION = 2048;
  public static final double PULSES_PER_METER = MOTOR_PULSES_PER_ROTATION * MOVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE;

  public static final double BACK_STEER_RATIO = 150.0 / 7.0;
  public static final double FRONT_STEER_RATIO = 12.8;

  public static final double BACK_PULSES_PER_DEGREE = BACK_STEER_RATIO * MOTOR_PULSES_PER_ROTATION / 360.0;
  public static final double FRONT_PULSES_PER_DEGREE = FRONT_STEER_RATIO * MOTOR_PULSES_PER_ROTATION / 360.0;

  // PID
  public static final PID_Constants MOVE_PID = new PID_Constants(0.05, 0, 0);
  public static final PID_Constants FRONT_STEER_PID = new PID_Constants(0.2, 0, 0);
  public static final PID_Constants BACK_STEER_PID = new PID_Constants(0.2, 0, 0);
  // Feed Forward Gains
  public static final FF_Constants MOVE_FF = new FF_Constants(0.05, 0.263, 0.1);
  public static final FF_Constants FRONT_STEER_FF = new FF_Constants(0.05, 0.0006, 0.01);
  public static final FF_Constants BACK_STEER_FF = new FF_Constants(0.05, 0.006, 0.01);

  public final static SwerveModuleConstants FRONT_LEFT = new SwerveModuleConstants(
      3, 4, 12,
      new Translation2d(0.332, 0.277),
      257.607421875,
      MOVE_PID,
      FRONT_STEER_PID,
      MOVE_FF,
      FRONT_STEER_FF,
      PULSES_PER_METER,
      FRONT_PULSES_PER_DEGREE,
      false);
  public final static SwerveModuleConstants FRONT_RIGHT = new SwerveModuleConstants(
      5, 6, 13,
      new Translation2d(0.332, -0.277),
      290.7,
      MOVE_PID,
      FRONT_STEER_PID,
      MOVE_FF,
      FRONT_STEER_FF,
      PULSES_PER_METER,
      FRONT_PULSES_PER_DEGREE,
      false);

  public final static SwerveModuleConstants BACK_LEFT = new SwerveModuleConstants(
      2, 1, 11,
      new Translation2d(-0.332, 0.288),
      230,
      MOVE_PID,
      BACK_STEER_PID,
      MOVE_FF,
      BACK_STEER_FF,
      PULSES_PER_METER,
      BACK_PULSES_PER_DEGREE,
      false);

  public final static SwerveModuleConstants BACK_RIGHT = new SwerveModuleConstants(
      8, 7, 14,
      new Translation2d(-0.332, -0.288),
      276.1,
      MOVE_PID,
      BACK_STEER_PID,
      MOVE_FF,
      BACK_STEER_FF,
      PULSES_PER_METER,
      BACK_PULSES_PER_DEGREE,
      false);

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      FRONT_LEFT.moduleTranslationOffset,
      FRONT_RIGHT.moduleTranslationOffset,
      BACK_LEFT.moduleTranslationOffset,
      BACK_RIGHT.moduleTranslationOffset);

  public static class PID_Constants {
    public final double KP, KI, KD;

    PID_Constants(double KP, double KI, double KD) {
      this.KP = KP;
      this.KI = KI;
      this.KD = KD;
    }
  }

  public static class FF_Constants {
    public final double KS, KV, KA;

    FF_Constants(double KS, double KV, double KA) {
      this.KS = KS;
      this.KV = KV;
      this.KA = KA;
    }
  }

  public static class SwerveModuleConstants {
    public final int moveMotorId;
    public final int angleMotorId;
    public final int absoluteEncoderId;
    public final Translation2d moduleTranslationOffset;
    public final double steerOffset;
    public final PID_Constants movePID;
    public final PID_Constants steerPID;
    public final FF_Constants moveFF;
    public final FF_Constants steerFF;
    public final double pulsePerMeter;
    public final double pulsePerDegree;
    public final boolean inverted;

    public SwerveModuleConstants(int moveMotorId, int angleMotorId, int absoluteEncoderId,
        Translation2d moduleTranslationOffset, double steerOffset,
        PID_Constants movePID, PID_Constants steerPID, FF_Constants moveFF, FF_Constants steerFF,
        double pulsePerMeter, double pulsePerDegree, boolean inverted) {
      this.moveMotorId = moveMotorId;
      this.angleMotorId = angleMotorId;
      this.absoluteEncoderId = absoluteEncoderId;
      this.moduleTranslationOffset = moduleTranslationOffset;
      this.steerOffset = steerOffset;
      this.movePID = movePID;
      this.moveFF = moveFF;
      this.steerFF = steerFF;
      this.steerPID = steerPID;
      this.pulsePerDegree = pulsePerDegree;
      this.pulsePerMeter = pulsePerMeter;
      this.inverted = inverted;
    }
  }
}
