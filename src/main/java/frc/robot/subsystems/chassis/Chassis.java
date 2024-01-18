package frc.robot.subsystems.chassis;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sysid.Sysid;
import frc.robot.commands.chassis.utils.TestVelocity;

import static frc.robot.subsystems.chassis.Constants.*;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;

public class Chassis extends SubsystemBase {
  private final SwerveModule[] modules;
  private final Pigeon2 gyro;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field;

  public Chassis() {
    modules = new SwerveModule[] {
      new SwerveModule(FRONT_LEFT, this),
      new SwerveModule(FRONT_RIGHT, this),
      new SwerveModule(BACK_LEFT, this),
      new SwerveModule(BACK_RIGHT, this),
    };

    gyro = new Pigeon2(GYRO_ID);
    gyro.setYaw(0);

    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getAngle(), getModulePositions(), new Pose2d());
    field = new Field2d();
    SmartDashboard.putData(field);
    SmartDashboard.putData(this);
    for(SwerveModule m : modules) {
      SmartDashboard.putData(m.name, m);
    }
    modules[0].debug = true;

    SmartDashboard.putData("set coast", new InstantCommand(() -> setNeutralMode(NeutralMode.Coast)).ignoringDisable(true));
    SmartDashboard.putData("set brake", new InstantCommand(() -> setNeutralMode(NeutralMode.Brake)).ignoringDisable(true));

    SmartDashboard.putData("reset wheels", new InstantCommand(() -> {
      resetWheels();
    }).ignoringDisable(true));

    SmartDashboard.putData("Chassis Move Sysid", (new Sysid(this::setModulesPower, this::getMoveVelocity, 0.1, 0.5, this)).getCommand());
    new TestVelocity("Chassis", this::setVelocity, this::getMoveVelocity, 0.05, this);

  }

  public SwerveModule getModule(int i) {
    return modules[i];
  }

  public void resetWheels() {
    for (var module : modules) {
      module.setAngle(new Rotation2d());
    }
  }

  /**
   * Stops the entire chassis
   */
  public void stop() {
    for(var m : modules) {
      m.stop();
    }
  }


  public void setModulesSteerPower(double power) {
    for(var m : modules) {
      m.setSteerPower(power);
    }
  }

  public void setModulesPower(double power) {
    for(var m : modules) {
      m.setPower(power);
    }
  }

  public void setModulesSteerVelocity(double v) {
    for(var m : modules) {
      m.setSteerVelocity(v, false);
    }
  }
  
  public double[] getAngularVelocities() {
    double[] angularVelocities = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      angularVelocities[i] = modules[i].getSteerVelocity();
    }
    return angularVelocities;
  }

  public void setVelocity(double v) {
    for(SwerveModule m : modules) {
      m.setVelocity(v);
    }
  }

  public double[] getVelocities() {
    double[] angularVelocities = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      angularVelocities[i] = modules[i].getVelocity();
    }
    return angularVelocities;
  }



  /**
   * Sets the velocity of the chassis
   * @param speeds In m/s and rad/s
   */
  public void setVelocities(ChassisSpeeds speeds) {
    ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(relativeSpeeds);
    setModuleStates(states);
  }

  /**
   * Returns the velocity vector
   * @return Velocity in m/s
   */
  public Translation2d getVelocity() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  public double getMoveVelocity() {
    return getVelocity().getNorm();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets the neutral mode of every motor in the chassis
   * @param mode
   */
  public void setNeutralMode(NeutralMode mode) {
    Arrays.stream(modules).forEach((module) -> module.setNeutralMode(mode));
  }

  /**
   * Returns the angle of the gyro
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void setOdometryToForward(){
    poseEstimator.resetPosition(getAngle(), getModulePositions(), 
          new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(),Rotation2d.fromDegrees(0)));
  }

  /**
   * Returns the position of every module
   * @return Position relative to the field
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] res = new SwerveModulePosition[modules.length];
    for(int i = 0; i < modules.length; i++) {
      res[i] = modules[i].getModulePosition();
    }
    return res;
  }

  /**
   * Returns the state of every module
   * @return Velocity in m/s, angle in Rotation2d
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] res = new SwerveModuleState[modules.length];
    for(int i = 0; i < modules.length; i++) {
      res[i] = modules[i].getState();
    }
    return res;
  }

  public double[] getModulesAngles() {
    double[] angles = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      angles[i] = modules[i].getAngle().getDegrees();
    }
    return angles;
  }

  /**
   * Sets the state of every module
   * @param states Velocity in m/s, angle in Rotation2d
   */
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void updateField() {
    field.setRobotPose(getPose());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("chassis velocity", () -> getVelocity().getNorm(), null);
      builder.addDoubleProperty("chassis ang velocity", () -> Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond), null);
      SmartDashboard.putData("Set Modules Angle", new RunCommand(()->setModulesAngleFromSB(0)));
      SmartDashboard.putNumber("Angle", 90);
    }

    public void setModulesAngleFromSB(double angle) {
      Rotation2d a = Rotation2d.fromDegrees(angle);
      for (SwerveModule module : modules) {
        module.setAngle(a);
      }
    }


  @Override
  public void periodic() {
      poseEstimator.update(getAngle(), getModulePositions());
      updateField();
      SmartDashboard.putNumber("gyro angle", getAngle().getDegrees());
      SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
      SmartDashboard.putNumber("gyro roll", gyro.getRoll());

      SmartDashboard.putNumber("absolute encoder 2", modules[2].getAbsoluteEncoder());
      SmartDashboard.putNumber("absolute encoder 3", modules[3].getAbsoluteEncoder());
  }
}
