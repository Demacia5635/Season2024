package frc.robot.subsystems.chassis;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sysid.Sysid;
import frc.robot.Sysid.Sysid.Gains;
import frc.robot.commands.chassis.CheckModulesSteerVelocity;
import frc.robot.commands.chassis.SetModuleAngle;
import frc.robot.commands.chassis.utils.TestVelocity;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.LedControll;
import frc.robot.subsystems.LedControll.ledState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utils.Utils;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.utils.ResetWheelCommand;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

import java.util.ArrayList;
import static frc.robot.subsystems.chassis.ChassisConstants.*;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.fasterxml.jackson.core.StreamReadConstraints.Builder;

public class Chassis extends SubsystemBase {
  private final SwerveModule[] modules;
  private final Pigeon2 gyro;

  public static List<pathPoint> pointsForPathTeleop = new ArrayList<pathPoint>();
  public static List<pathPoint> pointsForAuto = new ArrayList<pathPoint>();

  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field;
  private LedControll ledControll;

  public Chassis(LedControll ledControll) {
    modules = new SwerveModule[] {
        new SwerveModule(FRONT_LEFT, this),
        new SwerveModule(FRONT_RIGHT, this),
        new SwerveModule(BACK_LEFT, this),
        new SwerveModule(BACK_RIGHT, this),
    };
    this.ledControll = ledControll;
    //configAllFactoryDefaut();

    gyro = new Pigeon2(GYRO_ID);
    gyro.setYaw(0);

    poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getAngle(), getModulePositions(), new Pose2d());
    field = new Field2d();
    SmartDashboard.putData(field);
    SmartDashboard.putData(this);
    for (SwerveModule m : modules) {
      SmartDashboard.putData(m.name, m);
    }
    modules[0].debug = true;

    SmartDashboard.putData("set coast",
        new InstantCommand(() -> setNeutralMode(NeutralMode.Coast)).ignoringDisable(true));
    SmartDashboard.putData("set brake",
        new InstantCommand(() -> setNeutralMode(NeutralMode.Brake)).ignoringDisable(true));
    SmartDashboard.putData("reset wheels", new InstantCommand(() -> resetWheels()).ignoringDisable(true));
        SmartDashboard.putData("reset pose", new InstantCommand(() -> setOdometryToForward()).ignoringDisable(true));

    SmartDashboard.putData("calibrate", new InstantCommand(()->calibrate(), this));

    SmartDashboard.putData("Chassis Move Sysid",
        (new Sysid(this::setModulesPower, this::getMoveVelocity, 0.2, 0.8, this)).getCommand());
    SmartDashboard.putData("Chassis Move Sysid2",
        (new Sysid(new Gains[] { Gains.KS, Gains.KV, Gains.KA, Gains.KV2, Gains.KVsqrt},
        this::setModulesPower,
        this::getMoveVelocity,
        null,
        null,
        0.1,
        0.6,
        3,
        1,
        1,
        this)).getCommand());
    SmartDashboard.putData("Test Steer Velocity", (new CheckModulesSteerVelocity(this, 200)));
    SmartDashboard.putData("Set Modules Angle", (new SetModuleAngle(this)));
    new TestVelocity("Chassis", this::setVelocity, this::getMoveVelocity, 0.05, this);
    SmartDashboard.putNumber("angle for chassis", 0);
    SmartDashboard.putData("go to 0", new RunCommand(()->setModulesAngleFromSB(SmartDashboard.getNumber("angle for chassis", 0)), this));

    SmartDashboard.putNumber("ANG", 0);
    SmartDashboard.putData("go to angle position", new RunCommand(()->modules[0].setAngleByPositionPID(Rotation2d.fromDegrees(SmartDashboard.getNumber("ANG", 0))), this));


  }

  public SwerveModule[] getModules() {
    return modules;
  }

  public void calibrate() {
    SmartDashboard.putNumber("LEFT FRONT", modules[0].getAngleDegrees());
    SmartDashboard.putNumber("RIGHT FRONT", modules[1].getAngleDegrees());
    SmartDashboard.putNumber("LEFT BACK", modules[2].getAngleDegrees());
    SmartDashboard.putNumber("RIGHT BACK", modules[3].getAngleDegrees());
  }

  public void setGyroAngle(double angle){
    gyro.setYaw(angle);
  }

  public SwerveModule getModule(int i) {
    return modules[i];
  }

  public void resetWheels() {
    for (var module : modules) {
      module.setAngleByPositionPID(new Rotation2d());
    }
  }

  /**
   * Stops the entire chassis
   */
  public void stop() {
    for (var m : modules) {
      m.stop();
    }
  }

  public void setModulesSteerPower(double power) {
    for (var m : modules) {
      m.setSteerPower(power);
    }
  }

  public void setModulesPower(double power) {
    for (var m : modules) {
      m.setPower(power);
    }
  }

  public void setModulesSteerVelocity(double v) {
    for (var m : modules) {
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
    for (SwerveModule m : modules) {
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
   * 
   * @param speeds In m/s and rad/s
   */
  public void setVelocities(ChassisSpeeds speeds) {
    ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getAngle());
    SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(relativeSpeeds);
    setModuleStates(states);
  }


  public void configAllFactoryDefaut() {
    for (int i=0; i<modules.length; i++) {
      modules[i].configFactoryDefault();
    }
  }



  /**
   * Returns the velocity vector
   * 
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
   * 
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

  public void setOdometryToForward() {
    gyro.setYaw(0);
    poseEstimator.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(),
        new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), getAngle()));
  }

  /**
   * Returns the position of every module
   * 
   * @return Position relative to the field
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] res = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      res[i] = modules[i].getModulePosition();
    }
    return res;
  }

  /**
   * Returns the state of every module
   * 
   * @return Velocity in m/s, angle in Rotation2d
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] res = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      res[i] = modules[i].getState();
    }
    return res;
  }

  public double[] getModulesAngles() {
    double[] angles = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      angles[i] = modules[i].getAngleDegrees();
    }
    return angles;
  }

  

  /**
   * Sets the state of every module
   * 
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("velocity", () -> getVelocity().getNorm(), null);
    builder.addDoubleProperty("Omega", () -> Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond), null);
    builder.addDoubleProperty("Angle", () -> Utils.degrees(getAngle()), null);
    builder.addDoubleProperty("Pitch", () -> gyro.getPitch(), null);
    builder.addDoubleProperty("Roll", () -> gyro.getRoll(), null);
    builder.addDoubleProperty("Pose X", () -> getPose().getX(), null);
    builder.addDoubleProperty("Pose Y", () -> getPose().getY(), null);
    SmartDashboard.putData("Set Modules Angle", new RunCommand(() -> setModulesAngleFromSB(0)));
  }

  public void setModulesAngleFromSB(double angle) {
    Rotation2d a = Rotation2d.fromDegrees(angle);
    for (SwerveModule module : modules) {
      module.setAngleByPositionPID(a);
    }
  }

  
  public Rotation2d getClosetAngleApriltag(){
    Translation2d finalVector = new Translation2d(Integer.MAX_VALUE, Integer.MAX_VALUE);
    //checks the distance from each april tag and finds
    for(int i = 0; i < aprilTagsPositions.length; i++){
      Translation2d currentAprilTagVector = getPose().minus(aprilTagsPositions[i]).getTranslation();

     if(currentAprilTagVector.getNorm() < finalVector.getNorm()){
      finalVector = currentAprilTagVector;
     }
      
    }

    return finalVector.getAngle();

  }

  @Override
  public void periodic() {
    
    poseEstimator.update(getAngle(), getModulePositions());
    field.setRobotPose(getPose());
    double[] llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython")
      .getDoubleArray(new double[8]);
    double distance = llpython[0];
    if(distance != 0){
      ledControll.state = ledState.SEE_NOTE;
    }
        SmartDashboard.putNumber("velocity of chassis", getMoveVelocity());
  }
}