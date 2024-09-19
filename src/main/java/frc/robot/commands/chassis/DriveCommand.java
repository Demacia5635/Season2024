package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ChassisConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.chassis.Chassis;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.utils.Utils.*;


import static frc.robot.subsystems.chassis.ChassisConstants.*;

public class DriveCommand extends Command {
  private final Chassis chassis;
  private final PS5Controller controller;

  private double direction;

  private boolean isRed;
  private boolean precisionDrive = false;

  private boolean isSeeNote = false;
  private boolean hasNote = false;
  private boolean hasVx = false;

  private boolean autoIntake;

  private double[] llpython;

  NetworkTableEntry llentry;

  public boolean start;
  private Timer timer;
  Translation2d robotToNote;
  Translation2d notePos = new Translation2d();

  public DriveCommand(Chassis chassis, PS5Controller controller) {
    this.chassis = chassis;
    this.autoIntake = true;
    this.controller = controller;
    this.timer = new Timer();
    
   
    addRequirements(chassis);
    

  }

  @Override
  public void initialize() {
     llentry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython");
    llpython = llentry.getDoubleArray(new double[8]);

  }


  //need to add intake
  private boolean hasNote(){
    return RobotContainer.robotContainer.intake.isNotePresent();
  }

  //need to add intake
  private boolean isSeeNote(double distance){
    
    return distance != 0;
  }
  private boolean isNearNote(Translation2d notePos){
    return chassis.getPose().getTranslation().getDistance(notePos) <= 1;
  }

  @Override
  public void execute() {
    
    if(controller.getCircleButtonPressed()) precisionDrive = !precisionDrive;
    if(controller.getTriangleButtonPressed()) { autoIntake = !autoIntake;}
    isRed = chassis.isRed();
    direction = isRed ? 1 : -1;
    
    double joyX = deadband(controller.getLeftY(), 0.1) * direction;
    double joyY = deadband(controller.getLeftX(), 0.1) * direction;
    double rot = -(deadband((controller.getR2Axis() + 1) / 2, 0.1)
        - deadband((controller.getL2Axis() + 1) / 2, 0.1));

    double velX = Math.pow(joyX, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyX);
    double velY = Math.pow(joyY, 2) * MAX_DRIVE_VELOCITY * Math.signum(joyY);
    double velRot = Math.pow(rot, 2) * MAX_OMEGA_VELOCITY * Math.signum(rot);

    llpython = llentry.getDoubleArray(new double[8]);
    hasVx = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(velX, velY, 0), chassis.getAngle()).vxMetersPerSecond != 0;
    hasNote = hasNote();
    //isSeeNote = isSeeNote(llpython[0]);
    isSeeNote = false;
    System.out.println("is see note: " + isSeeNote);

    if (precisionDrive) {
      velX /= 4;
      velY /= 4;
      velRot /= 4;
    }
    if(hasVx && !hasNote && (isSeeNote || isNearNote(notePos)) && autoIntake){
      
 //     if(!RobotContainer.robotContainer.manualIntake.isScheduled() && autoIntake) RobotContainer.robotContainer.manualIntake.schedule();
      
      llpython = llentry.getDoubleArray(new double[8]);
      double distanceToNote = llpython[0] / 100;
      double angle = (isSeeNote) ? llpython[1] : chassis.getPose().getTranslation().minus(notePos).getAngle().getDegrees();
      
      notePos = chassis.getPose().getTranslation().plus(new Translation2d(distanceToNote, Rotation2d.fromDegrees(angle)));

      
      double vectorAngle = angle * 2;
      robotToNote = new Translation2d(getV(), Rotation2d.fromDegrees(vectorAngle));
      chassis.setVelocitiesRobotRel(new ChassisSpeeds(robotToNote.getX(), robotToNote.getY(), 0)); 
    } 

    else{
      
      ChassisSpeeds speeds = new ChassisSpeeds(velX, velY, velRot);
      //updateAutoIntake();
      chassis.setVelocities(speeds);
    }
  }

  private double getV(){
    return Math.min(Math.hypot(controller.getLeftX(), controller.getLeftY()) * ChassisConstants.MAX_DRIVE_VELOCITY, ChassisConstants.MAX_DRIVE_VELOCITY);
  }

}
