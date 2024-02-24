package frc.robot.commands.chassis.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassisConstants;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.ActivateShooter;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;

public class StartTOP1 extends SequentialCommandGroup {
    double maxVel = ChassisConstants.MAX_DRIVE_VELOCITY;
    double maxAceel = ChassisConstants.DRIVE_ACCELERATION;
    Chassis chassis;
    Intake intake;
    Shooter shooter;
    boolean isRed;
    Translation2d speaker;

    static final double fieldLength = 16.54;
    static final double fieldWidth = 8.21;

    double nearNotesX = 1.4;
    double farNotesX = fieldLength/2 - 1.5;
    double topNearNoteY = fieldWidth/2 + 2*1.48 - 0.5; 
    double topFarNoteY = fieldWidth/2 + 2*1.68;

    pathPoint dummyPoint = new pathPoint(0, 0, new Rotation2d(), 0, false);
    pathPoint nearNote = new pathPoint(nearNotesX, topFarNoteY, Rotation2d.fromDegrees(-4), 0, false);
    pathPoint topFarNote = new pathPoint(farNotesX, topFarNoteY, new Rotation2d(0), 0, false);
    pathPoint secondFarNote = new pathPoint(farNotesX, topFarNoteY-1.68, new Rotation2d(0), 0, false);
    pathPoint shootPoint = new pathPoint(2.5, 6.58, new Rotation2d(), 0, false);

    /** Creates a new StartTOP auto. */
    public StartTOP1() {
        this.chassis = RobotContainer.robotContainer.chassis;
        this.intake = RobotContainer.robotContainer.intake;
        this.shooter = RobotContainer.robotContainer.shooter;
        this.isRed = RobotContainer.robotContainer.isRed();
        speaker = Utils.speakerPosition();

        addCommands(initShooter());
        addCommands(shoot());
        addCommands(getNote(nearNote));
        addCommands(turnToSpeaker());
        addCommands(shoot());
        addCommands(getNote(topFarNote));
        addCommands(goTo(shootPoint));
        addCommands(turnToSpeaker());
        addCommands(shoot());
        addCommands(getNote(secondFarNote));
        addCommands(goTo(shootPoint));
        addCommands(turnToSpeaker());
        addCommands(shoot());
    }

    private Command shoot() {
        return shooter.getShootCommand();
    }

    private Command initShooter() {
        return new ActivateShooter(shooter, intake, chassis, true)
                .andThen(new WaitUntilCommand(() -> shooter.isShootingReady()));
    }

    private Command goTo(pathPoint point) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxVel, maxAceel, 0, isRed, true);
    }

    private Command turnToSpeaker() {
        return new GoToAngleChassis(chassis, speaker);
    }

    private Command getNote(pathPoint point) {
        return (new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxVel, maxAceel, 1, isRed)
                .raceWith(new WaitUntilCommand(() -> Utils.seeNote())))
                .andThen(takeNote());
    }

    private Command takeNote() {
        return (new DriveToNote(chassis, 1).raceWith(new IntakeCommand(intake))).withTimeout(1);
    }

}