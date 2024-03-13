package frc.robot.commands.chassis.Auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ChassisConstants;
import frc.robot.RobotContainer;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.DriveToNote;
import frc.robot.commands.chassis.GoToAngleChassis;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;
import frc.robot.subsystems.chassis.ChassisConstants.*;

public class AutoUtils {

    private static double offset = 10;

    static Shooter shooter = RobotContainer.robotContainer.shooter;
    static Chassis chassis = RobotContainer.robotContainer.chassis;
    static Intake intake = RobotContainer.robotContainer.intake;
    static double maxVel = ChassisConstants.MAX_DRIVE_VELOCITY;
    static double maxAceel = ChassisConstants.DRIVE_ACCELERATION;
    static pathPoint dummyPoint = new pathPoint(0, 0, new Rotation2d(), 0, false);
    static Translation2d speaker = Utils.speakerPosition();
    static boolean isRed = RobotContainer.robotContainer.isRed();


    public static void addCommands(Command c, SequentialCommandGroup cmd) {
        cmd.addCommands(c);
    }

    public static boolean isInside(Translation2d wantedPos){
        return Math.abs(wantedPos.getX() - chassis.getPoseX()) <=  offset &&
        Math.abs(wantedPos.getY() - chassis.getPoseY()) <= offset;
    }
    

    public static pathPoint offset(Translation2d from, double x, double y, double angle) {
        System.out.println(" Point at " + (from.getX() + x) + " / " + (from.getY()+y));
        return new pathPoint(from.getX()+x, from.getY()+ y, Rotation2d.fromDegrees(angle),0,false);
    }


    public static  Command shoot() {
        return shooter.getShootCommand();
    }

    public static  Command initShooter() {
        return new WaitUntilCommand(() -> shooter.getIsShootingReady());
    }

    public static  Command goTo(pathPoint point) {
        return goTo(point, maxVel);
    }

    public static Command goToMultiple(pathPoint[] points, double maxVel){
        return new PathFollow(chassis, points, maxVel, maxAceel, 0, false);
    }
    public static  Command goTo(pathPoint point, double maxv) {
        return new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxv, maxAceel, 0, true);
    }


    public static  Command getNote(pathPoint point) {
        return (new PathFollow(chassis, new pathPoint[] { dummyPoint, point }, maxVel, maxAceel, 1, false)
                .raceWith(new WaitUntilCommand(() -> Utils.seeNote())))
                .andThen(takeNote());
    }

    public static  Command takeNote() {
        return new DriveToNote(chassis, 1, true)
        .raceWith(new IntakeCommand(intake)).andThen(new IntakeCommand(intake)).withTimeout(3);
    }

    
}
