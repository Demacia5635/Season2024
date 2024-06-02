// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.Auto.OpenDay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassisConstants;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.Auto.AutoUtils;
import frc.robot.commands.chassis.Paths.GoTo;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import static frc.robot.Constants.OpenDayAutoConstants.*;
import static frc.robot.commands.chassis.Auto.AutoUtils.goTo;

/** Add your docs here. */
public class OpenDayAuto {

    static Chassis chassis = RobotContainer.robotContainer.chassis;
    static Shooter shooter = RobotContainer.robotContainer.shooter;
    static Intake intake = RobotContainer.robotContainer.intake;
    public static SequentialCommandGroup cmd = new SequentialCommandGroup();


    public OpenDayAuto() {
    }

    public static void moveFwd(double dis) {
        cmd.addCommands(new MoveFwd(dis, chassis));
    }

    public static void wait(double sec) {
        cmd.addCommands(new WaitCommand(sec));
    }
    
    public static void moveRight(double dis) {
        cmd.addCommands(new GoTo(new Pose2d(chassis.getPoseX() + dis, chassis.getPoseY(), chassis.getAngle()), chassis.getPose(), MAX_VEL, MAX_ACC, false, chassis));
    }

    public static void moveRev(double dis) {
        cmd.addCommands(new GoTo(new Pose2d(chassis.getPoseX(), chassis.getPoseY() - dis, chassis.getAngle()), chassis.getPose(), MAX_VEL, MAX_ACC, false, chassis));
    }


    public static void moveLeft(double dis) {
        cmd.addCommands(new GoTo(new Pose2d(chassis.getPoseX() - dis, chassis.getPoseY(), chassis.getAngle()), chassis.getPose(), MAX_VEL, MAX_ACC, false, chassis));
    }

    public static void intake() {
        cmd.addCommands(new IntakeCommand(intake));
    }

    public static void shoot() {
        cmd.addCommands(new RunCommand(()->chassis.setVelocitiesRotateToSpeaker(new ChassisSpeeds()), chassis)
            .raceWith(new WaitUntilCommand(()->shooter.getIsShootingReady() && Math.abs(chassis.getErrorSpeakerAngle()) < 4)));

        cmd.addCommands(shooter.getShootCommand());
    }
}
