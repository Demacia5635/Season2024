package frc.robot.commands.chassis.Auto.OpenDay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PathFollow.Util.pathPoint;
import frc.robot.commands.chassis.Paths.PathFollow;
import frc.robot.subsystems.chassis.Chassis;

public class MoveFwd extends Command {

    double dis;
    Chassis chassis;
    PathFollow pathFollow;

    public MoveFwd(double dis, Chassis chassis) {
        this.dis = dis;
        this.chassis = chassis;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        pathFollow = new PathFollow(new pathPoint[]{
            new pathPoint(new Translation2d(chassis.getPoseX(),chassis.getPoseY()), chassis.getAngle()),
            new pathPoint(new Translation2d(chassis.getPoseX(),chassis.getPoseY() + dis), chassis.getAngle())
        });

        System.out.println(chassis.getPose().toString());

        pathFollow.schedule();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return pathFollow.isFinished();
    }
}