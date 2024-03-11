package frc.robot.commands.chassis.Auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class Check extends Command{

    Chassis chassis;

    public Check(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
    }




    @Override
    public void execute() {
        chassis.setVelocities(new ChassisSpeeds(-1, 0, 0.5));
        System.out.println("gyro rate= " + Math.toRadians(chassis.getGyroRate()) + ", y= " + Math.toRadians(chassis.getGyroRateY()) + ", x= " + Math.toRadians(chassis.getGyroRateX()));
    }
    
}
