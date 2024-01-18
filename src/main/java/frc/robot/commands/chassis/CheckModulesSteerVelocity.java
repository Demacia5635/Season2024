package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class CheckModulesSteerVelocity extends Command {

    Chassis chassis;
    double velocity;


    public CheckModulesSteerVelocity(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
        if(SmartDashboard.getNumber("CheckSteerVelocity", 1000) == 1000) {
            SmartDashboard.putNumber("CheckSteerVelocity", 1);
            SmartDashboard.getNumber("CheckSteerVelocity", 1000);
        }
    }

    @Override
    public void initialize() {
        velocity = SmartDashboard.getNumber("CheckSteerVelocity", 1);
        SmartDashboard.putNumber("CheckSteerVelocity Target", velocity);
    }

    @Override
    public void execute() {
        for(int i = 0; i < 4; i++) {
            chassis.getModule(i).setVelocity(velocity);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

}
