package frc.robot.commands.chassis;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.Constants;

public class SetModuleAngle extends Command {

    Chassis chassis;
    Rotation2d angle = null;
    double degrees = 0;
    private GenericEntry angleEntry;

    public SetModuleAngle(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
        ShuffleboardTab tab = Shuffleboard.getTab("Module Test");
        angleEntry = tab.add("Set Module Angle To", 0).getEntry();
        tab.add("Set Module Entry", this);

    }

    @Override
    public void initialize() {
        this.angle = Rotation2d.fromDegrees(angleEntry.getDouble(0));
    }

    @Override
    public void execute() {
        for(int i = 0; i < 4; i++) {
            chassis.getModule(i).setAngle(angle);
        }
    }

    @Override
    public boolean isFinished() {
        var angles = chassis.getModulesAngles();
//        System.out.println(" angles = " + angles[0] + " " + angles[1] + " " + angles[2] + " " + angles[3]);
        for(double a : angles) {
            if(Math.abs(a-degrees) > Constants.MAX_STEER_ERROR) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
   
}
