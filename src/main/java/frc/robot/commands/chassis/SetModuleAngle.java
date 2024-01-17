package frc.robot.commands.chassis;

import org.ejml.interfaces.decomposition.CholeskySparseDecomposition_F64;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.Constants;

public class SetModuleAngle extends Command {

    Chassis chassis;
    Rotation2d angle = null;
    double degrees = 0;

    public SetModuleAngle(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
        if(SmartDashboard.getNumber("SetModuleAngle", 1000) == 1000) {
            SmartDashboard.putNumber("SetModuleAngle", 0);
            SmartDashboard.getNumber("SetModuleAngle", 1000);
        }
    }

    @Override
    public void initialize() {
        degrees = SmartDashboard.getNumber("SetModuleAngle", 0);
        SmartDashboard.putNumber("SetModuleAngle Target", degrees);
        this.angle = Rotation2d.fromDegrees(degrees);
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
