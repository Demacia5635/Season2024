package frc.robot.commands.chassis;

import org.ejml.interfaces.decomposition.CholeskySparseDecomposition_F64;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class SetModuleAngle extends Command {

    Chassis chassis;
    double angle;
    SwerveModuleState[] states = new SwerveModuleState[4];

    public SetModuleAngle(Chassis chassis, double angle) {
        this.chassis = chassis;
        this.angle = angle;
        for(int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
        }
        addRequirements(chassis);
    }

    @Override
    public void execute() {
        chassis.setModuleStates(states);
    }

    @Override
    public boolean isFinished() {
        var a = chassis.getModulesAngles();
        for(double an : a) {
            if(Math.abs(an-angle) > 1) {
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
