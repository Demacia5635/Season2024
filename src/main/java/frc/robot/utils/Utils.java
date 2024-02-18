package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Utils {
    
    public static double degrees(Rotation2d r) {
        return MathUtil.inputModulus(r.getDegrees(), -180, 180);
    }
    public static double degrees(double angle) {
        return MathUtil.inputModulus(angle, -180, 180);
    }
    public static boolean joystickOutOfDeadband(CommandXboxController controller){
        return deadband(controller.getLeftX(), 0.1) != 0 && deadband(controller.getLeftY(), 0.1) != 0
        && deadband(controller.getLeftTriggerAxis(), 0.1) != 0 && deadband(controller.getRightTriggerAxis(), 0.1) != 0 ;
            

    }
    public static double deadband(double x, double threshold) {
        return (Math.abs(x) < threshold)?0:x;
    }
}
