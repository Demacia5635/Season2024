package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbJoystick  extends Command{


    ClimbSubsystem climb;
    XboxController xboxController;

    private double powerLeft;
    private double powerRight;


    public ClimbJoystick(ClimbSubsystem climb, XboxController xboxController) {
        this.climb = climb;
        this.xboxController = xboxController;
        addRequirements(climb);
    }


    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        powerLeft = xboxController.getLeftY();
        powerRight = xboxController.getRightY();

        powerLeft = deadband(powerLeft, 0.1);
        powerRight = deadband(powerRight, 0.1);


        climb.climbLeft(powerLeft);
        climb.climbRight(powerRight);

        SmartDashboard.putNumber("powerLeft", powerLeft);
        SmartDashboard.putNumber("powerRight", powerRight);

    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }


    private double deadband(double x, double threshold) {
        if (Math.abs(x) < threshold) return 0;
        else return x;
      }

    


    
}
