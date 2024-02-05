package frc.robot.subsystems.climb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climb.BrakeClimbCommand;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimbBrakerSubsystem extends SubsystemBase {
    private final TalonSRX s_Motors;

    public ClimbBrakerSubsystem() {
        s_Motors = new TalonSRX(BRAKE_MOTORS);

        s_Motors.configFactoryDefault();
        s_Motors.setInverted(false);
    }

    public void setPower(double p) {
        s_Motors.set(ControlMode.PercentOutput, p);
    }

    public double getCurrent() {
        return s_Motors.getStatorCurrent();
    }

    public void brake() {
        new BrakeClimbCommand(this).schedule();
    }

    public void release() {
        new RunCommand(() -> setPower(-1), this).withTimeout(0.5).andThen(() -> setPower(0), this);
    }
}
