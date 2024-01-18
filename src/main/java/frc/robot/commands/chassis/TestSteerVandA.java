package frc.robot.commands.chassis;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.chassis.SwerveModule;

public class TestSteerVandA extends Command {
    
    Chassis chassis;
    int n;
    double power;
    SwerveModule module;

    public TestSteerVandA(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
        module = chassis.getModule(0);
    }

    @Override
    public void initialize() {
        power = 0.2;
        n = 0;
    }

    @Override
    public void execute() {
         module.setSteerPower(power);
         n++;
        System.out.println(" power = " + power + " v=" + module.getSteerVelocity() + " a =" + module.getAngleDegrees());
        if(n == 5) {
            power = -power;
            n = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        module.setSteerPower(0);
    }
}
