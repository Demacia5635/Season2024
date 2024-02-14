package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class DispenseCommand extends CommandBase {
    private final Intake intake;
    private double count = 0;

    public DispenseCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
        
        
        SmartDashboard.putData("Dispense Note", new InstantCommand(
            () -> new DispenseCommand(intake).schedule()));
        }

    @Override
    public void initialize(){
        intake.setBrake();
        
    }

    @Override
    public void execute() {
        count +=0.02;
        intake.setPower(IntakeConstants.Parameters.DISPENSE_POWER); // Set dispense velocity
        
        // withTimeout(IntakeConstants.Parameters.DISPENSE_TIME); // Set timeout for dispensing
    }

    @Override
    public boolean isFinished() {
        // Command ends when current falls under minimum
        return (intake.getMotorCurrent() < IntakeConstants.Parameters.MIN_CURRENT_TO_AMP) && (count > IntakeConstants.Parameters.DISPENSE_TIME);
        //return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}