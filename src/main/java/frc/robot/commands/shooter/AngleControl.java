// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.Shooter;

/**command that control angle */
public class AngleControl extends Command {

    /**the shooter that want to be used */
    Shooter shooter;
    /**the controller we want to use */
    CommandXboxController controller;

    /**
     * new command that control the angle through controller
     * @param shooter the shooter that we want to control the angle motor of
     * @param controller the controller we want to get the jostick
     */
    public AngleControl(Shooter shooter, CommandXboxController controller) {
        this.shooter = shooter;
        this.controller = controller;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    /**if the joystick is less than 0.3 that means its does not meant to be there and calibrate to 0 */
    @Override
    public void execute() {
        shooter.angleSetPow(Math.abs(controller.getLeftY()) >= 0.3 ? -1*(controller.getLeftY() * 0.5) : 0);
    }

    // Called once the command ends or is interrupted.
    /**stop the angle motor when finishing the command */
    @Override
    public void end(boolean interrupted) {
        shooter.anlgeStop();
    }

    // Returns true when the command should end.
    /**checks if the amper is more than the limit */
    @Override
    public boolean isFinished() {
        return shooter.isSupplyLimit(4);
    }
}
