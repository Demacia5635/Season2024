// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.Shooter;

public class AngleControl extends Command {

    Shooter shooter;
    CommandXboxController controller;

    /** Creates a new NeonControl. */
    public AngleControl(Shooter shooter, CommandXboxController controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooter = shooter;
        this.controller = controller;
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.anlgeSetVel(Math.abs(controller.getLeftY()) >= 0.3 ? -1*(controller.getLeftY() * 10000) : 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.anlgeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
