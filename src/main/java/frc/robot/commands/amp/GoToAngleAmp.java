// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;
import frc.robot.utils.TrapezoidCalc;

public class GoToAngleAmp extends Command {
  Amp amp;
  double angleRad;
  double maxVelRad;
  double acceleRad;
  TrapezoidCalc trap;
  double startTime;

  /** Creates a new GoToAngleAmp. */
  public GoToAngleAmp( Amp amp, double angleRad, double maxVelRad, double acceleRad) {
    this.amp = amp;
    this.angleRad = angleRad;
    this.maxVelRad = maxVelRad;
    this.acceleRad = acceleRad;
    this.trap = new TrapezoidCalc();
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    if(amp.isOpen()){
      amp.setPowerSnowblower(-0.2);
    }
    amp.startRad(amp.getPoseByPulses());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp()-startTime >= 0.5){
      amp.setPowerSnowblower(0);
    }

    double currentAngleRad = amp.getPoseByPulses();
    double velRad = trap.trapezoid(amp.getVelRadArm(), maxVelRad, 0.17, Math.abs(acceleRad), angleRad-currentAngleRad);
    amp.setVel(velRad);
    if((amp.isClose()||amp.isOpen())&&(Timer.getFPGATimestamp()-startTime > 0.5)){
      amp.stop();
      if(amp.isOpen()){
        amp.runSnowblower(0.3, 350);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.setPowerSnowblower(0.1);
    amp.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return ((amp.getPoseRad()>=angleRad-10/360*2*Math.PI)&&(amp.getPoseRad()<=angleRad+10/360*2*Math.PI));
    return (amp.getSnowblowerA()>=350)&&(Timer.getFPGATimestamp()-startTime >1);
  }
}