// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.amp.AmpConstants;

public class AmpIntake extends Command {
  Amp amp;
  boolean last;
  double v1;
  double v2;
  double countRev[] = new double[2];
  double count1;
  double count2;
  /** Creates a new AmpIntake. */
  public AmpIntake(Amp amp, double v1, double v2) {
    this.amp = amp;
    this.v1 = v1;
    this.v2 = v2;
    addRequirements(amp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    amp.neoEncoderReset();
    last = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(amp.isClose()){
      if(!amp.isNoteThere(last)){
        amp.neosSetVel(v1, v2);
        countRev = amp.getNeosRev();
        count1 = countRev[0];
        count2 = countRev[1];
        countRev = amp.getNeosRev();
        count1 = countRev[0];
        count2 = countRev[1];
      }else {
        if(amp.getNeosRev()[0]-count1 >=AmpConstants.CommandParams.NUM_OF_ROTATION){
          amp.neosSetVel(0, 0);
        }else{
          amp.neosSetVel(v1, v2);
        } 
        if(amp.getNeosRev()[0]-count1 >=AmpConstants.CommandParams.NUM_OF_ROTATION){
          amp.neosSetVel(0, 0);
        }else{
          amp.neosSetVel(v1, v2);
        } 
      }
    }
    if(amp.isOpen()){
      if(amp.isNoteThere(last)){
        amp.neosSetVel(-v1, -v2);
        countRev = amp.getNeosRev();
        count1 = countRev[0];
        count2 = countRev[1];
      }else {
        if(count1- amp.getNeosRev()[0] >=AmpConstants.CommandParams.NUM_OF_ROTATION){
          amp.neosSetVel(0, 0);
        }else{
          amp.neosSetVel(v1, v2);
        } 
        if(count1- amp.getNeosRev()[0] >=AmpConstants.CommandParams.NUM_OF_ROTATION){
          amp.neosSetVel(0, 0);
        }else{
          amp.neosSetVel(v1, v2);
        } 
      }
    }    
    last = amp.didNotePass();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    amp.neosSetVel(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((amp.isOpen() ) && (count1- amp.getNeosRev()[0] >=AmpConstants.CommandParams.NUM_OF_ROTATION) && (!amp.isNoteThere(last))) || ((amp.isClose()) && (amp.getNeosRev()[0]-count1 >=AmpConstants.CommandParams.NUM_OF_ROTATION) && (amp.isNoteThere(last))) );
  }
}