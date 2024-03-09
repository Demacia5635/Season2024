// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Field;
import frc.robot.RobotContainer;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_MODE;
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_MOTOR;
import frc.robot.utils.Utils;

public class ActivateShooter extends Command {

    
    /**true if currently is shooting */
    boolean isShooting;

    /**timer to stop the shooter after 0.5 sec */
    Timer timer;

    /**the wanted shooter */
    Shooter shooter;

    /**the wanted intake */
    Intake intake;

    /**the wanted chassis pos */
    Chassis chassis;

    /**the pos to the speaker pos*/
    Translation2d speaker;

    /**a var to check if the shooter is ready to shoot */
    boolean isReady = false;

    /**the vel to the up motor */
    double velUp;

    /**the vel to the down motor */
    double velDown;

    /**the angle to the shooter */
    double angle;

    boolean hasCalibrated = false;

    /**
     * activate the shooter
     * @param shooter the wanted shooter
     * @param intake the wanted intake
     * @param chassis the wanted chassis pos
     * @param from the pos to shoot, if null will shoot from chassis pos
     * @param isContinious if the command will not stop the motor
     */
    public ActivateShooter(Shooter shooter, Intake intake, Chassis chassis) {
        this.shooter = shooter;
        this.intake = intake;
        this.chassis = chassis;
        this.timer = new Timer();
        addRequirements(shooter);
    }

    /**
     * set the speaker to the wanted alliance pos
     * make finish to false
     * reset timer
     * set shotoer active to true
     */
    @Override
    public void initialize() {
    }

    /**
     * put the var based on if shooting to amp or speaker
     * put the angle motor at the right angle
     * if the shooter is not ready set the shooting motor vel
     * if the shooter is ready set the intake and the feeding motor full power
     * if the shooter is ready and the timer passed 0.4 than finish shooting
     */
    @Override
    public void execute() {
        speaker = Utils.speakerPosition();

        if(!hasCalibrated) {
            shooter.angleSetPow(-0.3);
            if (shooter.isLimit()) {
                shooter.angleSetPow(0);
                hasCalibrated = true;
            } else {
                return;
            }
        }

        switch (shooter.getShooterMode()) {
            case AMP:
                // velUp = ShooterConstants.AmpVar.UP;
                // velDown = ShooterConstants.AmpVar.DOWN;
                // angle = ShooterConstants.AmpVar.ANGLE;
                velDown = SmartDashboard.getNumber("vel calibrate", 0);
                velUp = velDown;
                angle = SmartDashboard.getNumber("angle calibrate", 0);

                break;

            case PODIUM:
                velUp = ShooterConstants.PodiumVar.UP;
                velDown = ShooterConstants.PodiumVar.DOWN;
                angle = ShooterConstants.PodiumVar.ANGLE;
                break;

            case SUBWOOFER:
                var av = Utils.getShootingClose();
                angle = av.getFirst();
                velDown = av.getSecond();
                velUp = velDown;
                break;

            // case AUTO :
            case AUTO_CONTINIOUS, AUTO:

                double dis = speaker.minus(chassis.getPose().getTranslation()).getNorm();
                av = Utils.getShootingAngleVelocity(dis);
                angle = av.getFirst();
                velDown = av.getSecond();
                velUp = velDown;

                break;

            case IDLE:
                velUp = 0;
                velDown = 0;
                angle = ShooterConstants.PodiumVar.ANGLE;
                break;
        
          
        }
       
        /*put the anlge motor at the wanted angle */
        double angleError = shooter.getAngle() - angle;
        angleError = Math.abs(angleError) > 0.5 ? angleError: 0;
        boolean isAtLimit = shooter.isDisLimits(angleError > 0);

        double power = isAtLimit ? 0:
                MathUtil.clamp(0.07 * Math.signum(angleError) + angleError * 0.06, -0.4, 0.4);
        if(isAtLimit) {
            System.out.println(" shooter is at limtit " + " angle = " + angle + " angle error =" + angleError + " cur angle = " + 
                shooter.getAngle());
        }
        shooter.angleSetPow(power);

        /*start the shooting motors */
        shooter.setVel(velUp, velDown);
        double velErrorUp = shooter.getMotorVel(SHOOTER_MOTOR.UP) - velUp;
        double velErrorDown = shooter.getMotorVel(SHOOTER_MOTOR.DOWN) - velDown;
        isReady = Math.abs(angleError) < 1 && 
                          Math.abs(velErrorUp) < 0.5 && 
                          Math.abs(velErrorDown) < 0.5;
        shooter.setIsShootingReady(isReady && velDown>0);

        if(isShooting)
             System.out.println("timer value" + timer.get());

        /*checks if the shooter is ready and if the timer did not hit 0.4*/
        if (!isShooting && shooter.getIsShooting()) {
            /*if the shooter is ready than start the timer */
            isShooting = true;
            timer.reset();
            timer.start();
            shooter.feedingSetPow(1);
            intake.setPower(1);

        } else if (isShooting && timer.get() > 1) {
            /*if the shooter is shooting and the timer hit 0.4 than stop shooting */
            isShooting = false;
            timer.reset();
            shooter.setIsShooting(false);
            shooter.feedingSetPow(0);
            intake.setPower(0);
            shooter.setIsShootingReady(false);

            if (shooter.getShooterMode()!=SHOOTER_MODE.AUTO_CONTINIOUS) {
                shooter.setShooterMode(SHOOTER_MODE.IDLE);
            }
        }
    }

    /**
     * stop the shooter and intake if the want to finish
     */
    @Override
    public void end(boolean interrupted) {


    }

    /**
     * end the command if finish or timer is bigger than 0.5 means that the command do not want to be stopped
     */
    @Override
    public boolean isFinished() {
        //SmartDashboard.putBoolean("is activate finish", finish);
      return false;
    }
}
