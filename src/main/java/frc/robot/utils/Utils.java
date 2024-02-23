package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.leds.SubStrip;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Utils {
    
    public static double degrees(Rotation2d r) {
        return MathUtil.inputModulus(r.getDegrees(), -180, 180);
    }
    public static double degrees(double angle) {
        return MathUtil.inputModulus(angle, -180, 180);
    }

    public static double angleDif(Rotation2d r1, Rotation2d r2) {
      return degrees(r1.minus(r2));
    }
    public static boolean joystickOutOfDeadband(CommandXboxController controller){
        return deadband(controller.getLeftX(), 0.1) != 0 ||
        deadband(controller.getLeftX(), 0.1) != 0
        || deadband(controller.getLeftTriggerAxis(), 0.1) != 0||
        deadband(controller.getRightTriggerAxis(), 0.1) != 0;
    }
    
  public static double deadband(double x, double threshold) {
    return (Math.abs(x) < threshold)?0:x;
  }

  public static boolean seeNote() {
        double[] llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
        return llpython[0] != 0 && llpython[0] < 200;
  }

  private static double shootDistance[] = {1.35, 1.7, 3.7, 4.65};
  private static double shootAngle[] = {56, 53, 37, 34};
  private static double shootVelocity[] = {16.5, 16.7, 17.5, 18};

  public static double extrapolatre(double d1, double d2, double v1, double v2, double d) {
    return v1 + (v2-v1)*(d-d1)/(d2-d1);
  }
  
  public static Pair<Double,Double> getShootingAngleVelocity(double distance) {
    double v = 0;
    double a = 0;
    int i = 0;
    while(i < shootDistance.length && shootDistance[i] < distance) {
      i++;
    }
    if(i == shootDistance.length) {
      v = shootVelocity[i-1];
      a = shootAngle[i-1];
    } else if(i == 0) {
      v = shootVelocity[i];
      a = shootAngle[i];
    } else {
      v = extrapolatre(shootDistance[i-1],shootDistance[i], shootVelocity[i-1],shootVelocity[i], distance);
      a = extrapolatre(shootDistance[i-1],shootDistance[i], shootAngle[i-1],shootAngle[i], distance);
    }

    return new Pair<Double,Double>(a,v);
  }

  // public static Command setLed(SubStrip led){
  //   double[] llpython = NetworkTableInstance.getDefault().getTable("limelight").getEntry("llpython").getDoubleArray(new double[8]);
  //   double Dist = llpython[0];
  //   System.out.println("Dist is : " + Dist);
  //   if(Dist != 0){
  //     if(Dist <= 150){
  //       return led.setBlink(Color.kGreen);
  //     }
  //     else{
  //       return led.setColor(Color.kGreen);
  //     }
  //   } else {
  //     return led.turnOff();
  //   }
  // }

  public static Translation2d speakerPosition() {
    return RobotContainer.robotContainer.isRed()? ShooterConstants.RED_ALLIANCE_SPEAKER.getTranslation():
      ShooterConstants.BLUE_ALLIANCE_SPEAKER.getTranslation();
  }
}
