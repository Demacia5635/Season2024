package frc.robot.commands.chassis;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.chassis.Chassis;

public class CurveCommand extends Command {
    private Chassis chassis;
    private double timeSeconds;

    private Translation2d p0;
    private Translation2d p1;
    private Translation2d handle;

    private double passedDistance;
    private List<Double> distancesLUT;

    public CurveCommand(Chassis chassis, double timeSeconds, Translation2d p0, Translation2d p1, Translation2d handle) {
        this.chassis = chassis;
        this.timeSeconds = timeSeconds;
        this.p0 = p0;
        this.p1 = p1;
        this.handle = handle;

        distancesLUT = new ArrayList<>();
        loadLUT(distancesLUT, p0, p1, handle);

        addRequirements(chassis);
    }
    
    @Override
    public void initialize() {
        passedDistance = 0;
    }

    @Override
    public void execute() {
        Translation2d b = calculatePoint(p0, p1, handle, distanceToTime(distancesLUT, passedDistance));
        Translation2d dx = b.minus(chassis.getPose().getTranslation());
        Rotation2d dRot = dx.getAngle().minus(chassis.getAngle());
        chassis.setVelocities(new ChassisSpeeds(dx.getX() / 0.02 / timeSeconds, dx.getY() / timeSeconds / 0.02, 0));
        
        passedDistance += 1 * 0.02;
    }

    @Override
    public boolean isFinished() {
        return chassis.getPose().getTranslation().getDistance(p1) <= 0.2;
    }

    private Translation2d calculatePoint(Translation2d p0, Translation2d p1, Translation2d anchor, double time) {
        Translation2d q0 = p0.interpolate(handle, time);
        Translation2d q1 = handle.interpolate(p1, time);
        Translation2d b = q0.interpolate(q1, time);
        return b;
    }

    private void loadLUT(List<Double> LUT, Translation2d p0, Translation2d p1, Translation2d anchor) {
        double dist = 0;
        Translation2d lastPoint = p0;
        final int sampleCount = 64;
        for (int i = 1; i <= sampleCount; i++) {
            Translation2d p = calculatePoint(p0, p1, anchor, (1.0/sampleCount) * i);
            dist += p.minus(lastPoint).getNorm();
            lastPoint = p;
            LUT.add(dist);
        }
    }

    private double distanceToTime(List<Double> LUT, double distance) {
        double curveLength = LUT.get(LUT.size() - 1);
        double n = LUT.size();
        if (distance >= 0 && distance <= curveLength) {
            for (int i = 0; i < n - 1; i++) {
                if (distance > LUT.get(i) && distance < LUT.get(i + 1)) {
                    return map(
                        distance,
                        LUT.get(i),
                        LUT.get(i + 1),
                        i / (n - 1.0),
                        (i + 1) / (n - 1.0)
                    );
                }
            }
        }
        return distance / curveLength;
    }

    private double map(double s, double a1, double a2, double b1, double b2) {
        return b1 + (s-a1)*(b2-b1)/(a2-a1);
    }
}

