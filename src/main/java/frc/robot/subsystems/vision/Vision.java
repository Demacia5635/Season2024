package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.Arrays;
import java.util.Comparator;
import java.util.NoSuchElementException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chassis.Chassis;
import frc.robot.subsystems.vision.utils.SwerveDrivePoseEstimator;

import static frc.robot.Constants.VisionConstants.*;

public class Vision extends SubsystemBase {
    //declaring fields for testing
    Field2d poseEstimatorField;
    Field2d visionField;
    Field2d visionField3;
    Field2d visionField5;
    Field2d visionFieldavg3;
    Field2d visionFieldavg5;

    //declaring limelights
    PhotonCamera AmpSideRaspberry;
    PhotonPoseEstimator photonPoseEstimatorForAmpSideRaspberry;
    PhotonCamera ShooterSideRaspberry;
    PhotonPoseEstimator photonPoseEstimatorForShooterSideRaspberry;

    // declaring poseEstimator chassis and buffers 
    SwerveDrivePoseEstimator poseEstimator;
    Chassis chassis;
    VisionData[] buf3Avg;
    VisionData[] buf3Med;

    VisionData[] buf5Avg;
    VisionData[] buf5Med;


    int lastData;
    int lastData5;
    double lastUpdateTime;
    double lastUpdateTime5;
    boolean firstRun;
    int countCycles;

    SwerveDrivePoseEstimator testPoseEstimatorBuf3Avg;
    SwerveDrivePoseEstimator testPoseEstimatorBuf3Med;
    SwerveDrivePoseEstimator testPoseEstimatorBuf5Avg;
    SwerveDrivePoseEstimator testPoseEstimatorBuf5Med;

    public Vision(Chassis chassis, SwerveDrivePoseEstimator estimator, SwerveDrivePoseEstimator testPoseEstimatorBuf3Avg, SwerveDrivePoseEstimator testPoseEstimatorBuf3Med, SwerveDrivePoseEstimator testPoseEstimatorBuf5Avg, SwerveDrivePoseEstimator testPoseEstimatorBuf5Med) {
        
        this.testPoseEstimatorBuf3Avg = testPoseEstimatorBuf3Avg;
        this.testPoseEstimatorBuf3Med = testPoseEstimatorBuf3Med;
        this.testPoseEstimatorBuf5Avg = testPoseEstimatorBuf5Avg;
        this.testPoseEstimatorBuf5Med = testPoseEstimatorBuf5Med;
        
        
        
        
        
        this.chassis = chassis;
        this.poseEstimator = estimator;
        this.lastData = -1;
        this.lastData5 = -1;
        this.buf3Avg  = new VisionData[3];
        this.buf5Avg = new VisionData[5];
        this.buf3Med  = new VisionData[3];
        this.buf5Med = new VisionData[5];
        this.AmpSideRaspberry = new PhotonCamera(AmpSideRaspberryName);
        this.ShooterSideRaspberry = new PhotonCamera(ShooterSideRaspberryName);
        
        countCycles = 0;
        this.firstRun = true;

        //initializing photons pose estimators
        try {
            this.photonPoseEstimatorForAmpSideRaspberry = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
            PoseStrategy.AVERAGE_BEST_TARGETS, AmpSideRaspberry, robotCenterToAmpSideRaspberry);

            this.photonPoseEstimatorForShooterSideRaspberry = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile),
            PoseStrategy.AVERAGE_BEST_TARGETS, ShooterSideRaspberry, robotCenterToShooterSideRaspberry);
        } 
        catch (IOException e) {
            System.out.println("problem with photon pose estimators");
            e.printStackTrace();
        } 
        
        //initializing buffers
        for (int i = 0; i < buf3Avg.length; i++) {
            this.buf3Avg[i] = new VisionData(null, 0, testPoseEstimatorBuf3Avg);
        }
        for (int i = 0; i < buf5Avg.length; i++) {
            this.buf5Avg[i] = new VisionData(null, 0, testPoseEstimatorBuf5Avg);
        }
        for (int i = 0; i < buf3Med.length; i++) {
            this.buf3Med[i] = new VisionData(null, 0, testPoseEstimatorBuf3Med);
        }
        for (int i = 0; i < buf5Med.length; i++) {
            this.buf5Med[i] = new VisionData(null, 0, testPoseEstimatorBuf5Med);
        }

        //initializing fields for testing
        this.visionField= new Field2d();   
        this.visionField3 = new Field2d();
        this.visionField5 = new Field2d();
        this.visionFieldavg3 = new Field2d();
        this.visionFieldavg5 = new Field2d();
        this.poseEstimatorField = new Field2d();
        
        //putting testing fields on shuffleboard
        SmartDashboard.putData("no Filter vision", visionField);
        SmartDashboard.putData("field check pose estimator", poseEstimatorField);
        
        SmartDashboard.putData("vision3", visionField3);
        SmartDashboard.putData("vision5", visionField5);
        SmartDashboard.putData("visionavg3", visionFieldavg3);
        SmartDashboard.putData("visionavg5", visionFieldavg5);

        SmartDashboard.putData("resetVisionCycles", new InstantCommand(()->resetVisionResetCycles()).ignoringDisable(true));

    }
    public void resetVisionResetCycles(){
        countCycles = 0;
        SmartDashboard.putNumber("run bill", 1);
    }
    //takes the visions snapshots from the buffer and medians or avg it and add vision mesurements to pose estimator
    public void updateRobotPose() {
        double time = getTime();
        if (validBuf(time)) {
            VisionData vDataMed = median(buf3Med);
            Pair<Pose2d, Double> vData3AvgPair = avg(buf5Avg);
            VisionData vDataAvg = new VisionData(vData3AvgPair.getFirst(), vData3AvgPair.getSecond(), testPoseEstimatorBuf3Avg)  ;
            if (vDataMed != null && vDataMed.getPose() != null && vDataAvg.getPose() != null && vDataAvg != null) {
                testPoseEstimatorBuf3Med.addVisionMeasurement(vDataMed.pose, vDataMed.timeStamp);
                testPoseEstimatorBuf3Avg.addVisionMeasurement(vDataAvg.pose, vDataAvg.timeStamp);
                poseEstimatorField.setRobotPose(poseEstimator.getEstimatedPosition());
                visionField3.setRobotPose(vDataMed.getPose());
                visionFieldavg3.setRobotPose(vDataAvg.getPose());
                lastUpdateTime = time;
                time = vDataMed.getTimeStamp();

                for (VisionData vd : buf3Med){
                    vd.clear();
                }
                for (VisionData vd : buf3Avg){
                    vd.clear();
                }
            }
        } 
    }

    public void updateRobotPose5() {
        double time = getTime();
        if (validBuf5(time)) {
            VisionData vData5Med = median(buf5Med);
            Pair<Pose2d, Double> vData5AvgPair = avg(buf5Avg);
            VisionData vDataAvg5 = new VisionData(vData5AvgPair.getFirst(), vData5AvgPair.getSecond(), testPoseEstimatorBuf5Avg)  ;
            SmartDashboard.putBoolean("updates", vData5Med != null && vData5Med.getPose() != null && vDataAvg5.getPose() != null && vDataAvg5 != null);
            if (vData5Med != null && vData5Med.getPose() != null && vDataAvg5.getPose() != null && vDataAvg5 != null) {
           
                testPoseEstimatorBuf5Med.addVisionMeasurement(vData5Med.pose, vData5Med.timeStamp);
                testPoseEstimatorBuf5Avg.addVisionMeasurement(vDataAvg5.pose, vDataAvg5.timeStamp);
                
                visionField5.setRobotPose(vData5Med.getPose());
                visionFieldavg5.setRobotPose(vDataAvg5.getPose());
                lastUpdateTime5 = time;
                time = vData5Med.getTimeStamp();

                for (VisionData vd : buf5Med){
                    vd.clear();
                }
                for (VisionData vd : buf5Avg){
                    vd.clear();
                }
                
            }
        } 
    }

    // calls the limelights to get updates and put the data in the buffer 
    private void getNewDataFromLimelightX(RaspberryPi raspberryPi) {
        //determines camera
        PhotonPoseEstimator photonPoseEstimator;
        if(raspberryPi.equals(RaspberryPi.ShooterSideRaspberry))
            photonPoseEstimator = photonPoseEstimatorForShooterSideRaspberry;
        else
            photonPoseEstimator = photonPoseEstimatorForAmpSideRaspberry;
        if (chassis.getVelocity().getNorm() <= maxValidVelcity) {
            var PhotonUpdate = photonPoseEstimator.update();
            if(PhotonUpdate != null){
                try {
                    var estimatedRobotPose = PhotonUpdate.get();
                    var estimatedPose = estimatedRobotPose.estimatedPose;
                    if(estimatedRobotPose != null){
                        VisionData newVisionData3Avg = new VisionData(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds,testPoseEstimatorBuf3Avg);
                        VisionData newVisionData5Avg = new VisionData(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds,testPoseEstimatorBuf5Avg);
                        VisionData newVisionData3Med = new VisionData(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds,testPoseEstimatorBuf3Med);
                        VisionData newVisionData5Med = new VisionData(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds,testPoseEstimatorBuf5Med);
                        if(countCycles < 100){
                            SmartDashboard.putNumber("cyclelesstan100", 1);
                            poseEstimator.resetPosition(chassis.getAngle(), chassis.getModulePositions(), newVisionData3Avg.getFalsePose());
                            
                            testPoseEstimatorBuf3Avg.resetPosition(chassis.getAngle(), chassis.getModulePositions(), newVisionData3Avg.getFalsePose());
                            testPoseEstimatorBuf3Med.resetPosition(chassis.getAngle(), chassis.getModulePositions(), newVisionData3Avg.getFalsePose());
                            testPoseEstimatorBuf5Avg.resetPosition(chassis.getAngle(), chassis.getModulePositions(), newVisionData3Avg.getFalsePose());
                            testPoseEstimatorBuf5Med.resetPosition(chassis.getAngle(), chassis.getModulePositions(), newVisionData3Avg.getFalsePose());

                            
                            // firstRun = false;
                        }
                        if (newVisionData5Avg != null && newVisionData5Avg.getPose() != null && newVisionData3Avg != null &&
                         newVisionData3Avg.getPose() != null && newVisionData5Med != null && 
                         newVisionData5Med.getPose() != null&& newVisionData3Med != null && 
                         newVisionData3Med.getPose() != null) {
                            lastData = next();
                            lastData5 = next5(); 
                            buf3Avg[lastData] = newVisionData3Avg;
                            buf5Avg[lastData5] = newVisionData5Avg;
                            buf3Med[lastData] = newVisionData3Med;
                            buf5Med[lastData5] = newVisionData5Med;
                            

                            visionField.setRobotPose(newVisionData3Avg.getPose());
                        }   
                    }
                } catch (NoSuchElementException e) {
                    //System.out.println("got exception at get new data eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
                }
            }
        }
    }


    @Override
    public void periodic() {
        super.periodic();
        countCycles++;

        getNewDataFromLimelightX(RaspberryPi.AmpSideRaspberry);
        getNewDataFromLimelightX(RaspberryPi.ShooterSideRaspberry);
        updateRobotPose();
        updateRobotPose5();
        SmartDashboard.putNumber("angle", visionField.getRobotPose().getRotation().getDegrees());
        double x = 15.513;
        double y = 4.424;
        Pose2d visionPose = visionField.getRobotPose();
        SmartDashboard.putNumber("no filter X", visionPose.getX());
        SmartDashboard.putNumber("no filter Y", visionPose.getY());
        SmartDashboard.putNumber("no filter dist",Math.sqrt(Math.pow(x-visionPose.getX(), 2) + Math.pow(y-visionPose.getY(),2) ) );
        Pose2d vision3Pose = visionField3.getRobotPose();
        SmartDashboard.putNumber("buf 3 med X", vision3Pose.getX());
        SmartDashboard.putNumber("buf 3 med Y", vision3Pose.getY());
        SmartDashboard.putNumber("buf 3 med dist",Math.sqrt(Math.pow(vision3Pose.getX()-x, 2) + Math.pow(vision3Pose.getY()-y,2) ) );
        Pose2d vision5Pose = visionField5.getRobotPose();
        SmartDashboard.putNumber("buf 5 med X", vision5Pose.getX());
        SmartDashboard.putNumber("buf 5 med Y", vision5Pose.getY());    
        SmartDashboard.putNumber("buf 5 med dist",Math.sqrt(Math.pow(vision5Pose.getX()-x, 2) + Math.pow(vision5Pose.getY()-y,2) ) );

        Pose2d vision3avgPose = visionFieldavg3.getRobotPose();
        SmartDashboard.putNumber("buf 3 avg X", vision3avgPose.getX());
        SmartDashboard.putNumber("buf 3 avg Y", vision3avgPose.getY());
        SmartDashboard.putNumber("buf 3 avg dist",Math.sqrt(Math.pow(vision3avgPose.getX()-x, 2) + Math.pow(vision3avgPose.getY()-y,2) ) );

        Pose2d vision5avgPose = visionFieldavg5.getRobotPose();
        SmartDashboard.putNumber("buf 5 avg X", vision5avgPose.getX());
        SmartDashboard.putNumber("buf 5 avg Y", vision5avgPose.getY());    
        SmartDashboard.putNumber("buf 5 avg dist",Math.sqrt(Math.pow(vision5avgPose.getX()-x, 2) + Math.pow(vision5avgPose.getY()-y,2) ) );


    }
    //util

    public double getTime() {
        return Timer.getFPGATimestamp();
    }

    public  Pair<Pose2d, Double> avg(VisionData[] visionDataArr) {

        double averageX = 0.0;
        double averageY = 0.0;
        double averageRotation = 0.0;
        double avarageTimeStamp = 0.0;

        for (VisionData vData : visionDataArr) {
            if(vData != null && vData.pose != null){
                Pose2d pose = vData.pose;
                averageX += pose.getTranslation().getX();
                averageY += pose.getTranslation().getY();
                averageRotation += pose.getRotation().getRadians();
                avarageTimeStamp += vData.timeStamp;
            }
        }

        averageX /= visionDataArr.length;
        averageY /= visionDataArr.length;
        averageRotation /= visionDataArr.length;
        avarageTimeStamp /= visionDataArr.length;
        // Create a new Pose2d with the calculated averages
        return new Pair<Pose2d, Double>((new Pose2d(averageX, averageY, new Rotation2d(averageRotation))), avarageTimeStamp);
    }

    Comparator<VisionData> comperator = new Comparator<VisionData>() {
        @Override
        public int compare(VisionData data0, VisionData data1) {
            return Double.compare(data0.getDiffrence(), data1.getDiffrence());
        }
    };
    
    public VisionData median(VisionData[] visionDataArr) {
        Arrays.sort(visionDataArr, comperator);
        return visionDataArr[visionDataArr.length / 2];
    }

    

    // BUF 3
    int next() {
        return (lastData + 1) % buf3Avg.length;
    }

    /*
     * @param bla
     * @return bal2
     */
    private boolean validBuf(double time) {
        
        double minTime = time - 1.2;
        for (VisionData vData : buf3Avg) {
            if (vData.getTimeStamp() < minTime) {
                //System.out.println(vData.getTimeStamp() + ", " + minTime + ", " + (vData.getTimeStamp() < minTime));
                return false;
            }
        }
        return true;
    }

    public double lastUpdateLatency() {
        return getTime() - lastUpdateTime;
    }

    public boolean validVisionPosition() {
        return lastUpdateLatency() < 1;
    }
    // BUF 5
    int next5() {
        return (lastData5 + 1) % buf5Avg.length;
    }
    
    private boolean validBuf5(double time) {
        double minTime = time - 2;
        // System.out.println("---------------------------");
        // for (int i = 0; i < buf5.length; i++) {
        //     VisionData visionBill = buf5[i];
        //     System.out.println(visionBill.timeStamp + ", in index:" + i);
        // }
        // System.out.println("---------------------------");
        for (VisionData vData : buf5Avg) {
            //System.out.println(vData.timeStamp);
            if (vData.getTimeStamp() < minTime) {
                // System.out.println(vData.getTimeStamp() + ", " + minTime + ", " + (vData.getTimeStamp() < minTime));
                return false;
            }
        }
        return true;
    }
   
    public double lastUpdateLatency5() {
        return getTime() - lastUpdateTime5;
    }
    
    public boolean validVisionPosition5() {
        return lastUpdateLatency5() < 1;
    }
    
    //object to save vision data that includes a pose a timestamp and the difference at that moment from the odometrey
    class VisionData {
    
        private Pose2d pose;
        private double timeStamp;
        private double diffrence; // difference than odometry
        private Pose2d falsePose;
        private double falsetimeStamp;
        private SwerveDrivePoseEstimator bufPoseEstimator;
    
        VisionData(Pose2d pose, double timeStamp, SwerveDrivePoseEstimator bufPoseEstimator) {
            this.pose = pose;
            this.timeStamp = timeStamp;
            this.falsePose = pose;
            this.falsetimeStamp = timeStamp;
            this.bufPoseEstimator = bufPoseEstimator;
            if (timeStamp < 0) {
                System.out.println("cleared at constructor, " + timeStamp);
                clear();
                
            } else {
                
                setDiffrence();
            }
        }
    
        //utils
    
        public void recalc(double time) {
            // for newer data - recalc the twsit
            if (timeStamp < time) {
                //setDiffrence();
            } else {
                System.out.println("cleared at recalc");
                clear();
            }
        }
    
        protected void setDiffrence() {

            Pose2d poseSample = this.bufPoseEstimator.getSample(timeStamp);
            if (poseSample != null
                    && Math.abs(poseSample.getRotation().minus(pose.getRotation()).getDegrees()) < maxValidAngleDiff) {
                diffrence = poseSample.getTranslation().getDistance(pose.getTranslation());
            } else {
                if(poseSample != null){
                    System.out.println("cleared on setDifference() func pose sample isnt null, " + poseSample.getRotation().getDegrees() + " " + pose.getRotation().getDegrees());
                }
                System.out.println("cleared on setDifference() func");
                clear();
            }
        }
    
        private void clear() {
            diffrence = -1;
            pose = null;
            timeStamp = 0;
            //System.out.println("Cleared.");
        }    
            
        //getters
    
        public Pose2d getPose() {
            return pose;
        }
    
        public double getTimeStamp() {
            return timeStamp;
        }
    
        public double getDiffrence() {
            return diffrence;
        }
    
        public Pose2d getFalsePose() {
            return falsePose;
        }
    
        public double getFalsetimeStamp() {
            return falsetimeStamp;
        }
    
    
        
    }

    //enum for choosing which limelight to use
    enum RaspberryPi{
        ShooterSideRaspberry,
        AmpSideRaspberry
    }
}
