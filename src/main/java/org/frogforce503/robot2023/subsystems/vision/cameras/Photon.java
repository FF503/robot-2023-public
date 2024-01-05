package org.frogforce503.robot2023.subsystems.vision.cameras;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.auto.AutoUtil;
import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.fields.util.TagLocation;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.SwerveControlState;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Photon extends Camera.AprilTagCamera {

    List<Target> targets;
    
    private PhotonCamera frontCamera;
    private PhotonCamera backCamera;
    private PhotonPipelineResult latestResult;
    private double lastTimestamp;
    private static Photon instance = null;

    private Optional<EstimatedRobotPose> newestPoseResult;
    private Optional<EstimatedRobotPose> lastPoseResult = Optional.empty();
    private double secondsElapsed = 0;

    private Field2d field;

    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator frontCameraPoseEstimator;
    private PhotonPoseEstimator backCameraPoseEstimator;

    private double MINIMUM_TRUST_DISTANCE = 6.0;

    NetworkTableInstance inst;
    NetworkTable photonFrontCameraTable, photonBackCameraTable, storedValsFront, storedValsBack, photonVisionTable;
    DoublePublisher ambigiutyFrontPub, ambigiutyBackPub, distancePub;
    DoubleSubscriber ambigiutyFrontSub, ambigiutyBackSub, distanceSub;

    //x = 4.7 

    public Photon() {
        targets = Arrays.asList();
        frontCamera = new PhotonCamera("camera");
        backCamera = new PhotonCamera("back_camera");
        frontCamera.setDriverMode(false);
        backCamera.setDriverMode(false);

        field = Swerve.getInstance().getField();

        inst = NetworkTableInstance.getDefault();
        photonVisionTable = inst.getTable("photonvision");
        photonFrontCameraTable = photonVisionTable.getSubTable("camera");
        photonBackCameraTable = photonVisionTable.getSubTable("back_camera");
        storedValsFront = photonVisionTable.getSubTable("storedValsFront");
        storedValsBack = photonVisionTable.getSubTable("storedValsBack");


        ambigiutyFrontPub = storedValsFront.getDoubleTopic("ambigiuty").publish();
        ambigiutyBackPub = storedValsBack.getDoubleTopic("ambigiuty").publish();


        ambigiutyFrontSub = storedValsFront.getDoubleTopic("ambigiuty").subscribe(0.0);
        ambigiutyBackSub = storedValsBack.getDoubleTopic("ambigiuty").subscribe(0.0);

        distancePub = photonVisionTable.getDoubleTopic("distance").publish();
        distanceSub = photonVisionTable.getDoubleTopic("distance").subscribe(2.75);
        // robotToCamera = this.getCameraToRobot();
        // from backcamera
    }

    public static Photon getInstance() {
        if (instance == null)
            instance = new Photon();
        return instance;
    }

    public void initBackCamera(){
        Notifier n = new Notifier(() -> {

            System.out.println("SETTING PIPELINE");
            backCamera.setPipelineIndex(3);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            System.out.println("SETTING PIPELINE 50%");

            backCamera.setPipelineIndex(4);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            backCamera.setPipelineIndex(2);


            System.out.println("SETTING PIPELINE DONE");
            
        });

        n.startSingle(0.0);
    
        
    }
    @Override
    public void initializeFieldLayout(AllianceColor color) {
        List<AprilTag> colorTags = new ArrayList<>();

        Rotation2d zero = new Rotation2d();

        Transform3d tagFromGround = new Transform3d(new Translation3d(0, 0, Units.inchesToMeters(18)), new Rotation3d());
        Transform3d hp_tagFromGround = new Transform3d(new Translation3d(0, 0, Units.inchesToMeters(18 + 9.15)), new Rotation3d());

        if (color == AllianceColor.BLUE) {
            colorTags.add(
                new AprilTag(8, (new Pose3d(new Pose2d(FieldConfig.getInstance().TAG_8, zero))).plus(tagFromGround))
            );
            colorTags.add(
                new AprilTag(7, (new Pose3d(new Pose2d(FieldConfig.getInstance().TAG_7, zero))).plus(tagFromGround))
            );
            colorTags.add(
                new AprilTag(6, (new Pose3d(new Pose2d(FieldConfig.getInstance().TAG_6, zero))).plus(tagFromGround))
            );
            colorTags.add(
                new AprilTag(5, (new Pose3d(new Pose2d(FieldConfig.getInstance().TAG_5, zero))).plus(hp_tagFromGround))
            );
        } else {
            colorTags.add(
                new AprilTag(1, (new Pose3d(new Pose2d(FieldConfig.getInstance().TAG_1, zero))).plus(tagFromGround))
            );
            colorTags.add(
                new AprilTag(2, (new Pose3d(new Pose2d(FieldConfig.getInstance().TAG_2, zero))).plus(tagFromGround))
            );
            colorTags.add(
                new AprilTag(3, (new Pose3d(new Pose2d(FieldConfig.getInstance().TAG_3, zero))).plus(tagFromGround))
            );
            colorTags.add(
                new AprilTag(4, (new Pose3d(new Pose2d(FieldConfig.getInstance().TAG_4, zero))).plus(hp_tagFromGround))
            );
        }

        for (AprilTag tag : colorTags) {
            Swerve.getInstance().getField().getObject("TAG_" + tag.ID).setPose(tag.pose.toPose2d());
        }

        fieldLayout = new AprilTagFieldLayout(colorTags, FieldConfig.getInstance().getFieldDimensions().getX(), FieldConfig.getInstance().getFieldDimensions().getY());
        
        
        frontCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, frontCamera, getFrontCameraToRobot());
        backCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, backCamera, getBackCameraToRobot());

        frontCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
        backCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

        MINIMUM_TRUST_DISTANCE = FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT.plus(FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT).times(0.5).getX();
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose, boolean usingFrontCamera) {
        PhotonPoseEstimator poseEstimator = usingFrontCamera ? frontCameraPoseEstimator : backCameraPoseEstimator;

        if (poseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        poseEstimator.setLastPose(prevEstimatedRobotPose);

        return poseEstimator.update();
    }

    private Transform3d getFrontCameraToRobot() {
        return new Transform3d(new Translation3d(Units.inchesToMeters(4.25), Units.inchesToMeters(-11.25), Units.inchesToMeters(31.75)), new Rotation3d(0,0,0));
    }

    private Transform3d getBackCameraToRobot() {
        return new Transform3d(new Translation3d(Units.inchesToMeters(3.25), Units.inchesToMeters(-0.25), Units.inchesToMeters(37.0)), new Rotation3d(0,0, Math.PI));
    }

    @Override
    public Transform2d getCameraToRobot() {
        // 17.75 from front, 15.25 from left
        // Translation2d disp = new Translation2d(
        //     (Robot.bot.fullRobotLength/2 - Units.inchesToMeters(17.75)),
        //     (Robot.bot.fullRobotWidth/2 - Units.inchesToMeters(15.25))
        // );
        // return new Transform2d(new Translation2d(), new Rotation2d());
        return new Transform2d(new Translation2d(Units.inchesToMeters(4.75), Units.inchesToMeters(-11.25)), new Rotation2d());

    }
    @Override
    public boolean isTargetVisible() {
        if (this.latestResult == null)
            return false;
        
        boolean res = false;

        try {
            res = this.latestResult.hasTargets();
        } catch (Exception e) {
            System.out.println("Photonvision error");
            System.out.println(e);
        }

        return res;
    }

    @Override
    public List<Target> getAllTargets() {
        return this.targets;
    }
    
    // public void switchPipeline(int index)

    @Override
    public Target getPrimaryTarget() {
        if (isTargetVisible()) {
            return photonTargetToBaseTarget(this.latestResult.getBestTarget());
        }
        return null;
    }

    @Override
    public Translation2d getEstimatedRobotPose() {
        if (!isTargetVisible())
            return null;
        
        Target detectedTag = getPrimaryTarget();
        Translation2d robotStraightToTag = detectedTag.getRobotRelativePose().getTranslation().rotateBy(Swerve.getInstance().getAngleRotation2d());
        TagLocation tag = FieldConfig.getInstance().getTagById(detectedTag.id);

        if (tag == null)
            return null;
        
        return tag.minus(robotStraightToTag);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Number of Detected AprilTags", getAllTargets().size());
        // for (Target tag : getAllTargets()) {
        //     // Transform2d robotToField = new Transform2d(new Pose2d(), Swerve.getInstance().getPoseMeters());
        //     // Pose2d fieldRelativeTargetPose = tag.robotRelativePose.plus(robotToField);

        //     // field.getObject("APRILTAG_" + tag.id).setPose(fieldRelativeTargetPose);

        //     SmartDashboard.putNumberArray("AAPRILTAG_" + tag.id, new double[] {tag.robotRelativePose.getX(), tag.robotRelativePose.getY(), tag.robotRelativePose.getRotation().getDegrees()});
        // }

        // if (isTargetVisible()) {
        //     Translation2d robotPoint = getEstimatedGlobalPose(null)

        //     if (robotPoint != null) {
        //         field.getObject("Vision Robot").setPose(new Pose2d(robotPoint, Swerve.getInstance().getAngleRotation2d()));

        //         SmartDashboard.putNumber("Vision Robot X", robotPoint.getX());
        //         SmartDashboard.putNumber("Vision Robot Y", robotPoint.getY());
        //     }
        // }
    }

    @Override
    public void onStart() {  }

    /**
         * from limelight docs
         * 
         * 
         Limelight Camera Space:

        3d Cartesian Coordinate System with (0,0,0) at the camera lens.

        X+ → Pointing to the right (if you were to embody the camera)

        Y+ → Pointing downward
    
        + → Pointing out of the camera

        https://docs.limelightvision.io/en/latest/json_dump.html

     */

    @Override
    public void onLoop() {

        boolean usingFrontCamera = /*edu.wpi.first.wpilibj.RobotState.isAutonomous() ? AutoUtil.frontCam :*/ useFrontCamera(); //validResults(frontCamera.getLatestResult());

        SmartDashboard.putBoolean("Vision Pose Estimator Using Front Camera", usingFrontCamera);
        // if (AutoUtil.useFrontOnly)
        //     usingFrontCamera = true;

        this.latestResult = (usingFrontCamera ? frontCamera : backCamera).getLatestResult();

        // List<PhotonTrackedTarget> photonTargets = latestResult.getTargets();

        // ArrayList<Target> detectedTargets = new ArrayList<>();

        // if (latestResult.hasTargets()) {
        //     for (PhotonTrackedTarget target : photonTargets) {
        //         detectedTargets.add(photonTargetToBaseTarget(target));
        //     }
        // }

        // this.targets = detectedTargets;

        // if (isTargetVisible()) {
        if (Math.abs(Swerve.getInstance().getPitch()) < 2.0) {
            newestPoseResult = getEstimatedGlobalPose(Swerve.getInstance().getPoseMeters(), usingFrontCamera);
            if (newestPoseResult.isPresent()) {
                EstimatedRobotPose newPose = newestPoseResult.get();
                
                if (lastPoseResult.isPresent()){
                    boolean reject = newPose.timestampSeconds - lastPoseResult.get().timestampSeconds < 0.5 && 
                        newPose.estimatedPose.getTranslation().getDistance(newestPoseResult.get().estimatedPose.getTranslation()) > distanceSub.get();
                    
                    if (!reject){
                        Swerve.getInstance().acceptVisionMeasurement(newPose.estimatedPose.toPose2d(), newPose.timestampSeconds);
                        field.getObject("Vision Robot").setPose(new Pose2d(newPose.estimatedPose.toPose2d().getTranslation(), newPose.estimatedPose.toPose2d().getRotation()));
                        lastPoseResult = newestPoseResult;
                    }                
                } else {
                    lastPoseResult = newestPoseResult;
                }
                // Swerve.getInstance().acceptVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);

            }
        }
        // }
    }

    private boolean useFrontCamera() {
        double angle = Swerve.getInstance().getAngleRotation2d().getRadians();
        return !(angle < Math.PI/2 && angle > -Math.PI/2);
    }

    private boolean validResults(PhotonPipelineResult result) {

        if (!result.hasTargets())
            return false;
        
        boolean allGood = true;
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (!FieldConfig.isMyAlliance(target.getFiducialId()))
                allGood = false;
        }

        return allGood;
    }

    private List<PhotonTrackedTarget> filterResults(PhotonPipelineResult result){
        ArrayList<PhotonTrackedTarget> targetList = new ArrayList<PhotonTrackedTarget>();
        for (int i = 0; i < result.targets.size(); i++){
            if (result.targets.get(i).getPoseAmbiguity() < distanceSub.get()){
                targetList.add(result.targets.get(i));
            }
        }
        return targetList;
    }
    private Target photonTargetToBaseTarget(PhotonTrackedTarget target) {
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        
        return new Target(TargetType.APRILTAG, new Pose2d(bestCameraToTarget.getX(), bestCameraToTarget.getY(), new Rotation2d()), target.getFiducialId());
    }

    @Override
    public void onStop() {  }
}
