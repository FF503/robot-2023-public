package org.frogforce503.robot2023.subsystems.vision.cameras;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.fields.util.TagLocation;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends Camera.AprilTagCamera {

    List<Target> targets;

    private NetworkTable limelightTable;
    private JSONParser jsonParser;
    private final String DETECTOR_DEFAULT_JSON = "{\"Results\":{\"Classifier\":[],\"Detector\":[],\"Fiducial\":[],\"Retro\":[],\"pID\":0.0,\"tl\":0.0,\"ts\":0.0,\"v\":1}}";
    
    private static Limelight instance = null;

    private Field2d field;

    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        jsonParser = new JSONParser();
        targets = Arrays.asList();

        field = Swerve.getInstance().getField();
    }

    public static Limelight getInstance() {
        if (instance == null)
            instance = new Limelight();
        return instance;
    }

    @Override
    public Transform2d getCameraToRobot() {
        // 17.75 from front, 15.25 from left
        // Translation2d disp = new Translation2d(
        //     (Robot.bot.fullRobotLength/2 - Units.inchesToMeters(17.75)),
        //     (Robot.bot.fullRobotWidth/2 - Units.inchesToMeters(15.25))
        // );
        return new Transform2d(new Translation2d(), new Rotation2d());
    }

    @Override
    public boolean isTargetVisible() {
        return getAllTargets().size() > 0;
    }

    @Override
    public List<Target> getAllTargets() {
        return this.targets;
    }

    @Override
    public Target getPrimaryTarget() {
        return isTargetVisible() ? getAllTargets().get(0) : null;
    }

    @Override
    public Translation2d getEstimatedRobotPose() {
        if (!isTargetVisible())
            return null;
        
        Target detectedTag = getPrimaryTarget();
        Translation2d robotStraightToTag = detectedTag.getRobotRelativePose().getTranslation().rotateBy(Swerve.getInstance().getAngleRotation2d());
        TagLocation tag = FieldConfig.getInstance().getTagById(detectedTag.id);
        
        return tag.minus(robotStraightToTag);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Number of Detected AprilTags", getAllTargets().size());
        for (Target tag : getAllTargets()) {
            Transform2d robotToField = new Transform2d(new Pose2d(), Swerve.getInstance().getPoseMeters());
            Pose2d fieldRelativeTargetPose = tag.robotRelativePose.plus(robotToField);

            field.getObject("APRILTAG_" + tag.id).setPose(fieldRelativeTargetPose);

            SmartDashboard.putNumberArray("APRILTAG_" + tag.id, new double[] {tag.robotRelativePose.getX(), tag.robotRelativePose.getY()});
        }

        if (isTargetVisible()) {
            Translation2d robotPoint = getEstimatedRobotPose();
            field.getObject("Vision Robot").setPose(new Pose2d(robotPoint, Swerve.getInstance().getAngleRotation2d()));

            SmartDashboard.putNumber("Vision Robot X", robotPoint.getX());
            SmartDashboard.putNumber("Vision Robot Y", robotPoint.getY());
        }
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
        String jsonString = limelightTable.getEntry("json").getString(DETECTOR_DEFAULT_JSON);
        List<Target> detectedTargets = new ArrayList<Target>();

        try {
            JSONObject llResults =  (JSONObject) ((JSONObject) jsonParser.parse(jsonString)).get("Results");
            JSONArray aprilTags = (JSONArray) llResults.get("Fiducial");
            
            if (!aprilTags.isEmpty()) {
                Iterator<JSONObject> iterator = aprilTags.iterator();
                while(iterator.hasNext()) {
                    JSONObject tag = (JSONObject) iterator.next();
                   
                    int tagID = Math.toIntExact((Long) tag.get("fID"));

                    if (tagID > 8)
                        continue;
                    
                    JSONArray coords = (JSONArray) tag.get("t6t_cs");

                    if (!coords.isEmpty()) {
                        // camera space
                        double x = (double) coords.get(0);
                        double y = (double) coords.get(1);
                        double z = (double) coords.get(2);
                        double ry = (double) coords.get(4);
                      
                        // robot space
                        Pose3d targetPose = new Pose3d(z, -x, -y, new Rotation3d(0, 0, -ry));
                        Pose2d robotRelativeTargetPose = targetPose.toPose2d().plus(getCameraToRobot());

                        Target tagTarget = new Target(TargetType.APRILTAG, robotRelativeTargetPose, tagID);
                        detectedTargets.add(tagTarget);
                        
                    }
                }
            }

        } catch (ParseException e) {
            System.out.println("PARSING NETWORKTABLES JSON FAILED");
            e.printStackTrace();
        }

        this.targets = detectedTargets;

        if (isTargetVisible()) {
            Translation2d point = getEstimatedRobotPose();
            if (point != null) {
                Swerve.getInstance().acceptVisionMeasurement(getEstimatedRobotPose(), Timer.getFPGATimestamp());
            }
        }
    }

    @Override
    public void onStop() {  }
}
