package org.frogforce503.robot2023.subsystems.vision.cameras;

import org.frogforce503.lib.util.InterpolatingDouble;
import org.frogforce503.lib.util.InterpolatingTreeMap;
import org.frogforce503.lib.util.TaskScheduler;
import org.frogforce503.robot2023.Constants;
import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.StateEngine;
import org.frogforce503.robot2023.RobotState.GamePiece;
import org.frogforce503.robot2023.StateEngine.RobotStates;
import org.frogforce503.robot2023.subsystems.Intake;
import org.frogforce503.robot2023.subsystems.Intake.IntakeStates;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;

import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Jetson {  


    private static Jetson instance = null;

    // focalLength = objPX * calibrationDistance / objlength
    // Arducam 5MP
    // xFov = 90, imW = 1920, imH = 1080

    // values required by in-built functions
    double coneWidth, coneHeight, cubeWidth, cubeHeight, coneBase, cubeBase; 
    int classIDCone, classIDCube, classIDMain, numDetections;
    boolean cubeExists, coneExists;
    double centerXCube, centerXCone;
    double centerY;
    double TX;
    double targetPoseX;
    double targetPoseY;
    double targetTranslationX;
    double targetTranslationY;
    Pose2d savedFieldRelativePose, targetPose;

    // constants acquired from robot hardware
    double kConeFocalLength, kCubeFocalLength, kxFov, kImageH, kImageW;

    // network tables setup stuff
    NetworkTableInstance inst;
    NetworkTable coneTable, cubeTable, mainTable, storedVals;
    
    DoubleSubscriber coneWidthSub, coneHeightSub, cubeWidthSub, cubeHeightSub, numDetectionsSub, coneCenterXSub, cubeCenterXSub, targetTranslationXEntry, targetTranslationYEntry;
    DoublePublisher distancePub, txPub, targetTranslationXPub, targetTranslationYPub;
    BooleanSubscriber coneExistsSub, cubeExistsSub;

    public static Jetson getInstance() {
        return instance == null ? instance = new Jetson() : instance;
    }

    public Jetson() {
        inst = NetworkTableInstance.getDefault();
        mainTable = inst.getTable("jetson").getSubTable("targets");
        storedVals = mainTable.getSubTable("stored values");
        coneTable = mainTable.getSubTable("cone"); 
        cubeTable = mainTable.getSubTable("cube");
        
        numDetectionsSub = mainTable.getDoubleTopic("NumDetections").subscribe(0.0);

        // Publish Tx, Distance, Translations, and Distance
        txPub = storedVals.getDoubleTopic("TX").publish();
        targetTranslationYPub = storedVals.getDoubleTopic("Target Translation Y").publish();
        distancePub = storedVals.getDoubleTopic("Distance").publish();
        targetTranslationXPub = storedVals.getDoubleTopic("Target Translation X").publish();

        // Setup Subscribers
        coneWidthSub = coneTable.getDoubleTopic("Width").subscribe(0.0);
        coneHeightSub = coneTable.getDoubleTopic("Height").subscribe(0.0);
        coneCenterXSub = coneTable.getDoubleTopic("CenterXpos").subscribe(0.0);

        cubeWidthSub = cubeTable.getDoubleTopic("Width").subscribe(0.0);
        cubeHeightSub = cubeTable.getDoubleTopic("Height").subscribe(0.0);
        cubeCenterXSub = cubeTable.getDoubleTopic("CenterXpos").subscribe(0.0);

        coneExistsSub = coneTable.getBooleanTopic("ConeDetections").subscribe(false);
        cubeExistsSub = cubeTable.getBooleanTopic("CubeDetections").subscribe(false);

        // Get Constants
        kConeFocalLength = Robot.bot.kConeFocalLength;
        kCubeFocalLength = Robot.bot.kCubeFocalLength;
        kImageH = Robot.bot.kImageH;
        kImageW = Robot.bot.kImageW;
        kxFov = Robot.bot.kxFov;

    }

    public double getDistanceFromFocalLength(double pixels, boolean targetingCone) {
        double dist = targetingCone ? 
        (kConeFocalLength * Constants.CONE_SQUARE_LENGTH) / pixels : 
        (kCubeFocalLength * Constants.CUBE_SIDE_LENGTH) / pixels;
        distancePub.set(dist);
        return dist;
    }

    public double getTX(boolean targetingCone){
        double tx = 0;
        
        if (targetingCone){
            tx = ((centerXCone - (kImageW/2))*((kxFov/2)/(kImageW/2)));
            txPub.set(tx);
        }
        else 
        {
            tx = ((centerXCube- (kImageW/2))*((kxFov/2)/(kImageW/2)));
            txPub.set(tx);
        }
        return tx;

    }

    public double getYTranslation(double pixels, boolean targetingCone){
        double yTrnsl = targetingCone ? 
            getDistanceFromFocalLength(pixels, true)*Math.cos(Math.toRadians(getTX(targetingCone))) : 
            getDistanceFromFocalLength(pixels, false)*Math.cos(Math.toRadians(getTX(targetingCone)));
        targetTranslationYPub.set(Units.metersToInches(yTrnsl));
        return yTrnsl;
    }

    public double getXTranslation(double pixels, boolean targetingCone){
        double xTrnsl = targetingCone ? 
            getDistanceFromFocalLength(pixels, true) * Math.sin(Math.toRadians(getTX(targetingCone))) : 
            getDistanceFromFocalLength(pixels, false) * Math.sin(Math.toRadians(getTX(targetingCone)));
        targetTranslationXPub.set(Units.metersToInches(xTrnsl));
        return xTrnsl;
    }

    /**
    Pose to target for specified object, finds closest
    @param targetingCone If targeting cone. cone = true, cube = false 
    @return Pose to target
    */
    public Pose2d getPoseToTarget(boolean targetingCone) {
        if (targetingCone){
            coneWidth = coneWidthSub.get();
            coneHeight = coneHeightSub.get();
            centerXCone = coneCenterXSub.get();
            coneExists = coneExistsSub.get();
        }
        else{
            cubeWidth = cubeWidthSub.get();
            cubeHeight = cubeHeightSub.get();
            centerXCube = cubeCenterXSub.get();
            cubeExists = cubeExistsSub.get();
        }

        // X is forward, Y is different directions (robot relative) yessir
        if (targetingCone && coneExists) {
            coneBase = Math.min(coneWidth, coneHeight);
            targetPose = new Pose2d(getYTranslation(coneBase, true), -getXTranslation(coneBase, true), new Rotation2d(0));
            return targetPose;
        }
        else if (cubeExists && !targetingCone) {
            cubeBase = Math.min(cubeWidth, cubeHeight);
            targetPose = new Pose2d(getYTranslation(cubeBase, false), -getXTranslation(cubeBase, false), new Rotation2d(0));
            return targetPose;
        } else {
            return new Pose2d(0,0, new Rotation2d());
        }
    }

    /**
    Pose to target for any object, finds closest
    @return Pose to Target
    */
    public Pose2d getPoseToTarget() {
        numDetections = (int) numDetectionsSub.get();
        
        cubeExists = cubeExistsSub.get();
        coneExists = coneExistsSub.get();

        double coneDistance = 0;
        double cubeDistance = 0;

        if (coneExists){
            coneWidth = coneWidthSub.get();
            coneHeight = coneHeightSub.get();
            centerXCone = coneCenterXSub.get();
            coneBase = Math.min(coneWidth, coneHeight);
            coneDistance = getDistanceFromFocalLength(coneBase, true);
        }
        if (cubeExists){
            cubeWidth = cubeWidthSub.get();
            cubeHeight = cubeHeightSub.get();
            centerXCube = cubeCenterXSub.get();
            cubeBase = Math.min(cubeWidth, cubeHeight);
            cubeDistance = getDistanceFromFocalLength(cubeBase, false);
        }

        if (coneDistance < cubeDistance && coneExists || !cubeExists && coneExists){
            targetPose = new Pose2d(getYTranslation(coneBase, true), -getXTranslation(coneBase, true), new Rotation2d(0));
            return targetPose;
        }
        else if (cubeDistance < coneDistance && cubeExists || cubeExists && !coneExists){
            targetPose = new Pose2d(getYTranslation(cubeBase, false), -getXTranslation(cubeBase, false), new Rotation2d(0));
            return targetPose;
        }
        else{
            return new Pose2d(0,0, new Rotation2d());
        }
    }
    
    public void savePoseFromPreference() {
        savePose(Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE);
    }

    /**
    Save a snapshot to the object for pose
    */
    public void savePose(boolean targetingCone){
        // convert to field relative and drive to object
        StateEngine.getInstance().setRobotState(RobotStates.INTAKING);

        Transform2d cameraToRobot = new Transform2d(new Translation2d(Units.inchesToMeters(10), 0), new Rotation2d());
        Transform2d robotToField = new Transform2d(new Pose2d(), Swerve.getInstance().getPoseMeters());

        Translation2d robotToTarget = getPoseToTarget(targetingCone).plus(cameraToRobot).getTranslation();
        Translation2d robotToTargetInField = robotToTarget.rotateBy(Swerve.getInstance().getAngleRotation2d());
        
        Pose2d targetToField = new Pose2d(robotToTargetInField, new Rotation2d()).plus(robotToField);
        savedFieldRelativePose = targetToField;
                
        Swerve.getInstance().getField().getObject("CONE").setPose(savedFieldRelativePose);
    }

    /**
    Save a snapshot to the object for pose for closest object
    */
    public void savePose(){
        StateEngine.getInstance().setRobotState(RobotStates.INTAKING);

        // convert to field relative and drive to object
        Transform2d cameraToRobot = new Transform2d(new Translation2d(Units.inchesToMeters(10), 0), new Rotation2d());
        Transform2d robotToField = new Transform2d(new Pose2d(), Swerve.getInstance().getPoseMeters());

        Translation2d robotToTarget = getPoseToTarget().plus(cameraToRobot).getTranslation();
        Translation2d robotToTargetInField = robotToTarget.rotateBy(Swerve.getInstance().getAngleRotation2d());
        
        Pose2d targetToField = new Pose2d(robotToTargetInField, new Rotation2d()).plus(robotToField);
        savedFieldRelativePose = targetToField;
                
        Swerve.getInstance().getField().getObject("CONE").setPose(savedFieldRelativePose);
    }

    public Pose2d getSavedPose(){
        return savedFieldRelativePose;
    }

    public enum JetsonState {
        OFF, IDLE, FETCHING_NEAR, FETCHING_CONE, FETCHING_CUBE, 
    }

}
