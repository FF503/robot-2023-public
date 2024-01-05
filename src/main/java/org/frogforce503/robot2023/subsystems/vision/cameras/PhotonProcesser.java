package org.frogforce503.robot2023.subsystems.vision.cameras;

import java.security.Timestamp;
import java.util.ArrayList;
import java.util.List;

import org.frogforce503.robot2023.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

public class PhotonProcesser extends Camera.AprilTagCamera {

  @Override
  public Translation2d getEstimatedRobotPose() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Transform2d getCameraToRobot() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public List<Target> getAllTargets() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public boolean isTargetVisible() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public Target getPrimaryTarget() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void onStart() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void onLoop() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void onStop() {
    // TODO Auto-generated method stub
    
  }

}
