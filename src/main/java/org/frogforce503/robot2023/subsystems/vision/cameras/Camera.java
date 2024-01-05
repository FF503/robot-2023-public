package org.frogforce503.robot2023.subsystems.vision.cameras;

import java.util.List;

import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.subsystems.Subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class Camera extends Subsystem {
    public abstract Transform2d getCameraToRobot();

    public abstract List<Target> getAllTargets();
    public abstract boolean isTargetVisible();
    public abstract Target getPrimaryTarget();
    

    public static class Target {
        private TargetType type;
        public Pose2d robotRelativePose; // maybe make into pose3d if useful? 
        public int id;

        public Target(TargetType type, Pose2d robotRelativePose, int id) {
            this.type = type;
            this.robotRelativePose = robotRelativePose;
            this.id = id;
        }

        public Target(TargetType type, Pose2d robotRelativePose) {
            this(type, robotRelativePose, -1);
        }

        public TargetType getType() {
            return type;
        }

        public void setType(TargetType type) {
            this.type = type;
        }

        public Pose2d getRobotRelativePose() {
            return robotRelativePose;
        }

        public void setRobotRelativePose(Pose2d robotRelativePose) {
            this.robotRelativePose = robotRelativePose;
        }

    }

    public static abstract class AprilTagCamera extends Camera {
        public abstract Translation2d getEstimatedRobotPose();
        public void initializeFieldLayout(AllianceColor color) {}
    }
   
    public enum TargetType {
        APRILTAG, CONE, CUBE;
    }
}
