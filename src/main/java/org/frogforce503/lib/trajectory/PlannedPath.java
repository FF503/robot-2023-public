package org.frogforce503.lib.trajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class PlannedPath {
    private Trajectory driveTrajectory;
    private RotationSequence rotationSequence;

    public PlannedPath(Trajectory driveTrajectory, RotationSequence rotationSequence) {
        this.driveTrajectory = driveTrajectory;
        this.rotationSequence = rotationSequence;
    }

    public Trajectory getDriveTrajectory() {
        return this.driveTrajectory;
    }

    public RotationSequence getRotationSequence() {
        return this.rotationSequence;
    }

    public Pose2d getInitialHolonomicPose() {
        return new Pose2d(getDriveTrajectory().getInitialPose().getTranslation(), getRotationSequence().sample(0).position);
    }

    public Pose2d getFinalHolonomicPose() {
        return new Pose2d(getDriveTrajectory().sample(getTotalTimeSeconds()).poseMeters.getTranslation(), getRotationSequence().sample(getTotalTimeSeconds()).position);
    }

    private Pair<Trajectory.State, RotationSequence.State> _sample(double timeSeconds) {
        return Pair.of(this.driveTrajectory.sample(timeSeconds), this.rotationSequence.sample(timeSeconds));
    }

    public HolonomicState sample(double timeSeconds) {
        Pair<Trajectory.State, RotationSequence.State> pair = this._sample(timeSeconds);
        return new HolonomicState(pair.getFirst(), pair.getSecond());
    }

    public double getTotalTimeSeconds() {
        return getDriveTrajectory().getTotalTimeSeconds();
    }

    public static class HolonomicState {
        // The time elapsed since the beginning of the trajectory.
        public double timeSeconds;

        // The speed at that point of the trajectory.
        public double velocityMetersPerSecond;

        // The acceleration at that point of the trajectory.
        public double accelerationMetersPerSecondSq;

        // The pose at that point of the trajectory.
        public Pose2d poseMeters;

        // The curvature at that point of the trajectory.
        public double curvatureRadPerMeter;

        public Rotation2d holonomicAngle; // which direction the robot is facing
        public double angularVelocityRadiansPerSec; // speed at which the robot is rotating

        public HolonomicState(Trajectory.State trajState, RotationSequence.State rotState) {
            this.timeSeconds = trajState.timeSeconds;
            this.velocityMetersPerSecond = trajState.velocityMetersPerSecond;
            this.accelerationMetersPerSecondSq = trajState.accelerationMetersPerSecondSq;
            this.poseMeters = trajState.poseMeters;//new Pose2d(trajState.poseMeters.getTranslation(), rotState.position);
            this.curvatureRadPerMeter = trajState.curvatureRadPerMeter;
            this.holonomicAngle = rotState.position;
            this.angularVelocityRadiansPerSec = rotState.velocityRadiansPerSec;
        }
    }
    
}
