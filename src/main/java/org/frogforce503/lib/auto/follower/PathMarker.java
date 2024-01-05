package org.frogforce503.lib.auto.follower;

import org.frogforce503.lib.trajectory.PlannedPath;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class PathMarker {
    public Translation2d center;
    public double rangeMeters;
    public Runnable onEnter, periodic, onExit;
    
    public PathMarker(Translation2d center, double rangeMeters, Runnable onEnter, Runnable periodic, Runnable onExit) {
        this.center = center;
        this.rangeMeters = rangeMeters;
        this.onEnter = onEnter;
        this.periodic = periodic;
        this.onExit = onExit;
    }

    public PathMarker(Translation2d center, double rangeMeters, Runnable onEnter) {
        this(center, rangeMeters, onEnter, () -> {}, () -> {});
    }

    public PathMarker(Trajectory.State state, double rangeMeters, Runnable onEnter, Runnable periodic, Runnable onExit) {
        this(state.poseMeters.getTranslation(), rangeMeters, onEnter, periodic, onExit);
    }

    public PathMarker(Trajectory.State state, double rangeMeters, Runnable onEnter) {
        this(state.poseMeters.getTranslation(), rangeMeters, onEnter, () -> {}, () -> {});
    }

    public PathMarker(PlannedPath path, double percentage, Runnable onEnter, Runnable periodic, Runnable onExit) {
        this(path.sample(path.getDriveTrajectory().getTotalTimeSeconds() * percentage).poseMeters.getTranslation(), 0.5, onEnter, periodic, onExit);
    }

    public PathMarker(PlannedPath path, double percentage, Runnable onEnter) {
        this(path, 0.5, onEnter, () -> {}, () -> {});
    }

    // public MarkerConfig(Translation2d center, double rangeMeters, State state) {
    //     this(center, rangeMeters, () -> StateEngine.getInstance().setState(state), () -> {}, () -> {});
    // }
}