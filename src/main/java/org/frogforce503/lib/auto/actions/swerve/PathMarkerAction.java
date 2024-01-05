package org.frogforce503.lib.auto.actions.swerve;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.auto.follower.PathMarker;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathMarkerAction implements Action {

    private Translation2d center;
    private double rangeMeters;
    private Runnable onEnter, periodic, onExit;

    private boolean hasEntered = false;
    private boolean hasExited = false;
    private boolean hadExited = false;

    public PathMarkerAction(PathMarker config) {
        this.center = config.center;
        this.rangeMeters = config.rangeMeters;
        this.onEnter = config.onEnter;
        this.periodic = config.periodic;
        this.onExit = config.onExit;
    }

    @Override
    public void start() {
        this.rangeMeters *= rangeMeters; // square for more efficient range checking
    }

    private boolean inside() {
        Translation2d pose = Swerve.getInstance().getPoseMeters().getTranslation();
        return Math.pow(center.getX() - pose.getX(), 2) + Math.pow(center.getY() - pose.getY(), 2) <= rangeMeters; // no need for sqrt
    }

    @Override
    public void update() {
        if (!hasExited) {
            if (inside()) {
                if (!this.hasEntered)
                    this.onEnter.run();
                
                this.periodic.run();

                this.hasEntered = true;
            }
            this.hasExited = this.hasEntered && !this.inside();
            if (hasExited && !hadExited)
                this.onExit.run();
            hadExited = this.hasExited;
        }
    }

    @Override
    public boolean isFinished() {
        return hasExited;
    }

    @Override
    public void done() { }
    
}
