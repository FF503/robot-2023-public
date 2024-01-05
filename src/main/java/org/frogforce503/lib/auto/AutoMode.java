package org.frogforce503.lib.auto;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
public abstract class AutoMode {
    private boolean shouldBalance = false;
    
    public abstract Action routine();
    public abstract PlannedPath getPath();

    public void onStart() {}
    public void onLoop() {}
    public void onEnd() {}

    public String getName() {
        return this.getClass().getSimpleName();
    }

    public Pose2d getStartingPose() {
        PlannedPath p = this.getPath();
        return (p != null ? p.getInitialHolonomicPose() : Swerve.getInstance().getPoseMeters());
    }

    public void setShouldBalance(boolean shouldBalance) {
        this.shouldBalance = shouldBalance;
    }

    protected boolean shouldBalance() {
        return this.shouldBalance;
    }
}
