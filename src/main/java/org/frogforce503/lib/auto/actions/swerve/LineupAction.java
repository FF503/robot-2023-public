package org.frogforce503.lib.auto.actions.swerve;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.robot2023.planners.LineupPlanner;
import org.frogforce503.robot2023.planners.LineupPlanner.ScoringHeight;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.SwerveControlState;

public class LineupAction implements Action {

    private final int[] scoringLocation;
    
    public LineupAction(int grid, int cell) {
        this.scoringLocation = new int[] {grid, cell};
    }

    boolean hasStarted = false;

    @Override
    public void start() {  }

    @Override
    public void update() {
        if (Swerve.getInstance().getControlState() == SwerveControlState.LINEUP)
            hasStarted = true;
        
        LineupPlanner.getInstance().alignTo(this.scoringLocation[0], this.scoringLocation[1], ScoringHeight.HIGH);
    }

    @Override
    public boolean isFinished() {
        return hasStarted && Swerve.getInstance().getControlState() != SwerveControlState.LINEUP;
    }

    @Override
    public void done() {
        LineupPlanner.getInstance().stopTrying();
    }
    
}
