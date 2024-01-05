package org.frogforce503.lib.auto.actions;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.robot2023.StateEngine;

import edu.wpi.first.wpilibj.Timer;

public class ContinuousRobotStateAction implements Action {

    final StateEngine.RobotStates targetState;
    double time;
    Timer timer;

    public ContinuousRobotStateAction(StateEngine.RobotStates targetState, double time) {
        this.targetState = targetState;
        this.time = time;
        timer = new Timer();
    }

    @Override
    public void start() {
        StateEngine.getInstance().setRobotState(this.targetState);
        timer.start();
    }

    @Override
    public void update() {
        StateEngine.getInstance().setRobotState(this.targetState);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

    @Override
    public void done() { }
}
