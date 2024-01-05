package org.frogforce503.lib.auto.actions;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.robot2023.StateEngine;

public class RobotStateAction implements Action {

    final StateEngine.RobotStates targetState;
    public RobotStateAction(StateEngine.RobotStates targetState) {
        this.targetState = targetState;
    }

    @Override
    public void start() {
        StateEngine.getInstance().setRobotState(this.targetState);
    }

    @Override
    public void update() { }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() { }
}
