package org.frogforce503.lib.auto.actions.base;

import java.util.concurrent.locks.Condition;
import java.util.function.BooleanSupplier;

import org.frogforce503.robot2023.StateEngine;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new WaitAction(your_time))
 */
public class WaitUntil implements Action {

    BooleanSupplier condition;
    boolean forceStopped = false;

    public WaitUntil(BooleanSupplier cond) {
        this.condition = cond;
    }

    public static WaitUntil stateReached(StateEngine.RobotStates robotState) {
        return new WaitUntil(() -> {
            return robotState == StateEngine.getInstance().getRobotState();
        });
    }

    public static WaitUntil timeElapsed(double time) {
        WaitUntil action = new WaitUntil(() -> false) {
            Timer timer = new Timer();
            @Override
            public void start() {
                this.condition = () -> timer.hasElapsed(time);
                timer.start();
            }
        };

        return action;
    }

    @Override
    public void start() { }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return this.forceStopped || this.condition.getAsBoolean();
    }

    @Override
    public void done() { forceStopped = true; }
}