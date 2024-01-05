package org.frogforce503.robot2023.planners;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PickupPlanner {
    private static PickupPlanner instance;
    public static PickupPlanner getInstance() {
        if (instance == null)
            instance = new PickupPlanner();
        return instance;
    }

    private PickupState currentPickupState;
    private Translation2d lastCommanded;

    public PickupPlanner() {
        this.currentPickupState = PickupState.SEES_NOTHING;
    }

    public ChassisSpeeds update() {
        Translation2d translation = new Translation2d();
        double rx = 0;

        switch (this.currentPickupState) {
            case FETCHING: {
                // run PID off of jetson and stuff, set translation based off of that
                // continue until object not seen anymore, then transition to DRIVE_OVER
                break;
            }
            case DRIVE_OVER: {
                translation = this.lastCommanded;
                // keep driving at our last speed until beam break is triggered
                break;
            }
            case DONE: 
            case SEES_NOTHING:
            default:
        }

        this.lastCommanded = translation;
        return new ChassisSpeeds(translation.getX(), translation.getY(), rx); // robot-centric driving
    }

    
    public enum PickupState {
        SEES_NOTHING, FETCHING, DRIVE_OVER, DONE
    }
}
