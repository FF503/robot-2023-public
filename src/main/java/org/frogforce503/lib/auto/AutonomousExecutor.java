package org.frogforce503.lib.auto;

import java.lang.Thread.State;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousExecutor {

    private static AutonomousExecutor instance = null;
    public static AutonomousExecutor getInstance() {
        if (instance == null)
            instance = new AutonomousExecutor();
        return instance;
    }

    public AutonomousState currentState;
    public AutoMode selectedAutoMode;

    // private SwerveTrajectoryFollower trajectoryFollower;
    // private Behavior autonomousBehavior;
    private Action autonomousAction;

    public void periodic() {
        if (!RobotState.isAutonomous())
            this.currentState = AutonomousState.ERROR_STOPPED;
        
        switch(this.currentState) {
            case ERROR_STOPPED: {
                Swerve.getInstance().onStop();
                // pass through
            }
            case STOPPED: {
                break;
            }
            case STARTED: {
                if (selectedAutoMode != null) {
                    selectedAutoMode.onStart();
                    // trajectoryFollower = new SwerveTrajectoryFollower(selectedAutoMode.getPath());
                    // trajectoryFollower.startFollowing();
                    autonomousAction.start();
                    this.currentState = AutonomousState.RUNNING;
                } else {
                    this.currentState = AutonomousState.ERROR_STOPPED;
                }
                break;
            }
            case RUNNING: {
                if (autonomousAction.isFinished()) {
                    this.currentState = AutonomousState.DONE;
                    autonomousAction.done();
                    break;
                }
                autonomousAction.update();
                this.selectedAutoMode.onLoop();
                break;
            }
            case DONE: {
                System.out.println("Just finished");
                this.stop();
                this.selectedAutoMode.onEnd();
                break;
            }
        }

        // System.out.println("AUTONOMOUS EXEUCUTOR STATE: " +  getCurrentState().name());
    }

    public void stop() {
        this.autonomousAction = null;
        this.currentState = AutonomousState.STOPPED;
    }

    public void autonomousPeriodic() {
        // if (this.currentState == AutonomousState.STOPPED)
        //     this.currentState = AutonomousState.STARTED;
    }

    public void setSelectedAutoMode(AutoMode mode) {
        this.selectedAutoMode = mode;
        autonomousAction = selectedAutoMode.routine();
        Swerve.getInstance().resetOdometry(selectedAutoMode.getStartingPose());
    }

    public AutonomousState getCurrentState() {
        return this.currentState;
    }

    public void startAuton() {
        this.currentState = AutonomousState.STARTED;
    }

    public static enum AutonomousState {
        STARTED,
        RUNNING,
        DONE,
        STOPPED,
        ERROR_STOPPED
    }
}
