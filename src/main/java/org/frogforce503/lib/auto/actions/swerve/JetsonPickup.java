package org.frogforce503.lib.auto.actions.swerve;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.robot2023.StateEngine;
import org.frogforce503.robot2023.RobotState.GamePiece;
import org.frogforce503.robot2023.StateEngine.RobotStates;
import org.frogforce503.robot2023.subsystems.Intake;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.SwerveControlState;
import org.frogforce503.robot2023.subsystems.vision.cameras.Jetson;

import edu.wpi.first.wpilibj.Timer;

public class JetsonPickup implements Action {

    GamePiece desiredGamePiece;
    double timeout;
    double startTime;

    public JetsonPickup() {
        this.desiredGamePiece = GamePiece.CUBE;
        this.timeout = 1.75;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        Jetson.getInstance().savePose(this.desiredGamePiece == GamePiece.CONE);
    }

    @Override
    public void update() {
        Swerve.getInstance().setControlState(SwerveControlState.FETCHING);
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > timeout;
    }

    @Override
    public void done() {
        StateEngine.getInstance().setRobotState(RobotStates.IDLE);
        Swerve.getInstance().onStop();
    }
    
}
