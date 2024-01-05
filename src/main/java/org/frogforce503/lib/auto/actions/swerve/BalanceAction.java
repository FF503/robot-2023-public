package org.frogforce503.lib.auto.actions.swerve;

import java.awt.event.ActionEvent;
import java.beans.PropertyChangeListener;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.robot2023.planners.ParkPlanner;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.ModuleSnapPositions;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.SwerveControlState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class BalanceAction implements Action{

    boolean hasStarted = false;

    Timer timer;

    public BalanceAction() {
        timer = new Timer();
    }
    
    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {
        if (Swerve.getInstance().getControlState() == SwerveControlState.PARK)
            hasStarted = true;
        
        ParkPlanner.getInstance().alignTo();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(4.0) || (hasStarted && Swerve.getInstance().getControlState() != SwerveControlState.PARK);
    }

    @Override
    public void done() {
        ParkPlanner.getInstance().stopTrying();
    }
    


}
