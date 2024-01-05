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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroBalanceAction implements Action{

    ChassisSpeeds balanceSpeed;
    PIDController balancePID;
    boolean hasStarted = false;
    boolean hasPeaked = false;
    double lastAngle = 0;
    Timer timer;

    public GyroBalanceAction() {
        balanceSpeed = new ChassisSpeeds();
        balancePID = new PIDController(0.15, 0, 0.015);
        balancePID.setTolerance(0);
        timer = new Timer();
    }
    
    @Override
    public void start() {
        balancePID.setSetpoint(0);
        Swerve.getInstance().setControlState(SwerveControlState.PATH_FOLLOWING);
        timer.start();
        lastAngle = Swerve.getInstance().getPitchDegrees();
    }

    @Override
    public void update() {
        System.out.println("BALANCING");
        double out = balancePID.calculate(Swerve.getInstance().getPitchDegrees()) * 0.22;

        // double out = balancePID.calculate(Swerve.getInstance().getPitchDegrees()) * (hasPeaked ? -1.25 : 0.75);
        
        // if (Math.abs(Swerve.getInstance().getPitchDegrees()) > 15)
        //     hasStarted = true;
        // else if (!hasStarted)
        //     out = 3.0;

        // if (hasStarted && Math.abs(Swerve.getInstance().getPitchDegrees()) < 10)
        //     hasPeaked = true;

        
        balanceSpeed.vxMetersPerSecond = out;


        Swerve.getInstance().setPathFollowerSpeeds(balanceSpeed);
    }

    @Override
    public boolean isFinished() {

        // boolean done = hasPeaked && Math.abs(balancePID.getPositionError()) < 5;
        boolean done = ((/*Math.abs(lastAngle) - Math.abs(Swerve.getInstance().getPitch()) > 3) || */Math.abs(balancePID.getPositionError()) < 1.7) || timer.hasElapsed(7.0));
        
        if (done)
            Swerve.getInstance().setPathFollowerSpeeds(new ChassisSpeeds());

        return done;
        // return hasFinished;
        // return hasStarted && (level || Swerve.getInstance().getControlState() != SwerveControlState.PARK);
    }

    @Override
    public void done() {
        Swerve.getInstance().setPathFollowerSpeeds(new ChassisSpeeds());
        Swerve.getInstance().snapModulesTo(ModuleSnapPositions.DEFENSE);
        Swerve.getInstance().onStop();
        // Swerve.getInstance().onStop();
    }
    


}
