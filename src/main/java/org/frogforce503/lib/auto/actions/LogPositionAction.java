package org.frogforce503.lib.auto.actions;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.opencv.ml.StatModel;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class LogPositionAction implements Action {

    double startTimestamp = 0;
    DoubleLogEntry timeLog, xLog, yLog;

    @Override
    public void start() {
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        startTimestamp = Timer.getFPGATimestamp();

        timeLog = new DoubleLogEntry(log, "wheels/timestamp");
        xLog = new DoubleLogEntry(log, "wheels/x");
        yLog = new DoubleLogEntry(log, "wheels/y");

        System.out.println("Started Logging");
    }

    @Override
    public void update() {
        timeLog.append(Timer.getFPGATimestamp() - startTimestamp);
        xLog.append(Swerve.getInstance().getPoseMeters().getX());
        yLog.append(Swerve.getInstance().getPoseMeters().getY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void done() {
        
    }
    
}
