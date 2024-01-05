package org.frogforce503.lib.auto.actions.swerve;

import java.util.function.Supplier;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class SwerveStationarySnapAction implements Action {

    private final Supplier<Rotation2d> rotationSupplier;
    private double toleranceDeg = 5;
    Timer timer = new Timer();

    public SwerveStationarySnapAction(Supplier<Rotation2d> rotationSupplier, double tol) {
        this.rotationSupplier = rotationSupplier;
        this.toleranceDeg = tol;
    }

    public SwerveStationarySnapAction(Rotation2d angle) {
        this(() -> angle, 12);
    }

    public SwerveStationarySnapAction(Rotation2d angle, double tol) {
        this(() -> angle, tol);
    }
    

    @Override
    public void start() { 
        timer.start();
    }

    @Override
    public void update() {
        Swerve.getInstance().setPathFollowerSpeeds(new ChassisSpeeds(0, 0, Swerve.getInstance().getSnappingGain(rotationSupplier.get())));
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(Swerve.getInstance().getAngleRotation2d().minus(rotationSupplier.get()).getDegrees()) < 5) || timer.hasElapsed(1.25);
    }

    @Override
    public void done() { 
        Swerve.getInstance().setPathFollowerSpeeds(Swerve.ZERO_CHASSIS_SPEED);
        Swerve.getInstance().onStop();
    }
    
}
