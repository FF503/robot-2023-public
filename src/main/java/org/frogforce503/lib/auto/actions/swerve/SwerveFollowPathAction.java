package org.frogforce503.lib.auto.actions.swerve;

import java.util.function.Supplier;
import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.auto.follower.SwervePathFollower;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.ModuleSnapPositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveFollowPathAction implements Action {

    private final Supplier<PlannedPath> dynamicPath;
    private PlannedPath path;
    private final SwervePathFollower controller; 
    private final Timer timer;

    private double lastTime = 0;
    private Rotation2d lastAngle = new Rotation2d();
    private Translation2d lastPosition = new Translation2d();

    private Supplier<Rotation2d> headingOverride = null;

    DoubleLogEntry timeLog, xLog, yLog, vxLog, vyLog, desiredXLog, desiredYLog, desiredVxLog, desiredVyLog;

    public SwerveFollowPathAction(Supplier<PlannedPath> dynamicPath) {
        this.dynamicPath = dynamicPath;
        this.controller = Swerve.getInstance().getPathFollower();
        this.timer = new Timer();

        DataLog log = DataLogManager.getLog();
        this.timeLog = new DoubleLogEntry(log, "wheels/timestamp");
        this.xLog = new DoubleLogEntry(log, "wheels/x");
        this.yLog = new DoubleLogEntry(log, "wheels/y");
        this.desiredXLog = new DoubleLogEntry(log, "wheels/xTarget");
        this.desiredYLog = new DoubleLogEntry(log, "wheels/yTarget");
        this.vxLog = new DoubleLogEntry(log, "wheels/vx");
        this.vyLog = new DoubleLogEntry(log, "wheels/vy");
        this.desiredVxLog = new DoubleLogEntry(log, "wheels/vxTarget");
        this.desiredVyLog = new DoubleLogEntry(log, "wheels/vyTarget");
    }

    public SwerveFollowPathAction(PlannedPath path) {
        this(() -> path);
    }

    public void configureMarkers() { } // TODO: Make this work

    public void setHeadingOverride(Supplier<Rotation2d> func) {
        this.headingOverride = func;
    }

    @Override
    public void start() {
        this.path = dynamicPath.get();
        
        this.timer.reset();
        this.controller.reset();
        this.timer.start();
        
        DataLogManager.start();

        lastPosition = this.path.getInitialHolonomicPose().getTranslation();
        lastAngle = this.path.getInitialHolonomicPose().getRotation();
        lastTime = 0;
    }

    @Override
    public void update() {
        double currentTime = this.timer.get();
        
        PlannedPath.HolonomicState desiredState = this.path.sample(currentTime);

        desiredState.holonomicAngle = headingOverride != null ? headingOverride.get() : desiredState.holonomicAngle;

        // double angleDiff = lastAngle == null ? 0 : Math.abs(lastAngle.minus(desiredState.holonomicAngle).getDegrees());
        // // System.out.println("Angle : " + angleDiff);
        // System.out.println("Angle difference: " + angleDiff);
        
        // // if (headingOverride != null && lastAngle != null) {
        // // if (angleDiff < 30) {
        // //     desiredState.holonomicRotation = lastAngle;
        // // }
        // // }



        Pose2d currentPose = Swerve.getInstance().getPoseMeters();
        // this.field.setRobotPose(currentPose);

        SmartDashboard.putNumber("PathFollower/Desired Robot X", desiredState.poseMeters.getX());
        SmartDashboard.putNumber("PathFollower/Desired Robot Y", desiredState.poseMeters.getY());
        SmartDashboard.putNumber("PathFollower/Desired Robot Theta (deg)", desiredState.holonomicAngle.getDegrees());

        SmartDashboard.putNumber("PathFollower/X Error", currentPose.getX() - desiredState.poseMeters.getX());
        SmartDashboard.putNumber("PathFollower/Y Follower Error", currentPose.getY() - desiredState.poseMeters.getY());
        SmartDashboard.putNumber("PathFollower/Theta Follower Error (deg)", currentPose.getRotation().getDegrees() - desiredState.holonomicAngle.getDegrees());

        this.timeLog.append(currentTime);
        this.desiredXLog.append(desiredState.poseMeters.getX());
        this.desiredYLog.append(desiredState.poseMeters.getY());
        this.xLog.append(currentPose.getX());
        this.yLog.append(currentPose.getY());
        
        ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);

        // if (Math.abs((controller.getThetaController().getPositionError())) > Math.toRadians(80))
        //     targetChassisSpeeds.omegaRadiansPerSecond = 0;

        // if (headingOverride != null)
        //     targetChassisSpeeds.omegaRadiansPerSecond = Swerve.getInstance().getSnappingGain(desiredState.holonomicRotation);

        SmartDashboard.putNumber("Desired X Velocity", targetChassisSpeeds.vxMetersPerSecond);
        
        this.desiredVxLog.append(targetChassisSpeeds.vxMetersPerSecond);
        this.desiredVyLog.append(targetChassisSpeeds.vyMetersPerSecond);

        Translation2d measuredVelocity = currentPose.getTranslation().minus(lastPosition).times(1/(currentTime-lastTime));
        SmartDashboard.putNumber("Measured X Velocity", measuredVelocity.getX());

        this.vxLog.append(measuredVelocity.getX());
        this.vyLog.append(measuredVelocity.getY());

        Swerve.getInstance().setPathFollowerSpeeds(targetChassisSpeeds);
        // SmartDashboard.putNumber("PPSwerveControllerCommand_xOutput", targetChassisSpeeds.vxMetersPerSecond);
        // SwerveModuleState[] targetModuleStates = this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

        // this.outputModuleStates.accept(targetModuleStates);
        lastTime = currentTime;
        lastPosition = currentPose.getTranslation();      
        lastAngle = desiredState.holonomicAngle;  
    }

    @Override
    public boolean isFinished() {
        boolean timeHasFinished = this.timer.hasElapsed(this.path.getTotalTimeSeconds());
        boolean poseTolerance = this.controller.atReference();
        boolean tooLong = timeHasFinished && this.timer.hasElapsed(this.path.getTotalTimeSeconds() + 0.5);

        boolean m_isFinished = (timeHasFinished && poseTolerance) || tooLong;
        SmartDashboard.putNumber("Path Has Finished", m_isFinished ? 1.0 : 0);

        return timeHasFinished;
    }

    @Override
    public void done() {
        Swerve.getInstance().setPathFollowerSpeeds(Swerve.ZERO_CHASSIS_SPEED);
        Swerve.getInstance().snapModulesTo(ModuleSnapPositions.DEFENSE);
        Swerve.getInstance().onStop();
    }
    
}
