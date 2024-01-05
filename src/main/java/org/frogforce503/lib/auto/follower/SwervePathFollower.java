package org.frogforce503.lib.auto.follower;

import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.RotationSequence;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwervePathFollower {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private Translation2d translationError = new Translation2d();
    private Rotation2d rotationError = new Rotation2d();
    private Pose2d tolerance = new Pose2d();
    private boolean isEnabled = true;

    /**
     * 
     * @param xController A PID controller to respond to error in the field-relative X direction
     * @param yController A PID controller to respond to error in the field-relative Y direction
     * @param rotationController A PID controller to respond to error in rotation
     */
    public SwervePathFollower(PIDController xController, PIDController yController, PIDController rotationController){
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;

        // Auto-configure continuous input for rotation controller
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        Translation2d translationTolerance = this.tolerance.getTranslation();
        Rotation2d rotationTolerance = this.tolerance.getRotation();

        return Math.abs(this.translationError.getX()) < translationTolerance.getX()
            && Math.abs(this.translationError.getY()) < translationTolerance.getY()
            && Math.abs(this.rotationError.getRadians()) < rotationTolerance.getRadians();
    }

    /**
     * Sets the pose error whic is considered tolerance for use with atReference()
     * 
     * @param tolerance The pose error which is tolerable
     */
    public void setTolerance(Pose2d tolerance){
        this.tolerance = tolerance;
    }

    /**
     * Enables and disables the controller for troubleshooting. When calculate() is called on a disabled
     * controller, only feedforward values are returned.
     * 
     * @param enabled If the controller is enabled or not
     */
    public void setEnabled(boolean enabled){
        this.isEnabled = enabled;
    }

    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters, Rotation2d angleRef, double angleVelocityRefRadians) {

        // Calculate feedforward velocities (field-relative).
        double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
        double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
        double thetaFF = angleVelocityRefRadians;

        translationError = poseRef.relativeTo(currentPose).getTranslation();
        rotationError = angleRef.minus(currentPose.getRotation());

        // Calculate feedback velocities (based on position error).
        double xFeedback = xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = yController.calculate(currentPose.getY(), poseRef.getY());
        double thetaFeedback =
            rotationController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());

        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.getRotation());
  }

  public void reset() {
    this.xController.reset();
    this.yController.reset();
    this.rotationController.reset();
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose.
   * @param driveState The desired drive trajectory state.
   * @param holonomicRotationState The desired holonomic rotation state.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(Pose2d currentPose, PlannedPath.HolonomicState state) {
    return calculate(
        currentPose,
        state.poseMeters,
        state.velocityMetersPerSecond,
        state.holonomicAngle,
        state.angularVelocityRadiansPerSec);
  }

    public PIDController getXController() {
        return this.xController;
    }

    public PIDController getYController() {
        return this.yController;
    }

    public PIDController getThetaController() {
        return this.rotationController;
    }

}
