package org.frogforce503.lib.auto.follower;
// package org.frogforce503.lib.follower;

// import org.frogforce503.miniMe.subsystems.swerve.Swerve;
// import org.frogforce503.miniMe.subsystems.swerve.Swerve.SwerveControlState;


// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class FetchingHandler {

//     private static FetchingHandler instance = null;
//     private double fetchingXConst;
//     private double fetchingYConst;
//     private double fetchingThetaConst;

//     public static final ChassisSpeeds ZERO_CHASSIS_SPEED = new ChassisSpeeds();
//     private ChassisSpeeds fetchingChassisSpeeds = ZERO_CHASSIS_SPEED;

//     //@formatter:off
//     public ChassisSpeeds getFetchingSpeeds() {
//         if (Swerve.getInstance().getControlState() != SwerveControlState.BALL_FETCHING)
//             return null;

//         if (RobotBase.isSimulation()) {
//             this.fetchingChassisSpeeds.vxMetersPerSecond = 0.6;
//             this.fetchingChassisSpeeds.vyMetersPerSecond = 0.1;
//             this.fetchingChassisSpeeds.omegaRadiansPerSecond = Math.toRadians(10);
//             return fetchingChassisSpeeds;
//         }

//         if (JetsonProcessor.getInstance().isTargetVisible()) {
//             double angleTowardsBall = JetsonProcessor.getInstance().getTX();
//             double distanceToBall = JetsonProcessor.getInstance().getTD();

//             JetsonProcessor.getInstance().setLastCargoAngle(angleTowardsBall);
//             JetsonProcessor.getInstance().setLastDistance(distanceToBall);

//             double xSpeed = Math.min(2, (distanceToBall > 0.2) ? fetchingXConst * distanceToBall : 0);

//             double ySpeed = (Math.abs(angleTowardsBall) > 5)? fetchingYConst * (distanceToBall * Math.sin(Math.toRadians(angleTowardsBall))): 0;

//             double thetaSpeed = (Math.abs(angleTowardsBall) > 5) ? fetchingThetaConst * angleTowardsBall : 0;

//             this.fetchingChassisSpeeds.vxMetersPerSecond = xSpeed;
//             this.fetchingChassisSpeeds.vyMetersPerSecond = ySpeed;
//             this.fetchingChassisSpeeds.omegaRadiansPerSecond = Math.toRadians(thetaSpeed);

//             SmartDashboard.putNumber("ANGULAR VEL TOWARDS TRACKED BALL", thetaSpeed);
//             SmartDashboard.putNumber("DISTANCE TO TRACKED BALL", distanceToBall);
//             SmartDashboard.putNumber("zFETCH STATE", 1);
//         } else {

//             double distanceToBall = JetsonProcessor.getInstance().getLastDistance();

//             double xSpeed = this.fetchingChassisSpeeds.vxMetersPerSecond;
//             double ySpeed = this.fetchingChassisSpeeds.vyMetersPerSecond;
//             double thetaSpeed = this.fetchingChassisSpeeds.omegaRadiansPerSecond;

//             if (Math.abs(fetchingChassisSpeeds.vxMetersPerSecond) > .2) {
//                 xSpeed -= 0.1;
//             } else {
//                 xSpeed = 0;
//             }
//             if (Math.abs(fetchingChassisSpeeds.vyMetersPerSecond) > .2) {
//                 ySpeed += -Math.signum(ySpeed) * 0.1;
//             } else {
//                 ySpeed = 0;
//             }
//             if (Math.abs(fetchingChassisSpeeds.omegaRadiansPerSecond) > Math.toRadians(2)) {
//                 thetaSpeed += -Math.signum(ySpeed) * 0.5;
//             } else {
//                 thetaSpeed = 0;
//             }
//             this.fetchingChassisSpeeds.vxMetersPerSecond = xSpeed;
//             this.fetchingChassisSpeeds.vyMetersPerSecond = ySpeed;
//             this.fetchingChassisSpeeds.omegaRadiansPerSecond = thetaSpeed;//already stored in radians/sec

//             SmartDashboard.putNumber("ANGULAR VEL TOWARDS TRACKED BALL", thetaSpeed);
//             SmartDashboard.putNumber("DISTANCE TO TRACKED BALL", distanceToBall);
//             SmartDashboard.putNumber("zFETCH STATE", 1);
//         }
//         return fetchingChassisSpeeds;
//     }

//     public static FetchingHandler getInstance() {
//         return instance == null ? instance = new FetchingHandler() : instance;
//     }


//     public enum FetchingConfig {
//         AUTON(0.75, 1.5, 0.0), TELEOP(1.25, .2, 2.25), STRAFE(1.25, 1.5, 1.0), ROTATE(0.0, 0.0, 2.25), OFF(0.0, 0.0, 0.0);

//         private final double xGain;
//         private final double yGain;
//         private final double thetaGain;

//         FetchingConfig(double xGain, double yGain, double thetaGain) {
//             this.xGain = xGain;
//             this.yGain = -yGain;
//             this.thetaGain = -thetaGain;
//         }

//         public double[] getFetchingConfig() {
//             return new double[] { xGain, yGain, thetaGain };
//         }
//     }

//     public void configFetchingConstants(FetchingConfig fetchingConfig) {
//         // Called when inputting search zone
//         this.fetchingXConst = fetchingConfig.xGain; // 1.25 * dist in teleop
//         this.fetchingYConst = fetchingConfig.yGain; // .2 * theta
//         this.fetchingThetaConst = fetchingConfig.thetaGain; // -2.25 * theta

//     }
// }
