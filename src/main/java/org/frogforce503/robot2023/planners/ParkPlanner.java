
package org.frogforce503.robot2023.planners;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.planners.LineupPlanner.Position.LineupMethod;
import org.frogforce503.robot2023.planners.ParkPlanner.Position.ParkingMethod;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.ModuleSnapPositions;
import org.frogforce503.robot2023.subsystems.vision.AprilTagHandler;
import org.frogforce503.robot2023.subsystems.vision.cameras.Limelight;

import com.fasterxml.jackson.core.TreeNode;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ParkPlanner {

    private static ParkPlanner instance = null;
    public static ParkPlanner getInstance() {
        if (instance == null)
            instance = new ParkPlanner();
        return instance;
    }

    private Field2d field;

    private ParkState currentParkingState = ParkState.DONE;
    private Pose2d finalTarget;
    private Translation2d cornerTarget; 
    private Position currentTarget;

    // obstacle avoidance
    // private List<Translation2d> obstacles;
    // private double obstacleRadius, lookaheadDistance, maxAvoidanceForce, inflationDistance;
    private Translation2d inflationVector;
    private List<Pair<Translation2d, Translation2d>> chargeStationEdges;

    private double lastUpdateTime = 0.0;

    private final double ALIGNMENT_TOLERANCE = Math.pow(Units.inchesToMeters(1), 2);

    private int driverState = -1; // not aligning

    private boolean hasBeenOverriden = false;

    private final double MAX_HORIZONTAL_EXTENSION = 1.0; // temp, this should be measured and maybe included in the escalator class

    public void initialize() {
        field = Swerve.getInstance().getField();
        
        FieldConfig fc = FieldConfig.getInstance();
        Rotation2d r = new Rotation2d();

        List<Pose2d> chargingStationPoints = Arrays.asList(
            new Pose2d(fc.CHARGE_OUTER_TOP_LEFT, r),
            new Pose2d(fc.CHARGE_OUTER_TOP_RIGHT, r),
            new Pose2d(fc.CHARGE_OUTER_BOTTOM_RIGHT, r),
            new Pose2d(fc.CHARGE_OUTER_BOTTOM_LEFT, r)
            // new Pose2d(fc.CHARGE_HP_SIDE_TAPE_EDGE, r),
            // new Pose2d(fc.CHARGE_CABLE_PROTECTOR_EDGE, r)
        );

        field.getObject("Charging Station").setPoses(chargingStationPoints);

        inflationVector = new Translation2d((Math.hypot(Robot.bot.fullRobotLength, Robot.bot.fullRobotWidth)/2) + 0.05, new Rotation2d(Math.PI/2));
        Translation2d inflationNorm = inflationVector.rotateBy(new Rotation2d(0));

        chargeStationEdges = Arrays.asList(
            new Pair<Translation2d, Translation2d>(fc.CHARGE_OUTER_TOP_LEFT.plus(inflationNorm), fc.CHARGE_OUTER_TOP_RIGHT.plus(inflationVector)),
            new Pair<Translation2d, Translation2d>(fc.CHARGE_OUTER_TOP_RIGHT.plus(inflationVector), fc.CHARGE_OUTER_BOTTOM_RIGHT.minus(inflationNorm)),
            new Pair<Translation2d, Translation2d>(fc.CHARGE_OUTER_BOTTOM_RIGHT.minus(inflationNorm), fc.CHARGE_OUTER_BOTTOM_LEFT.minus(inflationVector)),
            new Pair<Translation2d, Translation2d>(fc.CHARGE_OUTER_BOTTOM_LEFT.minus(inflationVector), fc.CHARGE_OUTER_TOP_LEFT.plus(inflationNorm))
        );


        // Translation2d blMidpoint = fc.CHARGE_CABLE_PROTECTOR_EDGE.plus(fc.CHARGE_OUTER_BOTTOM_LEFT).times(0.5);
        // double halfd = blMidpoint.getX() - fc.CHARGE_OUTER_BOTTOM_LEFT.getX();

        // blMidpoint = blMidpoint.plus(new Translation2d(0, halfd - (inflationDistance/2)));
        // Translation2d brMidpoint = fc.CHARGE_CABLE_PROTECTOR_EDGE.plus(new Translation2d(halfd, halfd - (inflationDistance/2)));

        // Translation2d tlMidpoint = new Translation2d(blMidpoint.getX(), fc.CHARGE_OUTER_TOP_LEFT.getY() - (halfd - (inflationDistance/2)));
        // Translation2d trMidpoint = new Translation2d(brMidpoint.getX(), tlMidpoint.getY());

        // obstacles = Arrays.asList(
        //     blMidpoint,
        //     brMidpoint,
        //     tlMidpoint,
        //     trMidpoint
        // );

        List<Pose2d> dispObs = new ArrayList<Pose2d>();
        for (Pair<Translation2d, Translation2d> o : chargeStationEdges)
            dispObs.add(new Pose2d(o.getFirst(), r));

        field.getObject("Obstacles").setPoses(dispObs);

        // System.out.println(inflationDistance);
        // obstacleRadius = inflationDistance;
        // lookaheadDistance = obstacleRadius;
        // maxAvoidanceForce = lookaheadDistance * -0.25;
    }

    public void stopTrying() {
        if (this.driverState == 1) {
            Swerve.getInstance().disableParking();
            Swerve.getInstance().onStop();
            Swerve.getInstance().resetStabalizationHeading();
        }

        // Swerve.getInstance().snapModulesTo(ModuleSnapPositions.DEFENSE);

        // Swerve.getInstance().onStop();
        
        this.clearScreenPoints();
        this.driverState = -1;
    }

     /**
     * Returns XY coord of the center of the charging station
     */
    public Translation2d getCoordinateOfCenter() {
        Translation2d target = 
            (FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT.plus(FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT)).times(0.5).plus(new Translation2d(-2.1, 0.0));
        return target;
    }


    public void alignTo() {
        System.out.println("driver state: " + this.driverState);
        // snap swerve to scoring angle ?
        if (this.driverState == 1 || this.driverState == -2)
            return;
        
        // verify validity of alignment operation (ensure we are 'inside' community bounds) (not technically inside on top but close enough)
        //TODO: make this work on the red alliance too
        Translation2d robotPose = Swerve.getInstance().getPoseMeters().getTranslation();
        boolean valid = (robotPose.getY() < FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT.getY() - (Robot.bot.fullRobotWidth/2) - 0.10
            &&robotPose.getY()> FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT.getY() + (Robot.bot.fullRobotLength/2) + 0.10
            );
        if (!valid) {
            this.currentParkingState = ParkState.INELIGIBLE;
            this.driverState = 0;
            return;
        }

        Translation2d target;
        // if (goalHeight == ScoringHeight.HIGH) {
        target = new Translation2d(ParkPlanner.getInstance().getCoordinateOfCenter().getX(),
            robotPose.getY());
        // } else {, ALIGNMENT_TOLERANCE) ParkPlanner.getInstance().getCoordinateOfCenter().getX();
        // } else {
        //     // MID
        //     Translation2d goalLoc 
        // }
        
        this.finalTarget = new Pose2d(target, new Rotation2d(Math.PI));
        field.getObject("Parking Target").setPose(this.finalTarget);

        //  boolean isCornerNeccesary = ((robotPose.getX() > FieldConfig.getInstance().CHARGE_HP_SIDE_TAPE_EDGE.getX()) 
        //      && !MathUtils.isInRange(FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT.getY(), FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT.getY(), robotPose.getY()))
        //      || pathIntersectsChargeStation(robotPose, target.minus(robotPose));

        // if (isCornerNeccesary) {
        //     // calculate first corner position
        //     boolean lower = robotPose.getY() < (FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT.getY() + FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_RIGHT.getY()) / 2;

        //     Translation2d corner = (lower ? FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT : FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT)
        //         .plus(new Translation2d(Robot.bot.fullRobotLength, Rotation2d.fromDegrees(150 * (lower ? -1 : 1))));
            
        //     this.cornerTarget = corner;

        //     this.currentParkingState = ParkState.EN_ROUTE;
        //     this.currentTarget = new Position(corner, new Rotation2d(Math.PI), ParkingMethod.VISION);
        //     field.getObject("Lineup Corner").setPose(new Pose2d(this.cornerTarget, new Rotation2d()));
        // } else {
            this.currentParkingState = ParkState.ALIGN;
            this.currentTarget = new Position(this.finalTarget, Position.ParkingMethod.ODOM);
            field.getObject("Lineup Corner").setPose(new Pose2d());
        // }

       

        // // boolean cornerUnnecesary = grid != 2 && (!lower || grid != 3) && (lower || grid != 1) && (!lower || cell != 3) || ()
        // boolean cornerIsNecessary = (robotPose.getX() > this.cornerTarget.getX()) && (
        //     (
        //         grid == 2 || 
        //         (grid == 1 && !lower) ||
        //         (grid == 3 && lower) ||
        //         (lower && grid == 1 && cell == 3) ||
        //         (!lower && grid == 3 && cell == 1)
        //     ) && ((Math.abs(target.minus(robotPose).getAngle().minus(corner.minus(robotPose).getAngle()).getDegrees()) <= 90))            
        // );
        
        // TOOD: make this work on red too
        // if ( || Math.abs(target.minus(robotPose).getAngle().minus(corner.minus(robotPose).getAngle()).getDegrees()) > 90) {
        // if (!cornerIsNecessary) {
            // this.currentLineupState = LineupState.ALIGN;
            // this.currentTarget = new Position(this.finalTarget, LineupMethod.ODOM);
            // // this.currentTarget = new Position(this.targetScoringHeight == ScoringHeight.HIGH ? new Pose2d(target, new Rotation2d(Math.PI)) : calculateInterceptPoint(target), Position.LineupMethod.HYBRID);
            // field.getObject("Lineup Corner").setPose(new Pose2d());
        // } else {
            // this.currentLineupState = LineupState.EN_ROUTE;
            // this.currentTarget = new Position(corner, new Rotation2d(Math.PI), Position.LineupMethod.ODOM);
            // field.getObject("Lineup Corner").setPose(new Pose2d(this.cornerTarget, new Rotation2d()));
        // }

         //new Position(this.targetScoringHeight == ScoringHeight.HIGH ? new Pose2d(target, new Rotation2d(Math.PI)) : calculateInterceptPoint(target), Position.LineupMethod.ODOM);
                
        Swerve.getInstance().startParking();
        this.driverState = 1;
    }

    public Translation2d getDisplacementFromTarget(Translation2d curPos) {
        if (this.currentTarget.parkingMethod == ParkingMethod.VISION) {
            if (!AprilTagHandler.getInstance().isTargetVisible())
                return new Translation2d();

            Translation2d currentOffset = AprilTagHandler.getInstance().getPrimaryTarget().getRobotRelativePose().getTranslation();
            Translation2d desiredInv = this.currentTarget.point.times(1);
            
            Translation2d displacement = currentOffset.plus(desiredInv).times(-1);

            return displacement;
            // return AprilTagHandler.getInstance().getPrimaryTarget().getRobotRelativePose().getTranslation().minus(this.currentTarget.point);
        } else if (this.currentTarget.parkingMethod == ParkingMethod.HYBRID){
            double timeSinceLastUpdate = Timer.getFPGATimestamp() - this.lastUpdateTime;
            if (timeSinceLastUpdate > 0.75 && AprilTagHandler.getInstance().isTargetVisible()) {
                Translation2d goal = this.currentTarget.point;
                Translation2d positionVision = AprilTagHandler.getInstance().getEstimatedRobotPose();
                Translation2d positionOdometry = Swerve.getInstance().getPoseMeters().getTranslation();
                
                // turn the hybrid target into an effectively odom one by using difference to calculate new odom setpoint
                Translation2d goalOdometry = goal.minus( positionVision.minus(positionOdometry) ); 

                this.currentTarget.provideVisionUpdate(goalOdometry);

                this.lastUpdateTime = Timer.getFPGATimestamp();
            }
        }

        // System.out.println("Current target: " + this.currentTarget.getPoint());

        Translation2d disp = curPos.minus(this.currentTarget.getPoint()); // for both hybrid and odom
        return disp;
        // Translation2d avoidanceForce = this.currentLineupState == LineupState.EN_ROUTE ? calculateAvoidanceForce(disp, curPos) : new Translation2d();

        // return disp.plus(avoidanceForce);
    }

    public void overrideCurrentTarget(Translation2d visionOffset, ParkingMethod method) {
        this.currentTarget = new Position(visionOffset, new Rotation2d(Math.PI), method);
        this.hasBeenOverriden = true;
    }

    public Position getCurrentTarget() {
        return this.currentTarget;
    }
    
    public void update(Translation2d robotPos) {
        if (this.hasBeenOverriden)
            return;

        switch (this.currentParkingState) {
            case EN_ROUTE: {
                // here we want to calculate avoidance force
                if (robotPos.getX() < this.cornerTarget.getX() + Units.inchesToMeters(15))
                    this.currentParkingState = ParkState.CORNER;
                else
                    break;
            }
            // case CORNER: {
            //     // single cycle operation, whole state may not be needed idk (edit: it is now)
            //     if (this.targetScoringHeight == ScoringHeight.HIGH) {
            //         this.currentLineupState = LineupState.ALIGN;
            //         this.currentTarget = new Position(this.finalTarget, Position.LineupMethod.ODOM);
            //         // this.escalatorHeightTarget = MAX;
            //         break;
            //     }

            //     // TODO: do some intercepting if we are inside arm's radius (probably not going to do this anymore)
                
            //     // calculate new target position (intercept point)
            //     this.currentTarget = new Position(calculateInterceptPoint(this.finalTarget.getTranslation()), Position.LineupMethod.ODOM);
            //     break;
            // }
            case ALIGN: {
                if (MathUtils.distanceSquared(robotPos, this.finalTarget.getTranslation()) < ALIGNMENT_TOLERANCE)
                    this.currentParkingState = ParkState.DONE;
                else
                    break;
            }
            case DONE: {
                Swerve.getInstance().disableParking();
                Swerve.getInstance().onStop();
                this.driverState = -2;
                // here we would call gamespec stuff ? check if arm is out all the way and then deliver ?
                break;
            }
            default: {
                // basically do nothing (ineligible for autodrive)
            }
        }
    }

    private Pose2d calculateInterceptPoint() {
        Translation2d endEffectorPos = getCoordinateOfCenter();
        Translation2d disp = Swerve.getInstance().getPoseMeters().getTranslation().minus(endEffectorPos); // at some point add the displacement between arm base and center of robot
        
        Translation2d interceptPos = new Translation2d(MAX_HORIZONTAL_EXTENSION, disp.getAngle()).plus(endEffectorPos);
        // this.currentTarget = new Position(interceptPos, disp.times(-1).getAngle(), LineupMethod.HYBRID);

        // this.escalatorHeightTarget = MAX_HORIZONTAL_EXTENSION; // also would need to conver this with angle or something
        
        return new Pose2d(interceptPos, disp.times(-1).getAngle());
    }

    // current, dispVector, lineOrigin, lineDisp
    private boolean intersects(Translation2d p, Translation2d r, Translation2d q, Translation2d s) {
        double num = MathUtils.cross(q.minus(p), r);
        double den = MathUtils.cross(r, s);

        if (num == 0 && den == 0) // collinear
            return true;
        else if (den == 0) // parallel 
            return false;
        
        double u = num/den;
        double t = MathUtils.cross(q.minus(p), s) / den;

        return (t >= 0) && (t <= 1) && (u >= 0) && (u <= 1);
    }

    private boolean pathIntersectsChargeStation(Translation2d current, Translation2d dispVector) {
        for (Pair<Translation2d, Translation2d> edge : this.chargeStationEdges) {
            if (intersects(current, dispVector, edge.getFirst(), edge.getSecond().minus(edge.getFirst())))
                return true;
        }
        return false;        
        // check horizontal lines
        /*
         * methodology: define line of robot travel as
         * y - y1 = m(x - x1) where (x1, y1) is the robot position
         * 
         * now solve for x as a function of y
         * 
         * (y - y1 + m(x1))/m = x
         * 
         * plug in y of the horizontal lines and see if the resulting x is between the bounds of charge station
         */
        
         // check bottom of charge station

        
    }

    // based on math from: https://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-collision-avoidance--gamedev-7777
    // private Translation2d calculateAvoidanceForce(Translation2d disp, Translation2d curPos) {
    //     Translation2d ahead = new Translation2d(lookaheadDistance, disp.getAngle());
    //     Translation2d ahead2 = ahead.times(0.5).plus(curPos);
    //     ahead = ahead.plus(curPos);

    //     // if either ahead or ahead2 is inside any of the obstacles, then we are intersecting

    //     Translation2d mostThreatening = null;
    //     double distFromMostThreatening = 1000000000;
    //     for (Translation2d obstacle : this.obstacles) {
    //         boolean a1 = obstacle.getDistance(ahead) < obstacleRadius;
    //         boolean a2 = obstacle.getDistance(ahead2) < obstacleRadius;
    //         boolean collision = a1 || a2;

    //         double distTo = curPos.getDistance(obstacle);
    //         if (collision && (mostThreatening == null || distTo < distFromMostThreatening)) {
    //             mostThreatening = obstacle;
    //             distFromMostThreatening = distTo;
    //         }
    //     }

    //     if (mostThreatening == null) {
    //         System.out.println("NO OBSTACLE IN PATH");
    //         return new Translation2d();
    //     }

    //     Translation2d avoidanceVec = ahead.minus(mostThreatening);
    //     return new Translation2d(maxAvoidanceForce, avoidanceVec.getAngle());
    // }

    public void clearScreenPoints() {
        field.getObject("Parking Target").setPose(new Pose2d());
        field.getObject("Parking Corner").setPose(new Pose2d());
    }

    public enum ParkState {
        EN_ROUTE, CORNER, ALIGN, DONE, INELIGIBLE;
    }

    public static class Position {
        private final Translation2d point;
        public Optional<Rotation2d> angle;
        public ParkingMethod parkingMethod;

        private Translation2d _hybridVisionOffset = new Translation2d();

        public Position(Translation2d point, Rotation2d angle, ParkingMethod parkingMethod) {
            this.point = point;
            this.angle = Optional.ofNullable(angle);
            this.parkingMethod = parkingMethod;
        }

        public Position(Pose2d pose, ParkingMethod parkingMethod) {
            this(pose.getTranslation(), pose.getRotation(), parkingMethod);
        }

        public Position(Translation2d point, ParkingMethod parkingMethod) {
            this(point, null, parkingMethod);
        }

        public void provideVisionUpdate(Translation2d offset) {
            this._hybridVisionOffset = offset;
        }

        public Translation2d getPoint() {
            return this.point.plus(_hybridVisionOffset);
        }

        public enum ParkingMethod {
            ODOM, VISION, HYBRID
        }
    }
}
