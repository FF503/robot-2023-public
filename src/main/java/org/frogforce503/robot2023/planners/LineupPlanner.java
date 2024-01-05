package org.frogforce503.robot2023.planners;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.StateEngine;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.StateEngine.RobotStates;
import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.planners.LineupPlanner.Position.LineupMethod;
import org.frogforce503.robot2023.subsystems.DriverFeedback;
import org.frogforce503.robot2023.subsystems.Escalator;
import org.frogforce503.robot2023.subsystems.Intake;
import org.frogforce503.robot2023.subsystems.DriverFeedback.LEDLights;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LineupPlanner {

    private static LineupPlanner instance = null;
    public static LineupPlanner getInstance() {
        if (instance == null)
            instance = new LineupPlanner();
        return instance;
    }

    private Field2d field;

    private LineupState currentLineupState = LineupState.DONE;
    private Pose2d finalTarget;
    private Translation2d cornerTarget; 
    private Position currentTarget;
    private ScoringHeight targetScoringHeight;

    // obstacle avoidance
    // private List<Translation2d> obstacles;
    // private double obstacleRadius, lookaheadDistance, maxAvoidanceForce, inflationDistance;
    private Translation2d inflationVector;
    private List<Pair<Translation2d, Translation2d>> chargeStationEdges;

    private double lastUpdateTime = 0.0;

    private final double ALIGNMENT_TOLERANCE = Math.pow(Units.inchesToMeters(3), 2);

    private int driverState = -1; // not aligning

    private boolean hasBeenOverriden = false;

    private final double MAX_HORIZONTAL_EXTENSION = 1.0; // temp, this should be measured and maybe included in the escalator class
    private SendableChooser<Integer> gridSelector, cellSelector;

    private int[] selectedScoringLocation = new int[2];
    private ScoringHeight selectedScoringHeight = ScoringHeight.MID;

    private NetworkTable automagicTable;

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

        inflationVector = new Translation2d((Math.hypot(Robot.bot.fullRobotLength, Robot.bot.fullRobotWidth)/2) + 0.5, new Rotation2d(Math.PI/2));
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

        gridSelector = new SendableChooser<Integer>();
        gridSelector.setDefaultOption("Left Grid", 3);
        gridSelector.addOption("Center Grid", 2);
        gridSelector.addOption("Right Grid", 1);

        cellSelector = new SendableChooser<Integer>();
        cellSelector.setDefaultOption("Left Cell", 3);
        cellSelector.addOption("Center Cell", 2);
        cellSelector.addOption("Right Cell", 1);

        // ShuffleboardLayout automagicLayout = Shuffleboard.getTab("DriverStation")
        //     .getLayout("AutoMagic", BuiltInLayouts.kList).withSize(2, 5)
        //     .withPosition(3, 0);

        // automagicTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(Shuffleboard.getTab("DriverStation").getTitle())
        //     .getSubTable("AutoMagic");

        // automagicLayout.add("Grid Select", gridSelector).withSize(3, 1)
        //     .withPosition(1, 0)
        //     .withWidget(BuiltInWidgets.kSplitButtonChooser);
        // automagicLayout.add("Cell Select", cellSelector).withSize(3, 1)
        //     .withPosition(2, 0)
        //     .withWidget(BuiltInWidgets.kSplitButtonChooser);
        // System.out.println(inflationDistance);
        // obstacleRadius = inflationDistance;
        // lookaheadDistance = obstacleRadius;
        // maxAvoidanceForce = lookaheadDistance * -0.25;
    }

    public void selectGrid(int grid) {
        this.selectedScoringLocation[0] = grid;
        DriverFeedback.getInstance().getHUDTable()
            .getEntry("aprilSide")
            .setString(grid == 1 ? "RIGHT" : (grid == 2 ? "CENTER" : "LEFT"));
    }

    public void selectCell(int cell) {
        this.selectedScoringLocation[1] = cell;
        DriverFeedback.getInstance().getHUDTable()
            .getEntry("aprilBank")
            .setString(cell == 1 ? "RIGHT" : (cell == 2 ? "CENTER" : "LEFT"));
    }

    public void selectHeight(ScoringHeight height) {
        this.selectedScoringHeight = height;
        Escalator.getInstance().setDesiredScoringHeight(height);

        StateEngine.RobotStates state = StateEngine.getInstance().getRobotState();
        if (state == RobotStates.READY_TO_DROP)
            StateEngine.getInstance().setRobotState(RobotStates.PRE_DROP);

        DriverFeedback.getInstance().getHUDTable()
            .getEntry("aprilRow")
            .setString(height.name());
    }

    public int[] getSelectedScoringLocation() {
        return this.selectedScoringLocation;
        // return new int[] {
        //     gridSelector.getSelected(),
        //     cellSelector.getSelected()
        // };
    }

    public ScoringHeight getSelectedScoringHeight() {
        return this.selectedScoringHeight;
    }

    public void stopTrying() {
        if (this.driverState == 1) {
            Intake.getInstance().updateDriverFeebackLEDs();
            Swerve.getInstance().disableLineup();
            Swerve.getInstance().onStop();
            Swerve.getInstance().resetStabalizationHeading();
        }
        
        this.clearScreenPoints();
        this.driverState = -1;
    }

     /**
     * Returns XY coord of where the END EFFECTOR should go
     */
    public Translation2d getCoordinateOfMidJunction(int grid, int cell) {
        grid = MathUtil.clamp(grid, 1, 3);
        int cellShifted = MathUtil.clamp(cell, 1, 3) - 2;

        Translation2d target = FieldConfig.getInstance().getTagByGrid(grid)
            .plus(
                new Translation2d(FieldConfig.getInstance().X_DISTANCE_FROM_TAG_TO_MID_JUNCTION, FieldConfig.getInstance().DISTANCE_BETWEEN_CELLS * cellShifted)
            );
        return target;
    }

    public final double getOffsetFromTagToScoringLocation() {
        return 0.83;
    }

    /**
     * Returns XY coord where the ROBOT should go to score high 
     * TODO: come up with a way for combining this and the above funciton and include possibility of scoring low
     */
    public Translation2d getHighScoringLocationFor(int grid, int cell) {
        int cellShifted = MathUtil.clamp(cell, 1, 3) - 2;

        return FieldConfig.getInstance().getTagByGrid(grid)
            .plus(new Translation2d(getOffsetFromTagToScoringLocation(), cellShifted * FieldConfig.getInstance().DISTANCE_BETWEEN_CELLS));
        // return getCoordinateOfMidJunction(grid, cell)
        //     .plus(new Translation2d(
        //         Units.inchesToMeters(17.5) + Robot.bot.kWheelbaseLength * 0.75 - FieldConfig.getInstance().X_DISTANCE_FROM_TAG_TO_MID_JUNCTION, 0
        //     ));
    }

    /**
     * Without grid and cell necessary, just given the (robot) lineup posiiton for high, return the (end effector) position of the scoring
     * @param highScoringLocation
     * @return
     */
    public Translation2d getCoordinateOfMidJunction(Translation2d highScoringLocation) {
        return highScoringLocation.minus(new Translation2d(
            Units.inchesToMeters(17.5) + Robot.bot.kWheelbaseLength * 0.75 - FieldConfig.getInstance().X_DISTANCE_FROM_TAG_TO_MID_JUNCTION, 0
        ));
    }

    public void alignToSelected() {
       this.alignTo(this.selectedScoringLocation[0], this.selectedScoringLocation[1], this.selectedScoringHeight);
    }

    public void alignTo(int grid, int cell, ScoringHeight height) {
        // snap swerve to scoring angle ?
        if (this.driverState == 1 || this.driverState == -2)
            return;
        
        DriverFeedback.getInstance().setColor(LEDLights.RED);
        
        // verify validity of alignment operation (ensure we are 'inside' community bounds) (not technically inside on top but close enough)
        //TODO: make this work on the red alliance too
        Translation2d robotPose = Swerve.getInstance().getPoseMeters().getTranslation();
        boolean valid = robotPose.getX() < FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_RIGHT.getX();
        if (!valid) {
            this.currentLineupState = LineupState.INELIGIBLE;
            this.driverState = 0;
            return;
        }

        // calculate final target position
        this.targetScoringHeight = height;

        Translation2d target;
        // if (goalHeight == ScoringHeight.HIGH) {
        target = getHighScoringLocationFor(grid, cell);

        // double distance = target.minus(robotPose).getNorm();
        // double horizOffset = robotPose.getY() - target.getY();
        // if (Math.abs(horizOffset) < 0.25) {
        //     target = target.plus(new Translation2d(0, Units.inchesToMeters((cell == 2) ? 0 : 2 * (horizOffset > 0 ? -1 : 1))));
        // }

        // if (goalHeight == ScoringHeight.HIGH) {
        target = target.plus(new Translation2d(Units.inchesToMeters(9), 0));

        // } else {
        //     // MID
        //     Translation2d goalLoc 
        // }
        
        this.finalTarget = new Pose2d(target, new Rotation2d(Math.PI));
        field.getObject("Lineup Target").setPose(this.finalTarget);

        boolean rightOfHalf = robotPose.getY() < (FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT.getY() + FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_RIGHT.getY()) / 2;

        boolean isCornerNeccesary = ((robotPose.getX() > FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT.getX() + 0.75) 
            && !MathUtils.isInRange(FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT.getY(), FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT.getY(), robotPose.getY()))
            || pathIntersectsChargeStation(robotPose, target.minus(robotPose));

        if ((rightOfHalf && grid == 1 && cell == 1) || (!rightOfHalf && grid == 3 && cell == 3))
            isCornerNeccesary = false;

        if (isCornerNeccesary) {
            // calculate first corner position
            Translation2d corner = (rightOfHalf ? FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT : FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT)
                .plus(new Translation2d(Robot.bot.fullRobotLength + 0.1, Rotation2d.fromDegrees(135 * (rightOfHalf ? -1 : 1))));
            
            this.cornerTarget = corner;

            this.currentLineupState = LineupState.EN_ROUTE;
            this.currentTarget = new Position(corner, new Rotation2d(Math.PI), Position.LineupMethod.ODOM);
            field.getObject("Lineup Corner").setPose(new Pose2d(this.cornerTarget, new Rotation2d()));
        } else {
            this.currentLineupState = LineupState.ALIGN;
            this.currentTarget = new Position(this.finalTarget, LineupMethod.ODOM);
            field.getObject("Lineup Corner").setPose(new Pose2d());
        }

       

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
                
        Swerve.getInstance().startLineup();
        this.driverState = 1;
    }

    public Translation2d getDisplacementFromTarget(Translation2d curPos) {
        if (this.currentTarget.lineupMethod == LineupMethod.VISION) {
            if (!AprilTagHandler.getInstance().isTargetVisible())
                return new Translation2d();

            Translation2d currentOffset = AprilTagHandler.getInstance().getPrimaryTarget().getRobotRelativePose().getTranslation();
            Translation2d desiredInv = this.currentTarget.point.times(1);
            
            Translation2d displacement = currentOffset.plus(desiredInv).times(-1);

            return displacement;
            // return AprilTagHandler.getInstance().getPrimaryTarget().getRobotRelativePose().getTranslation().minus(this.currentTarget.point);
        } else if (this.currentTarget.lineupMethod == LineupMethod.HYBRID){
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
        
        if (disp.getNorm() < 0.075)
            disp = new Translation2d(0, disp.getY());
        
        return disp;
        // Translation2d avoidanceForce = this.currentLineupState == LineupState.EN_ROUTE ? calculateAvoidanceForce(disp, curPos) : new Translation2d();

        // return disp.plus(avoidanceForce);
    }

    public void overrideCurrentTarget(Translation2d visionOffset, LineupMethod method) {
        this.currentTarget = new Position(visionOffset, new Rotation2d(Math.PI), method);
        this.hasBeenOverriden = true;
    }

    public Position getCurrentTarget() {
        return this.currentTarget;
    }
    
    public void update(Translation2d robotPos) {
        if (this.hasBeenOverriden)
            return;

        switch (this.currentLineupState) {
            case EN_ROUTE: {
                // here we want to calculate avoidance force
                if (robotPos.getX() < this.cornerTarget.getX() + Units.inchesToMeters(15))
                    this.currentLineupState = LineupState.CORNER;
                else
                    break;
            }
            case CORNER: {
                // single cycle operation, whole state may not be needed idk (edit: it is now)
                if (this.targetScoringHeight == ScoringHeight.HIGH) {
                    this.currentLineupState = LineupState.ALIGN;
                    this.currentTarget = new Position(this.finalTarget, Position.LineupMethod.ODOM);
                    // this.escalatorHeightTarget = MAX;
                    break;
                }

                // TODO: do some intercepting if we are inside arm's radius (probably not going to do this anymore)
                
                // calculate new target position (intercept point)
                this.currentTarget = new Position(calculateInterceptPoint(this.finalTarget.getTranslation()), Position.LineupMethod.ODOM);
                break;
            }
            case ALIGN: {
                if (MathUtils.distanceSquared(robotPos, this.finalTarget.getTranslation()) < ALIGNMENT_TOLERANCE) {
                    // this.currentTarget = new Position(this.finalTarget.getTranslation().plus(new Translation2d(-Units.inchesToMeters(6.5), 0)), Position.LineupMethod.ODOM);
                    Intake.getInstance().updateDriverFeebackLEDs();
                    Swerve.getInstance().snapModulesTo(ModuleSnapPositions.DEFENSE);
                    this.currentLineupState = LineupState.DONE;
                } else
                    break;
            }
            case PULL_IN: {
                // if (MathUtils.distanceSquared(robotPos, this.currentTarget.point) < ALIGNMENT_TOLERANCE) {
                this.currentLineupState = LineupState.DONE;
                // } else
                //     break;
                break;
            }
            case DONE: {
                Swerve.getInstance().disableLineup();
                Swerve.getInstance().resetStabalizationHeading();
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

    public double getLoadingAngle() {
        return RobotState.getInstance().getAllianceColor() == AllianceColor.BLUE ? (Math.PI/2) : (-Math.PI/2);
    }

    public LineupState getLineupState() {
        return this.currentLineupState;
    }

    private Pose2d calculateInterceptPoint(Translation2d target) {
        Translation2d endEffectorPos = getCoordinateOfMidJunction(target);
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
        field.getObject("Lineup Target").setPose(new Pose2d());
        field.getObject("Lineup Corner").setPose(new Pose2d());
    }

    public enum LineupState {
        EN_ROUTE, CORNER, ALIGN, PULL_IN, DONE, INELIGIBLE;
    }

    public enum ScoringHeight {
        LOW, MID, HIGH;
    }

    public static class Position {
        private final Translation2d point;
        public Optional<Rotation2d> angle;
        public LineupMethod lineupMethod;

        private Translation2d _hybridVisionOffset = new Translation2d();

        public Position(Translation2d point, Rotation2d angle, LineupMethod lineupMethod) {
            this.point = point;
            this.angle = Optional.ofNullable(angle);
            this.lineupMethod = lineupMethod;
        }

        public Position(Pose2d pose, LineupMethod lineupMethod) {
            this(pose.getTranslation(), pose.getRotation(), lineupMethod);
        }

        public Position(Translation2d point, LineupMethod lineupMethod) {
            this(point, null, lineupMethod);
        }

        public void provideVisionUpdate(Translation2d offset) {
            this._hybridVisionOffset = offset;
        }

        public Translation2d getPoint() {
            return this.point.plus(_hybridVisionOffset);
        }

        public enum LineupMethod {
            ODOM, VISION, HYBRID
        }
    }
}
