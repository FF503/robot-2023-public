package org.frogforce503.robot2023.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.frogforce503.lib.auto.AutoChooser.StartingLocation;
import org.frogforce503.lib.auto.actions.RobotStateAction;
import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.auto.actions.base.InlineAction;
import org.frogforce503.lib.auto.actions.base.ParallelAction;
import org.frogforce503.lib.auto.actions.base.SeriesAction;
import org.frogforce503.lib.auto.actions.base.WaitUntil;
import org.frogforce503.lib.auto.actions.swerve.GyroBalanceAction;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.StateEngine;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.RobotState.GamePiece;
import org.frogforce503.robot2023.StateEngine.RobotStates;
import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.planners.LineupPlanner;
import org.frogforce503.robot2023.planners.LineupPlanner.ScoringHeight;
import org.frogforce503.robot2023.subsystems.Escalator;
import org.frogforce503.robot2023.subsystems.Intake;
import org.frogforce503.robot2023.subsystems.Intake.IntakeStates;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.ModuleSnapPositions;

import com.fasterxml.jackson.core.StreamWriteCapability;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// contains commonly used functions and actions
public class AutoUtil {

    public static boolean onFirstShot = false;
    public static boolean isOnCable = false;
    public static boolean useFrontOnly = false;

    public static boolean frontCam = true;

    public static Action shootAction(ScoringHeight height) {
        return new InlineAction(() -> {
            Escalator.getInstance().setDesiredScoringHeight(height);
            StateEngine.getInstance().allowDrop();
            StateEngine.getInstance().setRobotState(RobotStates.PRE_DROP);
        });
    }

    public static Action autoShootAction(ScoringHeight height) {
        return new ParallelAction(
            new InlineAction(() -> {
                Escalator.getInstance().setDesiredScoringHeight(height);
                StateEngine.getInstance().setRobotState(RobotStates.PRE_DROP);
            }),
            new SeriesAction(
                WaitUntil.stateReached(RobotStates.READY_TO_DROP),
                // WaitUntil.timeElapsed(0.5),
                new InlineAction(StateEngine.getInstance()::allowDrop)
            )
        );
    }

    public static Action dropAction() {
        return new InlineAction(() -> Escalator.getInstance().claw.set(Value.kForward));
    }

    public static Action retractAction() {
        return new RobotStateAction(RobotStates.ZERO);
    }

    public static Action stopIntakeAction() {
        return new RobotStateAction(RobotStates.INDEXING);
    }

    public static Action scoringSequence(ScoringHeight height, GamePiece gamePiece) {
        return scoringSequence(height, false, gamePiece);
    }

    public static Action scoringSequence(ScoringHeight height) {
        return scoringSequence(height, GamePiece.CUBE);
    }

    public static Action setDesiredGamePiece(GamePiece gamePiece) {
        return new InlineAction(() -> Intake.getInstance().setDesiredGamePiece(gamePiece));
    }

    public static Action cubeMode() {
        return setDesiredGamePiece(GamePiece.CUBE);
    }

    public static Action coneMode() {
        return setDesiredGamePiece(GamePiece.CONE);
    }

    private static BooleanSupplier elevatorDownEnough = () -> {
        RobotStates state = StateEngine.getInstance().getRobotState();
        return state == RobotStates.IDLE || (state == RobotStates.ZERO && Intake.getInstance().getControlState() == IntakeStates.STOWED);
    };

    public static Action scoringSequence(ScoringHeight height, boolean faster, GamePiece gamePiece) {
        if (RobotBase.isReal()) {
            return new SeriesAction(
                new InlineAction(() -> Intake.getInstance().setDesiredGamePiece(gamePiece)),
                shootAction(height),
                // WaitUntil.stateReached(RobotStates.READY_TO_DROP),
                // dropAction(),
                // WaitUntil.timeElapsed(0.125),
                // retractAction(),
                (faster ? new WaitUntil(elevatorDownEnough) : WaitUntil.stateReached(RobotStates.IDLE))
            );
        } else {
            return WaitUntil.timeElapsed(0.25);
        }
    }

    public static Action firstScoringSequence() {
        return new SeriesAction(
            new InlineAction(() -> {
                Intake.getInstance().setDesiredGamePiece(GamePiece.CUBE);
                StateEngine.getInstance().setRobotState(RobotStates.AUTON_MID_SHOOT);
            }),
            WaitUntil.timeElapsed(0.75),
            new InlineAction(() -> {
                StateEngine.getInstance().setRobotState(RobotStates.IDLE);
            })
        );
    }

    public static Action balance() {
        return new SeriesAction(
            new GyroBalanceAction(),
            new InlineAction(() -> Swerve.getInstance().snapModulesTo(ModuleSnapPositions.DEFENSE))
        );
    }

    public static class Waypoints {

        public static Waypoint scoring(int grid, int cell, boolean offsetSlight) {
            // this slight offset is to account with the issue with wpilib trajectory generation when starting and ending a trajectory at the same or very close points
            Translation2d point = LineupPlanner.getInstance().getHighScoringLocationFor(grid, cell).plus(new Translation2d(offsetSlight ? 0.03 : 0, offsetSlight ? 0.03 : 0));
            return Waypoint.fromHolonomicPose(new Pose2d(point, new Rotation2d(Math.PI)));
        }

        public static Waypoint scoring(int grid, int cell) {
            return scoring(grid, cell, false);
        }

        // returns the list of necessary transit points to exit the community, will handle whether or not the robot should spin while driving long distance based on if crossing cable protector or not
        public static List<Waypoint> transitPoints(StartingLocation grid, boolean exitingCommunity, boolean ignoreHeading) { 
            return transitPoints(grid, exitingCommunity, ignoreHeading, false);
        }

        public static List<Waypoint> shift(List<Waypoint> input, Translation2d delta) {
            ArrayList<Waypoint> output = new ArrayList<>();

            for (Waypoint w : input) {
                output.add(w.plus(delta));
            }

            return output;
        }

        public static List<Waypoint> withHeading(List<Waypoint> input, Rotation2d heading) {
            ArrayList<Waypoint> output = new ArrayList<>();

            for (Waypoint w : input) {
                output.add(w.setHolonomicRotation(heading));
            }

            return output;
        }

        public static List<Waypoint> transitPoints90(StartingLocation grid, boolean exitingCommunity) {
            boolean red = RobotState.getInstance().getAllianceColor() == AllianceColor.RED;
            boolean left = grid == StartingLocation.LEFT_GRID;
            boolean crossingCable = (left && red) || (!left && !red);

            List<Waypoint> points = new ArrayList<Waypoint>();

            Translation2d prePoint = (left ? FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT : FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT)
                .plus(new Translation2d(-0.1, 0.675 * (left ? 1 : -1)));
            
            Translation2d afterPoint = (left ? FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT : FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_RIGHT)
                .plus(new Translation2d(0.1, 0.55 * (left ? 1 : -1)));
            

            Waypoint preWaypoint = new Waypoint(prePoint, null, new Rotation2d((left ? -1 : 1) * (Math.PI/2)));
            Waypoint afterWaypoint = new Waypoint(afterPoint, null, new Rotation2d((left ? -1 : 1) * (Math.PI/2)));

            points.add(exitingCommunity ? preWaypoint : afterWaypoint);
            points.add(exitingCommunity ? afterWaypoint : preWaypoint);

            return points;
        }

        public static List<Waypoint> transitPoints(StartingLocation grid, boolean exitingCommunity, boolean ignoreHeading, boolean ninetyForCable) { 
            boolean red = RobotState.getInstance().getAllianceColor() == AllianceColor.RED;
            boolean left = grid == StartingLocation.LEFT_GRID;
            boolean crossingCable = (left && red) || (!left && !red);

            List<Waypoint> points = new ArrayList<Waypoint>();

            Translation2d prePoint = (left ? FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT : FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT)
                .plus(new Translation2d(-0.25, ((isOnCable && !exitingCommunity) ? 0.7 : 0.775) * (left ? 1 : -1)));
            
            Translation2d afterPoint = (left ? FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT : FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_RIGHT)
                .plus(new Translation2d(crossingCable ? 0.5 : 0.5, (exitingCommunity ? 0.775 : (isOnCable ? 0.75 : 0.825)) * (left ? 1 : -1)));
            
            Waypoint preWaypoint = ignoreHeading ? new Waypoint(prePoint, null, null) : Waypoint.fromHolonomicPose(new Pose2d(prePoint, (isOnCable && ninetyForCable) ? new Rotation2d(Math.PI/2) : new Rotation2d(Math.PI)));
            Waypoint afterWaypoint = ignoreHeading ? new Waypoint(afterPoint, null, null) : Waypoint.fromHolonomicPose(new Pose2d(afterPoint, (isOnCable && ninetyForCable) ? new Rotation2d(Math.PI/2) : new Rotation2d(exitingCommunity ? 0 : Math.PI)));

            points.add(exitingCommunity ? preWaypoint : afterWaypoint);

            if (crossingCable) {
                Translation2d justPastCable = prePoint.interpolate(afterPoint, 0.5);
                Waypoint justPastCableWaypoint = ignoreHeading ? new Waypoint(justPastCable) : Waypoint.fromHolonomicPose(new Pose2d(justPastCable, ninetyForCable ? new Rotation2d(Math.PI/2) : new Rotation2d(Math.PI)));
                points.add(justPastCableWaypoint);
            }

            points.add(exitingCommunity ? afterWaypoint : preWaypoint);
            return points;
        }

        public static List<Waypoint> transitPointsSpinBefore(StartingLocation grid, boolean exitingCommunity, boolean ignoreHeading) { 
            boolean red = RobotState.getInstance().getAllianceColor() == AllianceColor.RED;
            boolean left = grid == StartingLocation.LEFT_GRID;
            boolean crossingCable = (left && red) || (!left && !red);

            List<Waypoint> points = new ArrayList<Waypoint>();

            Translation2d prePoint = (left ? FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT : FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT)
                .plus(new Translation2d(-0.25, ((isOnCable && !exitingCommunity) ? 0.7 : 0.775) * (left ? 1 : -1)));
            
            Translation2d afterPoint = (left ? FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT : FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_RIGHT)
                .plus(new Translation2d(crossingCable ? 0.5 : 0.5, (exitingCommunity ? 0.775 : (isOnCable ? 0.75 : 0.825)) * (left ? 1 : -1)));
            
            Waypoint preWaypoint = ignoreHeading ? new Waypoint(prePoint, null, null) : Waypoint.fromHolonomicPose(new Pose2d(prePoint, new Rotation2d(exitingCommunity ? 0 : Math.PI)));
            Waypoint afterWaypoint = ignoreHeading ? new Waypoint(afterPoint, null, null) : Waypoint.fromHolonomicPose(new Pose2d(afterPoint, new Rotation2d(exitingCommunity ? 0 : Math.PI)));

            points.add(exitingCommunity ? preWaypoint : afterWaypoint);

            if (crossingCable) {
                Translation2d justPastCable = prePoint.interpolate(afterPoint, 0.5);
                Waypoint justPastCableWaypoint = ignoreHeading ? new Waypoint(justPastCable) : Waypoint.fromHolonomicPose(new Pose2d(justPastCable, new Rotation2d(exitingCommunity ? 0 : Math.PI)));
                points.add(justPastCableWaypoint);
            }

            points.add(exitingCommunity ? afterWaypoint : preWaypoint);
            return points;
        }

        // public static List<Waypoint> transitPointsCable(StartingLocation grid, boolean exitingCommunity) { 
        //     boolean red = RobotState.getInstance().getAllianceColor() == AllianceColor.RED;
        //     boolean left = grid == StartingLocation.LEFT_GRID;
        //     boolean crossingCable = false;// (left && red) || (!left && !red);

        //     List<Waypoint> points = new ArrayList<Waypoint>();

        //     Translation2d prePoint = (left ? FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT : FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_LEFT)
        //         .plus(new Translation2d(-0.25, ((isOnCable && !exitingCommunity) ? 0.7 : 0.775) * (left ? 1 : -1)));
            
        //     Translation2d afterPoint = (left ? FieldConfig.getInstance().CHARGE_OUTER_TOP_RIGHT : FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_RIGHT)
        //         .plus(new Translation2d(crossingCable ? 0.5 : 0.5, (exitingCommunity ? 0.775 : (isOnCable ? 0.75 : 0.825)) * (left ? 1 : -1)));
            
        //     Waypoint preWaypoint = ignoreHeading ? new Waypoint(prePoint, null, null) : Waypoint.fromHolonomicPose(new Pose2d(prePoint, (isOnCable && ninetyForCable) ? new Rotation2d(Math.PI/2) : new Rotation2d(Math.PI)));
        //     Waypoint afterWaypoint = ignoreHeading ? new Waypoint(afterPoint, null, null) : Waypoint.fromHolonomicPose(new Pose2d(afterPoint, (isOnCable && ninetyForCable) ? new Rotation2d(Math.PI/2) : new Rotation2d(exitingCommunity ? 0 : Math.PI)));

        //     points.add(exitingCommunity ? preWaypoint : afterWaypoint);

        //     if (crossingCable) {
        //         Translation2d justPastCable = prePoint.interpolate(afterPoint, 0.5);
        //         Waypoint justPastCableWaypoint = ignoreHeading ? new Waypoint(justPastCable) : Waypoint.fromHolonomicPose(new Pose2d(justPastCable, ninetyForCable ? new Rotation2d(Math.PI/2) : new Rotation2d(Math.PI)));
        //         points.add(justPastCableWaypoint);
        //     }

        //     points.add(exitingCommunity ? afterWaypoint : preWaypoint);
        //     return points;
        // }

        public static List<Waypoint> climbingPoints(boolean fromInside, StartingLocation grid) {
            FieldConfig f = FieldConfig.getInstance();
            
            Translation2d edge = (fromInside ? f.CHARGE_OUTER_TOP_LEFT : f.CHARGE_OUTER_TOP_RIGHT).interpolate(fromInside ? f.CHARGE_OUTER_BOTTOM_LEFT : f.CHARGE_OUTER_BOTTOM_RIGHT, 0.5);
            Translation2d center = f.CHARGE_OUTER_TOP_LEFT.interpolate(f.CHARGE_OUTER_BOTTOM_RIGHT, 0.5);

            Translation2d preBalance = edge.plus(new Translation2d(fromInside ? -0.25 : 0.25, 0));
            Translation2d parkTarget = center.plus(new Translation2d(fromInside ? 1.6 : -1.6, 0));

            Rotation2d heading = new Rotation2d(fromInside ? 0 : Math.PI);

            Translation2d sideOffset = new Translation2d(0, grid.climbOffset);

            return List.of(
                new Waypoint(preBalance.plus(sideOffset), heading, null),
                new Waypoint(edge.plus(sideOffset), heading, null),
                new Waypoint(parkTarget.plus(sideOffset), heading, null)
            );
        }

        public static List<Waypoint> climbingCrossover(Translation2d start) {
            // from inside center grid is inferred
            // Translation2d parkTarget = climbingPoints(true, StartingLocation.CENTER_GRID).get(2).getTranslation();

            
            Waypoint past = new Waypoint(start.plus(new Translation2d(1.7 - Units.feetToMeters(1.0) + Units.feetToMeters(2.0), 0)), null, null);
            Waypoint comeBack = new Waypoint(start.plus(new Translation2d(-0.4, 0)), null, null);

            return List.of(past, comeBack);
        }

        // public static RectangularRegionConstraint cableProtectorRegion() {
        //     Translation2d bottomLeft = 

        //     return new RectangularRegionConstraint(, null, new MaxVelocityConstraint(3.0))
        // }

    }    
}
