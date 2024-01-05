package org.frogforce503.lib.auto.follower;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.auto.actions.base.InlineAction;
import org.frogforce503.lib.auto.actions.base.NoopAction;
import org.frogforce503.lib.auto.actions.base.ParallelAction;
import org.frogforce503.lib.auto.actions.base.SeriesAction;
import org.frogforce503.lib.auto.actions.base.WaitUntil;
import org.frogforce503.lib.auto.actions.swerve.PathMarkerAction;
import org.frogforce503.lib.auto.actions.swerve.SwerveFollowPathAction;
import org.frogforce503.lib.trajectory.PlannedPath;

import edu.wpi.first.math.geometry.Rotation2d;

public class RoutineBuilder {

    public List<Action> listOfActions = new ArrayList<Action>();
    public List<PathMarker> listOfMarkers = Arrays.asList();
    private List<PlannedPath> pathsList = null;
    private int currentPathIndex = 1;
    private int lastPathIndex = -1;

    public RoutineBuilder(Action startingAction) {
        listOfActions.add(startingAction);

        if (startingAction instanceof SwerveFollowPathAction)
            lastPathIndex = 0;
    }
    
    public RoutineBuilder() {
        this(new NoopAction());
    }

    public RoutineBuilder(PlannedPath path) {
        this(new SwerveFollowPathAction(path));
    }

    public RoutineBuilder(Runnable startingAction) {
        this(new InlineAction(startingAction));
    }

    public RoutineBuilder(List<PlannedPath> paths) {
        this(paths.get(0));
        this.pathsList = paths;
    }

    public RoutineBuilder first(Action action) {
        listOfActions.add(0, action);
        return this;
    }

    public RoutineBuilder addAction(Action action) {
        listOfActions.add(action);
        if (action instanceof SwerveFollowPathAction)
            lastPathIndex = listOfActions.size()-1;
            
        return this;
    }

    public RoutineBuilder addAction(Runnable lambda) {
        return this.addAction(new InlineAction(lambda));
    }


    public RoutineBuilder conditionally(boolean condition, Consumer<RoutineBuilder> ifTrue, Consumer<RoutineBuilder> ifFalse) {
        if (condition && ifTrue != null) {
            ifTrue.accept(this);
        } else if (!condition && ifFalse != null) {
            ifFalse.accept(this);
        }
        return this;
    }

    public RoutineBuilder conditionally(boolean condition, Consumer<RoutineBuilder> ifTrue) {
        return conditionally(condition, ifTrue, null);
    }

    public RoutineBuilder addDynamicPath(Supplier<PlannedPath> dynamicPath) {
        return this.addAction(new SwerveFollowPathAction(dynamicPath));
    }

    public RoutineBuilder drivePath(PlannedPath path) {
        return this.addDynamicPath(() -> path);
    }

    public RoutineBuilder setPaths(List<PlannedPath> paths) {
        this.pathsList = paths;
        return this.drivePath(paths.get(0));
    }

    public RoutineBuilder drivePath(int pathIndex) {
        boolean err = false;
        if (pathsList == null) {
            System.out.println("ERROR: NO TRAJECTORY LIST GIVEN");
            err = true;
        } else if (pathIndex > pathsList.size() - 1) {
            System.out.println("ERROR: TRAJECTORY OUT OF BOUNDS");
            err = true;
        }

        if (err)
            return this;
        
        return drivePath(this.pathsList.get(pathIndex));
    }

    public RoutineBuilder addNextSegment() {
        currentPathIndex++;
        return drivePath(currentPathIndex-1);
    }

    public RoutineBuilder withDelayedAction(WaitUntil waitAction, Action action) {
        Action replacement = new ParallelAction(this.listOfActions.get(this.listOfActions.size() - 1), new SeriesAction(waitAction, action));
        this.listOfActions.remove(this.listOfActions.size() - 1);
        this.listOfActions.add(replacement);
        return this;
    }

    public RoutineBuilder withDelayedAction(WaitUntil waitAction, Runnable labmda) {
        return this.withDelayedAction(waitAction, new InlineAction(labmda));
    }

    public RoutineBuilder waitSeconds(double seconds) {
        return this.addAction(WaitUntil.timeElapsed(seconds));
    }

    // public RoutineBuilder waitForState(String state) { // TODO: make waiting for state work with state engine
    //     listOfActions.add(WaitUntil.stateAchieved(state));
    //     return this;
    // }

    public Action build() {
        if (this.listOfMarkers.size() == 0)
            return new SeriesAction(this.listOfActions);
        
        List<Action> parallelList = new ArrayList<Action>();
        parallelList.add(new SeriesAction(this.listOfActions));
        for (PathMarker marker : this.listOfMarkers) {
            parallelList.add(new PathMarkerAction(marker));
        }
        
        return new ParallelAction(parallelList);
    }

    // public class RoutineBuilderWithTrajectoryEnd extends RoutineBuilder {
    //     private RoutineBuilderWithTrajectoryEnd() {
    //         super(new SeriesAction());
    //     }

    private SwerveFollowPathAction getLastTrajectory() {
        if (lastPathIndex != -1)
            return ((SwerveFollowPathAction)this.listOfActions.get(lastPathIndex));
        System.out.println("ERROR: NO PATH HAS BEEN DEFINED");
        return null;
    }

    // public RoutineBuilder withMarker(String marker) { // TODO: figure out what to do for this config
    //     SwerveFollowTrajectoryAction traj = getLastTrajectory();
        
    //     if (traj != null)
    //         traj.configureMarkers();

    //     return this;
    // }

    public RoutineBuilder configureMarkers(List<PathMarker> markerConfigs) {
        this.listOfMarkers = markerConfigs;
        return this;
    }

    public RoutineBuilder configureHeadingOverride(Supplier<Rotation2d> headingOverride) {
        SwerveFollowPathAction traj = getLastTrajectory();

        if (traj != null)
            traj.setHeadingOverride(headingOverride);

        return this;
    }
}
