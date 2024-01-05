package org.frogforce503.robot2023.auto.tuning;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.auto.follower.RoutineBuilder;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ComplexTestPath extends AutoMode {

    private PlannedPath path;

    public ComplexTestPath() {
        path = SwervePathBuilder.generate(4.0, 3.5, 
            new Waypoint(new Translation2d(), null, new Rotation2d()),
            new Waypoint(new Translation2d(1.0, 0.0)),
            new Waypoint(new Translation2d(2.0, -0.5), new Rotation2d(), new Rotation2d(Math.PI))
        );
    }

    @Override
    public Action routine() {
        return new RoutineBuilder(path).build();
    }

    @Override
    public PlannedPath getPath() {
        return path;
    }
    
}
