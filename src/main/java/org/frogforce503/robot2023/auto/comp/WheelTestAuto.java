package org.frogforce503.robot2023.auto.comp;

import java.util.ArrayList;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.AutoChooser.StartingLocation;
import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.auto.actions.base.ParallelAction;
import org.frogforce503.lib.auto.follower.RoutineBuilder;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.lib.trajectory.SwervePathBuilder;
import org.frogforce503.lib.trajectory.Waypoint;
import org.frogforce503.robot2023.auto.AutoUtil;
import org.frogforce503.robot2023.fields.FieldConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class WheelTestAuto extends AutoMode {

    private PlannedPath goOutAndSpin;

    boolean usingCam = false; // sLEAVE THIS PLEASE

    public WheelTestAuto() {
        goOutAndSpin = SwervePathBuilder.generate(4.0, 3.5,
            new ArrayList<Waypoint>() {{
                add(AutoUtil.Waypoints.scoring(3, 2));
                addAll(AutoUtil.Waypoints.shift(AutoUtil.Waypoints.transitPoints(StartingLocation.LEFT_GRID, true, true), new Translation2d(0, 0.15)));

                add(new Waypoint(FieldConfig.getInstance().GAMEPIECE_A.plus(new Translation2d(-Units.feetToMeters(0.75), Units.inchesToMeters(4))), null, new Rotation2d()));
                addAll(AutoUtil.Waypoints.shift(AutoUtil.Waypoints.transitPoints(StartingLocation.LEFT_GRID, false, true), new Translation2d(0, 0.15)));
                add(AutoUtil.Waypoints.scoring(3, 2).plus(usingCam ? new Translation2d(0.025, 0.075) : new Translation2d(-0.05, Units.feetToMeters(1.2))));
            }}
        );

        // if (usingCam)
        AutoUtil.useFrontOnly = false;
    }    

    @Override
    public Action routine() {
        return new RoutineBuilder()
            .drivePath(goOutAndSpin)
            .build();
    }

    @Override
    public PlannedPath getPath() {
        return goOutAndSpin;
    }

    @Override
    public String getName() {
        return "WheelTestAuto";
    }
    
}
