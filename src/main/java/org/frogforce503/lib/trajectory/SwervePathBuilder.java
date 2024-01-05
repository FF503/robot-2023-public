package org.frogforce503.lib.trajectory;

import java.util.Arrays;
import java.util.List;

import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;

// static wrapper for the CustomTrajectoryGenerator for easier use
public class SwervePathBuilder {

    public static PlannedPath generate(TrajectoryConfig config, List<Waypoint> waypoints) {
        CustomTrajectoryGenerator generator = new CustomTrajectoryGenerator();
        
        try {
            generator.generate(config, waypoints);
        } catch (TrajectoryGenerationException exception) {
            System.out.print("TRAJECTORY GENERATION FAILED");
            exception.printStackTrace();
            return null;
        }
        
        return generator.getPlannedPath();
    }

    public static PlannedPath generate(double vMax, double aMax, double vInitial, double vFinal, List<Waypoint> waypoints) {
        return generate(makeConfig(vMax, aMax, vInitial, vFinal), waypoints);
    }

    public static PlannedPath generate(double vMax, double aMax, List<Waypoint> waypoints) {
        return generate(vMax, aMax, 0.0, 0.0, waypoints);
    }

    public static PlannedPath generate(double vMax, double aMax, double vInitial, double vFinal, Waypoint... waypoints) {
        return generate(vMax, aMax, vInitial, vFinal, Arrays.asList(waypoints));
    }

    public static PlannedPath generate(double vMax, double aMax, Waypoint... waypoints) {
        return generate(vMax, aMax, 0.0, 0.0, Arrays.asList(waypoints));
    }

    public static TrajectoryConfig makeConfig(double vMax, double aMax, double vInitial, double vFinal) {
        return new TrajectoryConfig(vMax, aMax)
            .setKinematics(Swerve.getInstance().getKinematics())
            .setStartVelocity(vInitial)
            .setEndVelocity(vFinal);
            // .addConstraints(chargedUpTrajectoryConstraints);
    }

    public static TrajectoryConfig cableConfig(double vMax, double aMax) {

        Translation2d bottomLeft = FieldConfig.getInstance().CHARGE_OUTER_TOP_LEFT.minus(new Translation2d(-0.175, 0));

        if (RobotState.getInstance().getAllianceColor() == AllianceColor.BLUE)
            bottomLeft = new Translation2d(bottomLeft.getX() + 0.175, 0);

        Translation2d topRight = new Translation2d(FieldConfig.getInstance().CHARGE_OUTER_BOTTOM_RIGHT.getX() - 0.175, bottomLeft.getY() + Units.inchesToMeters(65));

        Swerve.getInstance().getField().getObject("charge region top right").setPose(new Pose2d(topRight, new Rotation2d()));
        Swerve.getInstance().getField().getObject("charge region bottom left").setPose(new Pose2d(bottomLeft, new Rotation2d()));

        TrajectoryConstraint chargeStation = new RectangularRegionConstraint(bottomLeft, topRight, new MaxVelocityConstraint(2.0));

        return new TrajectoryConfig(vMax, aMax)
            .setKinematics(Swerve.getInstance().getKinematics())
            .addConstraint(chargeStation);
    }


    // make into method
    // private static List<TrajectoryConstraint> chargedUpTrajectoryConstraints = List.of(
    //             // Cable bump
    //             new RectangularRegionConstraint(
    //                 new Translation2d(Community.chargingStationInnerX, Community.rightY),
    //                 new Translation2d(Community.chargingStationOuterX, Community.chargingStationRightY),
    //                 new MaxVelocityConstraint(cableBumpMaxVelocity)),

    //             // Charging station
    //             new RectangularRegionConstraint(
    //                 new Translation2d(
    //                     Community.chargingStationInnerX - 0.8, Community.chargingStationRightY),
    //                 new Translation2d(
    //                     Community.chargingStationOuterX + 0.8, Community.chargingStationLeftY),
    //                 new MaxVelocityConstraint(chargingStationMaxVelocity)));
}
