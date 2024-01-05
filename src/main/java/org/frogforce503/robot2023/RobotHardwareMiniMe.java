
package org.frogforce503.robot2023;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RobotHardwareMiniMe extends RobotHardware {

    @Override
    public void initializeConstants() {
        this.backLeftName = "SDS10";
        this.backRightName = "SDS8";
        this.frontLeftName = "SDS1";
        this.frontRightName = "SDS3";

        this.moduleType = SDS_MODULE_TYPE.L3;
        this.swerveCANBus = "CANivore";

        this.pigeonID = 9;

        // TODO: measure this stuff
        this.kWheelbaseLength = Units.inchesToMeters(14.75); // long side (left and right) (30 inch side)
        this.kWheelbaseWidth = Units.inchesToMeters(14.75); // short side (frotnt and back) (26 inch side)
        // full robot size is 32.625 x 36.625 inches (with bumpers)
        this.wheelDiameter = Units.inchesToMeters(4);

        // Swerve Module Positions (relative to the center of the drive base)
        kVehicleToFrontLeft = new Translation2d(kWheelbaseWidth / 2, kWheelbaseLength / 2);
        kVehicleToFrontRight = new Translation2d(kWheelbaseWidth / 2, -kWheelbaseWidth / 2);
        kVehicleToBackRight = new Translation2d(-kWheelbaseWidth / 2, -kWheelbaseLength / 2);
        kVehicleToBackLeft = new Translation2d(-kWheelbaseWidth / 2, kWheelbaseLength / 2);

        kModulePositions = new Translation2d[] { kVehicleToFrontLeft, kVehicleToFrontRight, kVehicleToBackRight,
                kVehicleToBackLeft };

    }

}