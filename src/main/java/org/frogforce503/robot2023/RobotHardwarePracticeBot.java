package org.frogforce503.robot2023;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class RobotHardwarePracticeBot extends RobotHardware {

       @Override
    public void initializeConstants() {
        this.backLeftName = "SDS3"; // SDS8
        this.backRightName = "SDS10"; // SDS1
        this.frontLeftName = "SDS1"; // SDS10
        this.frontRightName = "SDS8"; // SDS3

        this.moduleType = SDS_MODULE_TYPE.L3;
        this.swerveCANBus = "CANivore";

        this.pigeonID = 9;

        // Intake Constants
        bottomIntakeRollerID = 6;

        topIntakeRollerP = 0.0000;// = 0.005;
        topIntakeRollerI = 0;
        topIntakeRollerD = 0;
        topIntakeRollerFF = 0.00022; // 0.0001834;

        bottomIntakeRollerP = 0.0000;// = 0.005;
        bottomIntakeRollerI = 0;
        bottomIntakeRollerD = 0;
        bottomIntakeRollerFF = 0.00022; // 0.0001834;

        intakeConeVelocity = 5000;
        intakeCubeVelocity = 2500;

        intakeReversed = false;
        rightMotorReversed = true;
        extenderEncoderReversed = false;

        topIntakeRollerID = 5;

        intakeForwardChannel = 1;
        intakeReverseChannel = 0;
        INTAKE_EXTENSION_TOLERANCE = 3;
        INTAKE_MIN_POSITION = 121;


        // Extender Constants
        extenderID = 7;

        extendingP = 0.26;
        extendingP = 0.25;
        extendingI = 0.0;
        extendingD = 0.0005;


        retractingP = 0.01;
        retractingI = 0;
        retractingD = 0.0006875/4;

        // Indexer Constants
        suctionWheelsID = 4;
        suctionWheelsP = 0.000085;
        suctionWheelsI = 0.000001;
        suctionWheelsD = 3.0; //2.0;
        suctionWheelsFF = 0.000088;
        
        beltMotorID = 3;
        beltMotorP = 0;
        beltMotorI = 0;
        beltMotorD = 0;
        beltMotorFF = 0.00025;

        beltMotorInverted = true;

        flipperWheelID = 7;
        flipperWheelP = 0.0;
        flipperWheelI = 0.0;
        flipperWheelD = 0.0;
        flipperWheelFF = 0.000088;

        // Sensor Constants
        ToFID = 9;
        
        // Escalator Constants
        escalatorID = 8;
        escalatorP = 2.0;
        escalatorI = 0.0;
        escalatorD = 0.10;
        escalatorFF = 0.0;
        escalatorIzone = 0.0;

        escalatorEncoderID = 12;

        escalatorConeP = 3.25;
        escalatorConeI = 0.0;
        escalatorConeD = 0.10;
        escalatorConeFF = 0.0;
        escalatorConeIzone = 0.0;
        //limit switch
        escalatorZeroID = 0;

        LOW_SCORING_HEIGHT = 30;
        MID_SCORING_HEIGHT = 44;
        MID_CUBE_SCORE_HEIGHT = 48; 
        HIGH_SCORING_HEIGHT = 61; // previously 65
        HIGH_SCORING_HEIGHT_AUTON = 61;
        HIGH_CUBE_SCORE_HEIGHT = 63;
        
        candleID = 10;

        claw3ShiftForwardID = 4;
        claw3ShiftReverseID = 5;

       intakeLockForwardID = 15;
       intakeLockReverseID = 2;

       wristShiftForwardID = 7;
       wristShiftReverseID = 6;
      
        // objPX * calibrationDistance / objlength
        // TODO: recalibrate for arducam
        kConeFocalLength = (215 * 1) / Constants.CONE_SQUARE_LENGTH;
        kCubeFocalLength = (218 * 1) / Constants.CUBE_SIDE_LENGTH;
        kxFov = 90;
        kImageH = 1920; 
        kImageW = 1080;

        // TODO: measure this stuff
        this.kWheelbaseLength = Units.inchesToMeters(21.0); 
        this.kWheelbaseWidth = Units.inchesToMeters(21.0); 

        // full robot size is 32.625 x 36.625 inches (with bumpers)
        this.wheelDiameter = Units.inchesToMeters(4);

        this.fullRobotLength = Units.inchesToMeters(33.5);
        this.fullRobotWidth = Units.inchesToMeters(33.5);


        // Swerve Module Positions (relative to the center of the drive base)
        kVehicleToFrontLeft = new Translation2d(kWheelbaseWidth / 2, kWheelbaseLength / 2);
        kVehicleToFrontRight = new Translation2d(kWheelbaseWidth / 2, -kWheelbaseWidth / 2);
        kVehicleToBackRight = new Translation2d(-kWheelbaseWidth / 2, -kWheelbaseLength / 2);
        kVehicleToBackLeft = new Translation2d(-kWheelbaseWidth / 2, kWheelbaseLength / 2);

        kModulePositions = new Translation2d[] { kVehicleToFrontLeft, kVehicleToFrontRight,
                kVehicleToBackLeft, kVehicleToBackRight };

    }

    
}
