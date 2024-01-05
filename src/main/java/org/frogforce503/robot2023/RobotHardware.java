/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.frogforce503.robot2023;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import org.frogforce503.lib.util.Util;
import org.frogforce503.robot2023.RobotState.Bot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Add your docs here.
 */
public abstract class RobotHardware {

    private static RobotHardware instance = null;

    // Constants
    // SwerveFileNames
    public String backLeftName;
    public String backRightName;
    public String frontLeftName;
    public String frontRightName;

    public SDS_MODULE_TYPE moduleType;
    public String swerveCANBus;

    // Intake Constants
    public int bottomIntakeRollerID;
    public int topIntakeRollerID;

    public int extenderID;
    public double extendingP;
    public double extendingI;
    public double extendingD;

    public boolean extenderEncoderReversed;

    public double retractingP;
    public double retractingI;
    public double retractingD;

    public int rightExtenderID;

    public double bottomIntakeRollerP;
    public double bottomIntakeRollerI;
    public double bottomIntakeRollerD;
    public double bottomIntakeRollerFF;

    public double topIntakeRollerP;
    public double topIntakeRollerI;
    public double topIntakeRollerD;
    public double topIntakeRollerFF;

    public double intakeConeVelocity;
    public double intakeCubeVelocity;
    public double intakeCubeAutoVelocity;

    public boolean intakeReversed;
    public boolean rightMotorReversed;

    public int intakeForwardChannel;
    public int intakeReverseChannel;
    public int INTAKE_EXTENSION_TOLERANCE;
    public int INTAKE_MIN_POSITION;

    //Indexer Constants
    public int suctionWheelsID;
    public int beltMotorID;
    public int flipperWheelID;

    public double suctionWheelsP;
    public double suctionWheelsI;
    public double suctionWheelsD;
    public double suctionWheelsFF;

    public double flipperWheelP;
    public double flipperWheelI;
    public double flipperWheelD;
    public double flipperWheelFF;

    public double beltMotorP;
    public double beltMotorI;
    public double beltMotorD;
    public double beltMotorFF;

    public boolean beltMotorInverted;


    //Escalator Constants
    public int escalatorID;
    public double escalatorP;
    public double escalatorI;
    public double escalatorD;
    public double escalatorFF;
    public double escalatorIzone;

    public int escalatorEncoderID;
    public double escalatorZero;

    public double escalatorConeP;
    public double escalatorConeI;
    public double escalatorConeD;
    public double escalatorConeFF;
    public double escalatorConeIzone;

    public double LOW_SCORING_HEIGHT;
    public double HIGH_SCORING_HEIGHT;
    public double HIGH_SCORING_HEIGHT_AUTON;
    public double MID_SCORING_HEIGHT;
    public double MID_CUBE_SCORE_HEIGHT;
    public double HIGH_CUBE_SCORE_HEIGHT;
    public double HIGH_CUBE_SCORE_HEIGHT_AUTON;

    public int claw3ShiftForwardID;
    public int claw3ShiftReverseID;

    public int intakeLockForwardID;
    public int intakeLockReverseID;
    
    public int wristShiftForwardID;
    public int wristShiftReverseID;

    public int pigeonID;

    // Limelight Constants
    public double limelightVerticalAngle;
    public double limelightFloorClearance;
    public double limelightMagnifiedOffset;
    public double visionTargetingRangeTolerance;

    // Sensor Constants
    public int ToFID;

    //DriverFeedback Constants
    public int candleID;

    // Swerve Calculations Constants (measurements are in inches)
    public double kWheelbaseLength;
    public double kWheelbaseWidth;
    public double wheelDiameter;

    public double fullRobotWidth; // with bumpers
    public double fullRobotLength; // with bumpers
  

    // Swerve Module Positions (relative to the center of the drive base)
    public Translation2d kVehicleToFrontRight;
    public Translation2d kVehicleToBackRight;
    public Translation2d kVehicleToFrontLeft;
    public Translation2d kVehicleToBackLeft;
    public Translation2d[] kModulePositions;

    public double limelightTXTolerance;

    public String limelightURL;
    public String usbCameraURL;

    public int escalatorZeroID;

    public double[][] coneDistanceMapValues;

    public double kConeFocalLength;
    public double kCubeFocalLength;
    public double kxFov;
    public int kImageW;
    public int kImageH;

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }

    public static RobotHardware getInstance() {
        if (instance == null) {
            if (RobotState.getInstance().getCurrentRobot().equals(Bot.Automatic)) {
                RobotState.getInstance().setCurrentRobot(Util.parseRobotNameToEnum(Util.readRobotName()));
            }
            switch (RobotState.getInstance().getCurrentRobot()) {
                case PracticeBot:
                    instance = new RobotHardwarePracticeBot();
                    break;
                case CompBot:
                    instance = new RobotHardwareCompBot();
                    break;
                case MiniMe:
                    instance = new RobotHardwareMiniMe();
                    break;
                case Automatic:
                default:
                    System.err.println("Robot should not be set to automatic... something went wrong");
                    break;
            }
            instance.initializeConstants();
            // Util.setPseudoInverseForwardKinematicsMatrix();
        }
        return instance;
    }

    public static enum SDS_MODULE_TYPE {
        L1(Units.feetToMeters(13.5), 8.14), L2(Units.feetToMeters(16.3), 6.75), L3(Units.feetToMeters(18), 6.12);

        public double MAX_DRIVE_SPEED_METERS_SEC;
        public double driveGearRatio;

        SDS_MODULE_TYPE(double max, double gs) {
            MAX_DRIVE_SPEED_METERS_SEC = max;
            driveGearRatio = gs;
        }
    }

    public abstract void initializeConstants();
}
