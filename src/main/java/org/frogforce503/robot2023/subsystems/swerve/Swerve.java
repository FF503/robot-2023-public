package org.frogforce503.robot2023.subsystems.swerve;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import javax.sound.sampled.Line;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import org.frogforce503.lib.auto.follower.SwervePathFollower;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.util.TaskScheduler;
import org.frogforce503.robot2023.OI;
import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.RobotState.GamePiece;
import org.frogforce503.robot2023.planners.LineupPlanner;
import org.frogforce503.robot2023.planners.ParkPlanner;
import org.frogforce503.robot2023.planners.LineupPlanner.LineupState;
import org.frogforce503.robot2023.planners.LineupPlanner.Position.LineupMethod;
import org.frogforce503.robot2023.planners.ParkPlanner.Position.ParkingMethod;
import org.frogforce503.robot2023.subsystems.DriverFeedback;
import org.frogforce503.robot2023.subsystems.Intake;
import org.frogforce503.robot2023.subsystems.Subsystem;
import org.frogforce503.robot2023.subsystems.DriverFeedback.LEDLights;
import org.frogforce503.robot2023.subsystems.vision.cameras.Jetson;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Swerve extends Subsystem {
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;

    private SwerveModule[] modules;
    private Pigeon2 imu;

    // math
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;

    // controllers
    private SwervePathFollower pathFollower;
    private PIDController stabilizationController;
    private PIDController snappingController;
    private PIDController autoSnappingController;

    private double stabilizationP = 0; // FIXME: tune this pid
    private double stabilizationI = 0;
    private double stabilizationD = 0;

    private double translationP = 0;
    private double translationI = 0;
    private double translationD = 0;

    static Swerve instance;

    // simulation stuff (field is technically used in both sim and real)
    private Field2d field;
    private double _simHeading;
    private Pose2d simPose = new Pose2d();
    private ChassisSpeeds simSpeeds = ZERO_CHASSIS_SPEED;
    private boolean _isPlottingRobotPose = false;

    // constants
    private final double SLOW_TRANSLATION_METERS_PER_SECOND = 1.5;
    private final double SUPER_SLOW_TRANSLATION_METERS_PER_SECOND = 0.5;
    private final double FAST_TRANSLATION_METERS_PER_SECOND = Units.feetToMeters(18);

    private final double SLOW_ROTATION_RADIANS_PER_SECOND = Math.PI / 6; // 30 deg per second
    private final double SUPER_SLOW_ROTATION_RADIANS_PER_SECOND = Math.PI / 12.0; // 30 deg per second
    private final double FAST_ROTATION_RADIANS_PER_SECOND = 7 * (Math.PI / 2); // 270 deg per second

    private final double MAX_ACCELERATION_METERS_PER_SEC_PER_SEC = FAST_TRANSLATION_METERS_PER_SECOND * 1.2;
    private final double MAX_ANGULAR_ACCLERATION_RAD_PER_SEC_PER_SEC = 5 * Math.PI / 2.0;

    public static final double JOYSTICK_DRIVE_TOLERANCE = 0.2;
    public static final double JOYSTICK_TURN_TOLERANCE = 0.075;

    public static final ChassisSpeeds ZERO_CHASSIS_SPEED = new ChassisSpeeds();
    private final Rotation2d DEFAULT_STABALIZATION_HEADING = Rotation2d.fromDegrees(503.503503);

    // current state of drivetrain
    private ChassisSpeeds currentVelocity = new ChassisSpeeds();
    private ChassisSpeeds lastVelocity = new ChassisSpeeds();
    private double lastTime;
    private Rotation2d lastTheta;
    private boolean brakeModeEnabled = false;

    // control of the robot
    private SwerveControlState previousControlState = SwerveControlState.STOPPED;
    private SwerveControlState swerveControlState = SwerveControlState.STOPPED;
    private ChassisSpeeds pathFollowerSpeeds = ZERO_CHASSIS_SPEED;
    private ChassisSpeeds teleopChassisSpeeds = ZERO_CHASSIS_SPEED;
    private ChassisSpeeds backupChassisSpeeds = ZERO_CHASSIS_SPEED;
    private ChassisSpeeds lineupChassisSpeeds = ZERO_CHASSIS_SPEED;
    private ChassisSpeeds parkingChassisSpeeds = ZERO_CHASSIS_SPEED;
    private ChassisSpeeds fetchingChassisSpeeds = ZERO_CHASSIS_SPEED;

    // teleop values
    private double stabilizationHeading = 503.503 * Math.PI / 180;
    private boolean slowmodeEnabled = false;
    private boolean superSlowMode = false;
    private boolean isRobotCentric = false;
    private boolean isSnapping = false;
    private boolean drivingAboveTolerance = false;
    private Debouncer stabalizationDebouncer = new Debouncer(0.45);
    private boolean shouldStabilize = false;
    private boolean shouldHaveStabilized = false;

    private Translation2d lineupTarget = new Translation2d();
    private Translation2d lineupOffset = new Translation2d();
    // private PIDController tagAlignmentController;
    private ProfiledPIDController lineupController;
    private final double MAX_LINEUP_DISTANCE = Units.feetToMeters(6);

    private boolean shouldSnapToLoad = false;

    private Translation2d parkingTarget = new Translation2d();
    private Translation2d parkingOffset = new Translation2d();
    private ProfiledPIDController parkcontroller;
    private PIDController balancingController;

    private Translation2d backupTarget = new Translation2d();
    private PIDController backupController;

    private double characterizationDriveVoltage = 0;

    private ShuffleboardLayout swerveTableList;
    private NetworkTable swerveTable;

    private double tolerance = 0.75;

    private double dt = 0;
    private double t0 = 0;

    /**
     * Creates a new Drive.
     */
    public Swerve() {
        this.frontLeftModule = new SwerveModule(Robot.bot.frontLeftName, ModuleLocation.FrontLeft);
        this.frontRightModule = new SwerveModule(Robot.bot.frontRightName, ModuleLocation.FrontRight);
        this.backLeftModule = new SwerveModule(Robot.bot.backLeftName, ModuleLocation.BackLeft);
        this.backRightModule = new SwerveModule(Robot.bot.backRightName, ModuleLocation.BackRight);

        this.modules = new SwerveModule[] {
                this.frontLeftModule,
                this.frontRightModule,
                this.backLeftModule,
                this.backRightModule
        };

        field = new Field2d();
        // field.getObject("ball").setPose(new Pose2d(3, 3, Rotation2d.fromDegrees(0)));

        SmartDashboard.putData("Field", field);

        imu = new Pigeon2(Robot.bot.pigeonID, Robot.bot.swerveCANBus);
        imu.configFactoryDefault();
        // imu.configMountPosePitch(180);
        // imu.configMountPose(-27.8174, 82.4247, -79.9805);

        // imu.config
        setAngle(0);

        Translation2d m_frontLeftLocation = new Translation2d(Robot.bot.kWheelbaseWidth, Robot.bot.kWheelbaseWidth);
        Translation2d m_frontRightLocation = new Translation2d(Robot.bot.kWheelbaseWidth, -Robot.bot.kWheelbaseWidth);
        Translation2d m_backLeftLocation = new Translation2d(-Robot.bot.kWheelbaseWidth, Robot.bot.kWheelbaseWidth);
        Translation2d m_backRightLocation = new Translation2d(-Robot.bot.kWheelbaseWidth, -Robot.bot.kWheelbaseWidth);

        kinematics = new SwerveDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        odometry = new SwerveDrivePoseEstimator(
            kinematics, 
            getAngleRotation2d(), 
            getSwerveModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2)),
            VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(3))
        );

        stabilizationController = new PIDController(8.0, 0, 0.0);
        snappingController = new PIDController(8.0, 0, 0.8);
                // // stabilizationController = new ProfiledPIDController(0.0, 0, 0.0,
                // new TrapezoidProfile.Constraints(2.5 * (Math.PI / 2), 25 * (Math.PI / 2)));
        
        PIDController autoXController = new PIDController(2.5, 0.0, 0.0);
        PIDController autoYController = new PIDController(2.5, 0.0, 0.15);
        PIDController autoThetaController = new PIDController(3.0, 0.0, 0.2);
        pathFollower = new SwervePathFollower(autoXController, autoYController, autoThetaController);

        lineupController = new ProfiledPIDController(12.0, 0.0, 1.3, 
            new TrapezoidProfile.Constraints(FAST_TRANSLATION_METERS_PER_SECOND * 0.25, MAX_ACCELERATION_METERS_PER_SEC_PER_SEC * 0.7)
        );
        // lineupController.setTolerance(0.01);
        // lineupController = new PIDController(RobotBase.isReal() ? 6.0 : 3.0, 0.0, RobotBase.isReal() ? 1.2 : 0.0);
        // tagAlignmentController = new PIDController(1.0, 0.0, 0.0);
        snappingController = new PIDController(2.25, 0.0, 0.225);
        snappingController.enableContinuousInput(-Math.PI, Math.PI);
        snappingController.setTolerance(Math.toRadians(1));

        autoSnappingController = new PIDController(2.5, 0.0, 0.1);
        autoSnappingController.enableContinuousInput(-Math.PI, Math.PI);
        autoSnappingController.setTolerance(Math.toRadians(1));

        backupController = new PIDController(1, 0, 0);
        backupController.setTolerance(Units.inchesToMeters(0.125));

        // parkcontroller = new Profiled3.0PIDController(1.0, 0.0, 0.0, 
        //     new TrapezoidProfile.Constraints(FAST_TRANSLATION_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SEC_PER_SEC)
        // );
        balancingController =  new PIDController(32.0, 0.0, 0.0);
        lineupController.setTolerance(0.01);

        this.stabilizationController.setTolerance(Math.toRadians(1));
        this.stabilizationController.enableContinuousInput(-Math.PI, Math.PI);
        

        this.snappingController.setTolerance(Math.toRadians(1));
        this.snappingController.enableContinuousInput(-Math.PI, Math.PI);

        this.lastTime = Timer.getFPGATimestamp();
        this.lastTheta = getAngleRotation2d();

        for (SwerveModule module : modules) {
            SmartDashboard.putString(module.locationName + " ID", module.moduleName);
        }

        // Shuffleboard.getTab("GameSpec").addString("Search Times", () ->
        // this.searchTimes + "");
        // Shuffleboard.getTab("GameSpec").addString("Disable Times", () ->
        // this.disableTimes + "");
        // Shuffleboard.getTab("GameSpec").addBoolean("CargoFetcher Sees Ball",
        // CargoFetcher.getInstance()::isCurrentlySearching);

        swerveTableList = Shuffleboard.getTab("GameSpec").getLayout("Swerve", BuiltInLayouts.kList);
        swerveTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("GameSpec")
                .getSubTable("Swerve");

        swerveTableList.withPosition(10, 0).withSize(2, 5);
    }

    // public Command createAutoCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    //     return new SequentialCommandGroup(
    //         // new InstantCommand(() -> {
    //         // // Reset odometry for the first path you run during auto TODO: do this
    //         // // if(isFirstPath){
    //         // //     this.resetOdometry(traj.getInitialHolonomicPose());
    //         // // }
    //         // }),
    //         new SwerveFollowPathCommand(
    //             traj, 
    //             this::getPoseMeters, // Pose supplier
    //             this.kinematics, // SwerveDriveKinematics
    //             new PIDController(translationP, translationI, translationD), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //             new PIDController(translationP, translationI, translationD), // Y controller (usually the same values as X controller)
    //             new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //             this::setSwerveModuleStates // Module states consumer
    //         ),
    //         new InstantCommand(() -> stop())
    //     );
        
    // }

    public static Swerve getInstance() {
        return instance == null ? instance = new Swerve() : instance;
    }

    public void initTelemetry() {
        swerveTableList.withPosition(0, 0).withSize(2, 5);

        swerveTableList
                .addNumber("Stabilization P", () -> swerveTable.getEntry("Stabilization P").getDouble(stabilizationP))
                .withSize(1, 1);
        swerveTableList
                .addNumber("Stabilization I", () -> swerveTable.getEntry("Stabilization I").getDouble(stabilizationI))
                .withSize(1, 1);
        swerveTableList
                .addNumber("Stabilization D", () -> swerveTable.getEntry("Stabilization D").getDouble(stabilizationD))
                .withSize(1, 1);


        swerveTableList
                .addNumber("Auto Translation P", () -> swerveTable.getEntry("Auto Translation P").getDouble(translationP))
                .withSize(1, 1);
        swerveTableList
                .addNumber("Auto Translation I", () -> swerveTable.getEntry("Auto Translation I").getDouble(translationI))
                .withSize(1, 1);
        swerveTableList
                .addNumber("Auto Translation D", () -> swerveTable.getEntry("Auto Translation D").getDouble(translationD))
                .withSize(1, 1);
                
    }

    public void updatePIDValues() {
        stabilizationController.setP(swerveTable.getEntry("Stabilization P").getDouble(0.0));
        stabilizationController.setI(swerveTable.getEntry("Stabilization I").getDouble(0.0));
        stabilizationController.setD(swerveTable.getEntry("Stabilization D").getDouble(0.0));

        translationP = swerveTable.getEntry("Auto Translation P").getDouble(0.0);
        translationI = swerveTable.getEntry("Auto Translation I").getDouble(0.0);
        translationI = swerveTable.getEntry("Auto Translation D").getDouble(0.0);
    }

    @Override
    public void onLoop() {
        updateOdometry();
        this.drawFields();
        this.control();

        if (RobotBase.isSimulation())
            this.simulationPeriodic();
    }

    /**
     * Main control function that is called in write periodic outputs. This should
     * be the ONLY place that Swerve.drive is ever called
     */
    private void control() {
        ChassisSpeeds speeds;
        boolean isOpenLoop = false;

        switch (this.swerveControlState) {
            case TELEOP:
                this.driveWithStabilization(false);

                this.teleopChassisSpeeds = correctDrift(teleopChassisSpeeds); // TODO: test and see if this works!!

                cleanseChassisSpeeds(teleopChassisSpeeds);
                speeds = this.teleopChassisSpeeds;
                isOpenLoop = true;
                break;
            case BACKUP: 
                this.updateBackupSpeeds();

                cleanseChassisSpeeds(backupChassisSpeeds);

                speeds = this.backupChassisSpeeds;
                isOpenLoop = true;

                break;
            case PATH_FOLLOWING:
                cleanseChassisSpeeds(this.pathFollowerSpeeds);

                this.pathFollowerSpeeds = correctDrift(pathFollowerSpeeds); // TODO: test and see if this works!!

                speeds = this.pathFollowerSpeeds;
                break;
            case CHARACTERIZATION:
                for (SwerveModule module : this.modules) {
                    module.setOpenLoopModuleState(this.characterizationDriveVoltage);
                }
                // System.out.println("Setting open loop: " + this.characterizationDriveVoltage);
                speeds = ZERO_CHASSIS_SPEED;
                break;
                // continue to zero chassis speed
            // case BALL_FETCHING:
            //     DriverFeedback.getInstance().setLEDState(LEDStates.RAINBOW);
            //     fetchingChassisSpeeds = FetchingHandler.getInstance().getFetchingSpeeds();
            //     this.driveWithStabilization();
            //     cleanseChassisSpeeds(teleopChassisSpeeds);
            //     cleanseChassisSpeeds(fetchingChassisSpeeds);

            //     fetchingChassisSpeeds.vxMetersPerSecond += teleopChassisSpeeds.vxMetersPerSecond;
            //     fetchingChassisSpeeds.vyMetersPerSecond += teleopChassisSpeeds.vyMetersPerSecond;
            //     fetchingChassisSpeeds.omegaRadiansPerSecond += teleopChassisSpeeds.omegaRadiansPerSecond;

            //     speeds = fetchingChassisSpeeds;
            //     break;
            case LINEUP:
                this.updateLineupSpeeds();
                cleanseChassisSpeeds(lineupChassisSpeeds);
                speeds = lineupChassisSpeeds;
                break;
            case PARK:
                this.updateParkingSpeeds();
                cleanseChassisSpeeds(parkingChassisSpeeds);
                speeds = parkingChassisSpeeds;
                break;
            case FETCHING:
                this.updateFetchingSpeeds();
                cleanseChassisSpeeds(fetchingChassisSpeeds);
                speeds = fetchingChassisSpeeds;
                isOpenLoop = true;
                break;
            case SNAPPING:
                // continue to zero chassis speed
                speeds = ZERO_CHASSIS_SPEED;
                // break;
            default:
                // this is stopped mode (snapping also does this stuff)
                speeds = ZERO_CHASSIS_SPEED;
                checkForStateChange();
                break;
        }

        dt = Timer.getFPGATimestamp() - t0;
        dt = dt < 0.25 ? dt : 0.02;
        
        t0 = Timer.getFPGATimestamp();

        if (this.swerveControlState != SwerveControlState.CHARACTERIZATION)
            this.drive(speeds, isOpenLoop);

        this.lastVelocity = this.currentVelocity;
    }

    private void checkForStateChange() {
        if (RobotState.isTeleop() && OI.tryingToDrive()) {
            this.setControlState(SwerveControlState.TELEOP);
        }
        if (RobotState.isAutonomous()) { // TODO: Fix this to work with path planner stopping
            this.setControlState(SwerveControlState.PATH_FOLLOWING);
        }
    }

    //@formatter:on
    int searchTimes = 0;
    int disableTimes = 0;

    public void setControlState(SwerveControlState state) {
        this.previousControlState = state != this.swerveControlState ? this.swerveControlState : this.previousControlState;
        this.swerveControlState = state;

        if (state == SwerveControlState.LINEUP)
            enableBrakeMode(true);

        if (RobotBase.isSimulation() && (state == SwerveControlState.STOPPED || state == SwerveControlState.SNAPPING))
            this.simSpeeds = ZERO_CHASSIS_SPEED;


        // if (state.ledColor != null){
        //     DriverFeedback.getInstance().setColor(state.ledColor);
        // } else {
        //     Intake.getInstance().updateDriverFeebackLEDs();
        // }
        // else
        //   //  DriverFeedback.getInstance().setOff();
    }

    public void setCharacterizationDriveVoltage(double voltage) {
        this.characterizationDriveVoltage = voltage;
    }

    public double getIndividualDriveModuleVelocity() {
        return this.modules[0].getVelocity();
    }

    public SwerveControlState getControlState() {
        return this.swerveControlState;
    }

    /**
     * Draws the path and the robot on the dashboard.
     */
    private void drawFields() {

        // List<Pose2d> ballList = new ArrayList<>();
       // ballList.add(JetsonProcessor.getInstance().getFieldRelativePosition());
        // field.getObject("ball").setPoses(ballList);
        field.setRobotPose(getPoseMeters());
        if (_isPlottingRobotPose)
            this.plotRobotPathPoint();
    }

    public Field2d getField() {
        return field;
    }

    public void checkSlowMode(boolean trigger) {
        this.slowmodeEnabled = trigger;
    }

    public void checkRobotCentric(boolean trigger) {
        this.isRobotCentric = trigger;
    }

    public void toggleRobotCentric() {
        this.isRobotCentric = !this.isRobotCentric;
    }

    public void toggleSlowMode() {
        this.slowmodeEnabled = !this.slowmodeEnabled;
    }

    public void checkSuperSlowMode(boolean enable) {
        this.superSlowMode = enable;
    }

    // /**
    //  * Snap the ROBOT to a heading (this will only work in Teleop as of now)
    //  * 
    //  * @param angle Angle in degres to snap to
    //  */
    // public void snapToAngle(double angle) {
    //     stabilizationHeading = angle * Math.PI / 180;
    //     stabilizationController.setSetpoint(stabilizationHeading);
    //     isSnapping = true;
    // }

    public Rotation2d getAngleToAim(Translation2d pointToAim) {
        Pose2d currentPose = getPoseMeters();
        double angleToTarget = Math.atan2(pointToAim.getY() - currentPose.getY(), pointToAim.getX() - currentPose.getX());
        return new Rotation2d(angleToTarget);
    }

    public void disableSnapping() {
        isSnapping = false;
    }

    public void snapToLoad() {
        if (Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE)
            this.snapToAngle(Math.PI/2 * (org.frogforce503.robot2023.RobotState.getInstance().getAllianceColor() == AllianceColor.BLUE ? 1 : -1));
        else
            this.snapToBackward();
    }

    public void snapToBackward() {
        this.snapToAngle(Math.PI);
    }

    public void snapToAngle(double angleRad) {
        if (OI.tryingToDrive()) {
            this.shouldSnapToLoad = true;
            stabilizationHeading = angleRad;
            stabilizationController.setSetpoint(stabilizationHeading);
        }
    }

    /**
     * Main teleop control loop
     */
    private void driveWithStabilization(boolean limitAcceleration) {

        SmartDashboard.putNumber("Driver Stick Y", -OI.getDriverLeftYValue());
        SmartDashboard.putNumber("Driver Stick X", -OI.getDriverLeftXValue());

        if (RobotState.isAutonomous()) {
            // something is wrong
            this.teleopChassisSpeeds = ZERO_CHASSIS_SPEED;
            return;
        }

        double rotationOutput = Math.pow(-OI.getDriverRightXValue(), 3);;

        Translation2d translation = new Translation2d(
            -OI.getDriverLeftYValue(),
            -OI.getDriverLeftXValue()
        );

        drivingAboveTolerance = (translation.getNorm() > JOYSTICK_DRIVE_TOLERANCE);      
        translation = translation.times(drivingAboveTolerance ? 1 : 0);

        translation = translation.times(drivingAboveTolerance ? 1 : 0);

        if (OI.tryingToAutoAlign() && LineupPlanner.getInstance().getLineupState() == LineupState.INELIGIBLE && !this.shouldSnapToLoad)
            this.snapToBackward();
        
        double rotationScaler = FAST_ROTATION_RADIANS_PER_SECOND;
        double translationScaler = FAST_TRANSLATION_METERS_PER_SECOND;
        
        if (this.superSlowMode) {
            rotationScaler = SUPER_SLOW_ROTATION_RADIANS_PER_SECOND;
            translationScaler = SUPER_SLOW_TRANSLATION_METERS_PER_SECOND;
        } else if (this.slowmodeEnabled) {
            rotationScaler = SLOW_ROTATION_RADIANS_PER_SECOND;
            translationScaler = SLOW_TRANSLATION_METERS_PER_SECOND;
        }

        this.shouldStabilize = stabalizationDebouncer.calculate((Math.abs(rotationOutput) < JOYSTICK_TURN_TOLERANCE) && drivingAboveTolerance);

        if (this.shouldStabilize) {
            if (!this.shouldHaveStabilized && !this.shouldSnapToLoad) {
                stabilizationHeading = getAngleDegrees() * Math.PI / 180;
                stabilizationController.setSetpoint(stabilizationHeading);
            }

            double stab = stabilizationController.calculate(getAngleRotation2d().getRadians(), stabilizationHeading);

            rotationOutput = (Math.abs(stabilizationController.getPositionError()) < Math.toRadians(10)) ? stab : snappingController.calculate(getAngleRotation2d().getRadians(), stabilizationHeading);
        } else {
            // open loop
            rotationOutput *= rotationScaler;
            this.shouldSnapToLoad = false;
        }

        this.shouldHaveStabilized = this.shouldStabilize;

        translation = translation.times(translationScaler);

        SmartDashboard.putNumber("xSpeed", translation.getX());
        SmartDashboard.putNumber("ySpeed", translation.getY());
        SmartDashboard.putNumber("Stabilization Heading", stabilizationHeading);
        SmartDashboard.putNumber("thetaSpeed", rotationOutput);

        if (this.isRobotCentric) {
            teleopChassisSpeeds.vxMetersPerSecond = translation.getX();
            teleopChassisSpeeds.vyMetersPerSecond = translation.getY();
            teleopChassisSpeeds.omegaRadiansPerSecond = rotationOutput;
        } else {
            teleopChassisSpeeds = (ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(),
                translation.getY(),
                    rotationOutput, getAngleRotation2d()));
        }

        if (limitAcceleration)
            teleopChassisSpeeds = this.limitAcceleration(teleopChassisSpeeds);

        double xVelAbs = Math.abs(teleopChassisSpeeds.vxMetersPerSecond);
        double yVelAbs = Math.abs(teleopChassisSpeeds.vyMetersPerSecond);
        double oVelAbs = Math.abs(teleopChassisSpeeds.omegaRadiansPerSecond);

        // System.out.println(translation);

        if (!isSnapping && xVelAbs < 0.05 && yVelAbs < 0.05
                && oVelAbs < 0.05
                && (Math.pow(getVelocity().vxMetersPerSecond,2) + Math.pow(getVelocity().vyMetersPerSecond,2) < (0.05 * 0.05))
                && getVelocity().omegaRadiansPerSecond < 0.05) {
            snapModulesTo(ModuleSnapPositions.DEFENSE);
        }
    }

    public void startBackup() {
        System.out.println("Backup start");
        backupTarget = getPoseMeters().getTranslation().minus(new Translation2d(Units.inchesToMeters(1.25), getAngleRotation2d()));
        this.setControlState(SwerveControlState.BACKUP);
    }

    public void disableBackup() {
        this.setControlState(SwerveControlState.TELEOP);
    }

    private void updateBackupSpeeds() {
        Translation2d current = getPoseMeters().getTranslation();
        Translation2d disp = backupTarget.minus(current);

        SmartDashboard.putNumber("backup_distance_inches", Units.metersToInches(disp.getNorm()));

        if (disp.getNorm() < Units.inchesToMeters(0.3)) {
            // TaskScheduler.getInstance().schedule("backupComplete", () -> {
            //     DriverFeedback.getInstance().setColor(LEDLights.RED);
            //     TaskScheduler.getInstance().allow("backupComplete");
            // });

        }

        if (disp.getNorm() > Units.inchesToMeters(4)) {
            this.backupChassisSpeeds = ZERO_CHASSIS_SPEED;
            this.setControlState(SwerveControlState.TELEOP);
            return;
        }

        double speed = Math.min(backupController.calculate(disp.getNorm() , 0), 0.5) * 10.0;
        Translation2d vel = new Translation2d(speed, getAngleRotation2d());

        this.backupChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vel.getX(), vel.getY(), 0, getAngleRotation2d());
        this.backupChassisSpeeds.vyMetersPerSecond = 0; // ensure no horizontal robot-centric
        SmartDashboard.putNumber("backup_speed", this.backupChassisSpeeds.vxMetersPerSecond);
    }


    private ChassisSpeeds limitAcceleration(ChassisSpeeds desiredSpeed) {
        // linear
        Translation2d lastLinearVelocity = new Translation2d(this.lastVelocity.vxMetersPerSecond, this.lastVelocity.vyMetersPerSecond);
        Translation2d desiredLinearVelocity = new Translation2d(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);

        Translation2d acceleration = desiredLinearVelocity.minus(lastLinearVelocity).div(dt);
        Translation2d normalizedAcceleration = new Translation2d(Math.min(acceleration.getNorm(), MAX_ACCELERATION_METERS_PER_SEC_PER_SEC), acceleration.getAngle());
        
        // angular
        double lastAngularVelocity = this.lastVelocity.omegaRadiansPerSecond;
        double desiredAngularVelocity = desiredSpeed.omegaRadiansPerSecond;

        double angularAcceleration = (desiredAngularVelocity - lastAngularVelocity) / dt;
        double normalizedAngularAcceleration = Math.signum(angularAcceleration) * Math.min(Math.abs(angularAcceleration), MAX_ANGULAR_ACCLERATION_RAD_PER_SEC_PER_SEC);

        Translation2d newVelocity = lastLinearVelocity.plus(normalizedAcceleration.times(dt));

        return new ChassisSpeeds(
            newVelocity.getX(), 
            newVelocity.getY(), 
            normalizedAngularAcceleration * dt
        );
    }

    private void updateLineupSpeeds() {
        Translation2d curPos = getPoseMeters().getTranslation();
        LineupPlanner.getInstance().update(curPos);

        LineupPlanner.Position target = LineupPlanner.getInstance().getCurrentTarget();
        // Translation2d disp = curPos.minus(target.point); // temp, used when we don't havve apriltag, 
        Translation2d disp = LineupPlanner.getInstance().getDisplacementFromTarget(curPos);

        double gain;
        // if (disp.getNorm() < tolerance)
        //     gain = 0;
        // if (target.lineupMethod == LineupMethod.ODOM && RobotBase.isReal())
        gain = lineupController.calculate(disp.getNorm(), 
            new TrapezoidProfile.State(0, LineupPlanner.getInstance().getLineupState() == LineupState.EN_ROUTE ? -2 : 0)
        );
        // else
        //     gain = tagAlignmentController.calculate(disp.getNorm(), 0);
        
        Translation2d speeds = new Translation2d(gain, disp.getAngle());

        if (LineupPlanner.getInstance().getLineupState() == LineupState.PULL_IN)
            speeds = speeds.times(0.5);
        
        SmartDashboard.putNumber("displacement", disp.getNorm());

        double angleTarget = Math.PI; // (target.angle.isPresent() ? target.angle.get().getRadians() : Math.PI);
        double turnSpeed = snappingController.calculate(getAngleRotation2d().getRadians(), angleTarget);
        double angleError = snappingController.getPositionError();

        // if (angleError < Math.toRadians(3))
        //     turnSpeed = 0;
        if (angleError < Math.toRadians(6))
            turnSpeed = stabilizationController.calculate(getAngleRotation2d().getRadians(), angleTarget);
        
        boolean allowedToDrive = Math.abs(angleError) < (15.0 * Math.PI/180);
        this.lineupChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            allowedToDrive ? speeds.getX() : 0, 
            allowedToDrive ? speeds.getY() : 0, 
            turnSpeed,
            getAngleRotation2d()
        );
        
        if (disp.getNorm() < 0.0125)
            this.lineupChassisSpeeds = ZERO_CHASSIS_SPEED;
        
        stabilizationHeading = angleTarget;

        SmartDashboard.putNumber("xSpeed", speeds.getX());
        SmartDashboard.putNumber("ySpeed", speeds.getY());
        // SmartDashboard.putNumber("thetaSpeed", rotationOutput);


        // Pose2d curPose = getPoseMeters();
        // if (Math.abs(lineupTargetPose.getX() - curPose.getX()) < MAX_LINEUP_DISTANCE) {
        //     PathPlannerState target = new PathPlannerState();
        //     target.holonomicRotation = lineupTargetPose.getRotation();
        //     target.poseMeters = lineupTargetPose;
        //     // double xSpeed = getPathFollower().getXController().calculate(curPose.getX(), lineupTargetPose.getX());
        //     // double ySpeed = getPathFollower().getYController().calculate(curPose.getY(), lineupTargetPose.getY());
        //     // double thetaSpeed = getPathFollower().getThetaController().calculate(curPose.getRotation().getRadians(), lineupTargetPose.getRotation().getRadians());

        //     lineupChassisSpeeds = getPathFollower().calculate(curPose, target);
        // } else {
        //     lineupChassisSpeeds = ZERO_CHASSIS_SPEED;
        // }
        // // target pose
        // // current pose
        // // pids to 
    }

    private void updateParkingSpeeds() {
        Translation2d curpos = getPoseMeters().getTranslation();
        ParkPlanner.getInstance().update(curpos);

        ParkPlanner.Position target = ParkPlanner.getInstance().getCurrentTarget();
        // Translation2d disp = curPos.minus(target.point); // temp, used when we don't havve apriltag, 
        Translation2d disp = ParkPlanner.getInstance().getDisplacementFromTarget(curpos);

        System.out.println("Park displacement: " + disp);
        double gain;
        // if (disp.getNorm() < tolerance)
        //     gain = 0;
        // if (target.parkingMethod == ParkingMethod.ODOM && RobotBase.isReal())
        //     gain = lineupController.calculate(disp.getNorm(), 0);
        // else
        gain = balancingController.calculate(disp.getNorm(), 0);
        
        Translation2d speeds = new Translation2d(gain, disp.getAngle());
        SmartDashboard.putNumber("displacement", disp.getNorm());
        System.out.println("disp: " + disp.getNorm());
        this.parkingChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.getX(), 
            speeds.getY(), 
            target.angle.isPresent() ? stabilizationController.calculate(getAngleRotation2d().getRadians(), target.angle.get().getRadians()) : Math.PI,
            getAngleRotation2d()
        );


        if (target.angle.isPresent())
            stabilizationHeading = target.angle.get().getRadians();

        SmartDashboard.putNumber("xSpeed", speeds.getX());
        SmartDashboard.putNumber("ySpeed", speeds.getY());


    }

    private void updateFetchingSpeeds() {
        Translation2d curpos = getPoseMeters().getTranslation();
        Translation2d disp = curpos.minus(Jetson.getInstance().getSavedPose().getTranslation());

        double gain;
        // if (disp.getNorm() < tolerance)
        //     gain = 0;
            gain = lineupController.calculate(disp.getNorm(), 0);
        
        Translation2d speeds = new Translation2d(gain, disp.getAngle());
        SmartDashboard.putNumber("displacement", disp.getNorm());

        ChassisSpeeds toFetch = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.getX(), 
            speeds.getY(), 
            0,
            getAngleRotation2d()
        );

        // ChassisSpeeds toFetch = new ChassisSpeeds(speeds.getX(), -speeds.getY(), 0.0);

        // ChassisSpeeds toFetch = PickupPlanner.getInstance().update();

        // if (RobotState.isTeleop()) {
        //     driveWithStabilization(false); // do acceleraiton limiting AFTER adding fetch speeds
        //     toFetch.vxMetersPerSecond += this.teleopChassisSpeeds.vxMetersPerSecond;
        //     toFetch.vyMetersPerSecond += this.teleopChassisSpeeds.vyMetersPerSecond;
        //     toFetch.omegaRadiansPerSecond += this.teleopChassisSpeeds.omegaRadiansPerSecond;   
        // }

        // System.out.println(toFetch);
       this.fetchingChassisSpeeds = (toFetch);
        // if (Jetson.getInstance().getConePose())
    }

    /**
     * Generated ChassisSpeeds objects that are obviously too low will be ignored
     * 
     * @param speeds ChassisSpeeds object to be cleaned
     */
    private void cleanseChassisSpeeds(ChassisSpeeds speeds) {
        speeds.vxMetersPerSecond = Math.abs(speeds.vxMetersPerSecond) < 0.05 ? 0 : speeds.vxMetersPerSecond;
        speeds.vyMetersPerSecond = Math.abs(speeds.vyMetersPerSecond) < 0.05 ? 0 : speeds.vyMetersPerSecond;
        speeds.omegaRadiansPerSecond = Math.abs(speeds.omegaRadiansPerSecond) < Math.toRadians(1) ? 0
                : speeds.omegaRadiansPerSecond;
    }

    // see: https://github.com/Team254/FRC-2022-Public/blob/6a24236b37f0fcb75ceb9d5dec767be58ea903c0/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
    private ChassisSpeeds correctDrift(ChassisSpeeds speeds) {
        Pose2d robot_pose_vel = new Pose2d(speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt,
        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));
        
        Twist2d twist_vel = MathUtils.poseLog(robot_pose_vel);
        speeds.vxMetersPerSecond = twist_vel.dx / dt;
        speeds.vyMetersPerSecond = twist_vel.dy / dt;
        speeds.omegaRadiansPerSecond = twist_vel.dtheta / dt;

        return speeds;
    }

    private void simulationPeriodic() {
        for (SwerveModule module : modules) {
            module.simulationPeriodic();
        }
    }

    public void resetStabalizationHeading() {
        this.stabilizationHeading = getAngleDegrees() * Math.PI / 180.0;
    }


    public void setToDefaultStabilizationHeading() {
        if (stabilizationHeading == (503.503 * Math.PI/180))
            stabilizationHeading = getAngleDegrees() * Math.PI / 180;
    }

    public boolean isDrivingAboveTolerance() {
        return RobotState.isTeleop() && drivingAboveTolerance;
    }

    public void startLineup() {
        this.setControlState(SwerveControlState.LINEUP);
    }

    public void startParking() {
        this.setControlState(SwerveControlState.PARK);
    }

    // public void sendLineup(Translation2d targetPose, Translation2d offset) {
    //     lineupTarget = targetPose;
    //     lineupOffset = offset;
    //     this.setControlState(SwerveControlState.LINEUP);
    // }

    public void disableLineup() {
        this.setControlState(this.previousControlState);
    }

    public void disableParking() {
        this.setControlState(this.previousControlState);
    }

    public void startFetching() {
        this.setControlState(SwerveControlState.FETCHING);
    }

    public void disableFetching() {
        if (this.getControlState() == SwerveControlState.FETCHING){
            this.fetchingChassisSpeeds = ZERO_CHASSIS_SPEED;
            if (RobotState.isTeleop())
                this.setControlState(SwerveControlState.TELEOP);
            else if (RobotState.isAutonomous())
                this.setControlState(SwerveControlState.PATH_FOLLOWING);
            else
                this.setControlState(SwerveControlState.STOPPED);
        }
    }

    public void enableBrakeMode(boolean enable) {
        for (SwerveModule module : modules) {
            module.enableBrakeMode(enable);
        }
        brakeModeEnabled = enable;
    }

    public boolean getBrakeMode() {
        return brakeModeEnabled;
    }

    public void setAngle(double angle) {
        imu.setYaw(angle);
        stabilizationHeading = (angle) * Math.PI / 180;

        if (stabilizationController != null)
            stabilizationController.setSetpoint(stabilizationHeading);

        _simHeading = angle;
    }

    public void setStabilizationP(double P) {
        this.stabilizationController.setP(P);
    }

    public void setStabilizationI(double I) {
        this.stabilizationController.setI(I);
    }

    public double getSnappingGain(Rotation2d setpoint) {
        return this.autoSnappingController.calculate(getAngleRotation2d().getRadians(), setpoint.getRadians());
    }

    public double getSnappingGain() {
        return stabilizationController.calculate(getAngleRotation2d().getRadians(), stabilizationHeading);
    }

    public HashMap<String, Double> getModuleRezeroValues() {
        HashMap<String, Double> rezeroValues = new HashMap<>();

        for (SwerveModule module : modules) {
            rezeroValues.put(module.moduleName, module.getRezeroValue());
        }

        return rezeroValues;
    }

    public double getPitch() {
        double angle = imu.getPitch();
        return angle; // used to be a negative, taken out to account for pigeon increase direction.
    }

    /**
     * Get the current angle of the robot in degrees
     * 
     * @return double (degrees)
     */
    public double getAngleDegrees() {
        double angle = (RobotBase.isReal() ? imu.getYaw() : simPose.getRotation().getDegrees()) % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle; // used to be a negative, taken out to account for pigeon increase direction.
    }

    public double getPitchDegrees() {
        double angle = imu.getPitch();
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    public double getRollDegrees() {
        double angle = imu.getRoll();
        if (angle > 180) {
            angle -= 360;
        } else if (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Get the current angle of the robot as a Rotation2d
     * 
     * @return Rotation2D object
     */
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(getAngleDegrees());
    }

    public void updateOdometry() {
        // for (SwerveModule module : this.modules)
        // module.readPeriodicInputs();

        // double dt = Timer.getFPGATimestamp() - lastTime;
        // Pose2d lastPose = getPoseMeters();
        odometry.update(getAngleRotation2d(), getSwerveModulePositions());
        currentVelocity = RobotBase.isReal() ? kinematics.toChassisSpeeds(frontLeftModule.getSwerveModuleState(), frontRightModule.getSwerveModuleState(), backLeftModule.getSwerveModuleState(), backRightModule.getSwerveModuleState()) : this.simSpeeds;
        
        SmartDashboard.putNumber("actual X spped", currentVelocity.vxMetersPerSecond);
        SmartDashboard.putNumber("actual y spped", currentVelocity.vyMetersPerSecond);
        
        
        if (RobotBase.isSimulation()) {
            simPose = simPose.exp(
                new Twist2d(
                    simSpeeds.vxMetersPerSecond * dt, 
                    simSpeeds.vyMetersPerSecond * dt, 
                    simSpeeds.omegaRadiansPerSecond * dt
                )
            );
        }
        // Pose2d curPose = odometry.getPoseMeters();
        // currentVelocity = new Pose2d((curPose.getX() - lastPose.getX()) / dt, (curPose.getY() - lastPose.getY()) / dt,
        //         getAngleRotation2d().minus(lastTheta).times(1 / dt));

        // lastTime = Timer.getFPGATimestamp();
        // lastTheta = getAngleRotation2d();
    }

    public void acceptVisionMeasurement(Pose2d apriltagRobotPose, double timestamp) {
        this.odometry.addVisionMeasurement(apriltagRobotPose, timestamp);
    }

    public void acceptVisionMeasurement(Translation2d apriltagRobotPose, double timestamp) {
        this.acceptVisionMeasurement(new Pose2d(apriltagRobotPose, getAngleRotation2d()), timestamp);
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose.getRotation(), getSwerveModulePositions(), pose);
        setAngle(pose.getRotation().getDegrees());

        if (RobotBase.isSimulation())
            this.simPose = pose;
    }

    public void setModuleDrivePct(double pct) {
        for (SwerveModule module : modules) {
            module.setDriveMotor(ControlMode.PercentOutput, pct);
        }
    }

    public void setDriveModuleVelocity(double vel) {
        for (SwerveModule module : modules) {
            module.setDriveMotor(ControlMode.Velocity, vel);
        }
    }

    /**
     * Snap all the swerve modules to a specified preset
     */
    public void snapModulesTo(ModuleSnapPositions preset) {
        setControlState(SwerveControlState.SNAPPING);
        int idx = 0;
        for (SwerveModule module : modules) {
            module.setDriveMotor(ControlMode.PercentOutput, 0);
            module.setRotationPosition(Rotation2d.fromDegrees(preset.getSnapPositions()[idx]));
            idx++;
        }
    }

    /**
     * Sets the rotation of ALL modules to the specified angle.
     * It is preferred to use the snapModulesTo method instead.
     */
    public void setModuleRotation(double degrees) {
        setControlState(SwerveControlState.SNAPPING);
        for (SwerveModule module : modules) {
            module.setRotationPosition(Rotation2d.fromDegrees(degrees));
        }
    }

    /**
     * ONLY TO BE CALLED BY THE CONTROL FUNCTION
     * 
     * @param chassisSpeeds
     * @param openLoop isOpenLoop
     */
    private void drive(ChassisSpeeds chassisSpeeds, boolean openLoop) { 
        if (swerveControlState == SwerveControlState.SNAPPING)
            return;

        

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        setSwerveModuleStates(moduleStates, openLoop); 
        
        if (RobotBase.isSimulation())
            this.simSpeeds = chassisSpeeds;
    }

    private void setSwerveModuleStates(SwerveModuleState[] moduleStates, boolean openLoop){
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveModule.MAX_DRIVE_SPEED_METERS_SEC);
        frontLeftModule.setSwerveModuleState(moduleStates[0], swerveControlState.rememberLastPosition, openLoop);
        frontRightModule.setSwerveModuleState(moduleStates[1], swerveControlState.rememberLastPosition, openLoop);
        backLeftModule.setSwerveModuleState(moduleStates[2], swerveControlState.rememberLastPosition, openLoop);
        backRightModule.setSwerveModuleState(moduleStates[3], swerveControlState.rememberLastPosition, openLoop); 
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
       return new SwerveModulePosition[] {frontLeftModule.getSwerveModulePosition(),
        frontRightModule.getSwerveModulePosition(), backLeftModule.getSwerveModulePosition(),
        backRightModule.getSwerveModulePosition()} ;
    }

    public SwerveDriveKinematics getKinematics() {
        return this.kinematics;
    }

    /**
     * Update the speeds that the path follower determines are necessary to follow
     * the path
     * Should be used in Auton mostly unless if we decided to add it into teleop or
     * tests as well.
     * 
     * @param chassisSpeeds The speeds to drive the robot at
     */
    public void setPathFollowerSpeeds(ChassisSpeeds chassisSpeeds) {
        if (RobotState.isAutonomous()) {
            this.pathFollowerSpeeds = chassisSpeeds;        
            this.setControlState(SwerveControlState.PATH_FOLLOWING);
        } else {
            this.pathFollowerSpeeds = ZERO_CHASSIS_SPEED;
            this.setControlState(RobotState.isTeleop() ? SwerveControlState.TELEOP : SwerveControlState.STOPPED);
        }
    }

    // private void drive(double xVelMeters, double yVelMeters, double
    // degreesPerSecond, boolean isFieldRelative) {
    // if (isFieldRelative) {
    // drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVelMeters, yVelMeters,
    // Math.toRadians(degreesPerSecond),
    // getAngleRotation2d()));
    // } else {
    // drive(new ChassisSpeeds(xVelMeters, yVelMeters,
    // Math.toRadians(degreesPerSecond)));
    // }
    // }

    // FIXME: we want to be able to do this with the new auto system too.
    // public void drawPath(Path path) {
    //     ArrayList<Pose2d> poses = new ArrayList<>();
    //     for (Path.State state : path.getStates()) {
    //         poses.add(state.poseMeters);
    //     }
    //     field.getObject("path").setPoses(poses);
    // }

    // public void drawMarkers(List<Marker> markers) {
    //     ArrayList<Pose2d> poses = new ArrayList<>();
    //     for (Marker marker : markers) {
    //         Pose2d upright = new Pose2d(marker.poseMeters.getX(), marker.poseMeters.getY() + marker.radius,
    //                 Rotation2d.fromDegrees(0));
    //         for (int i = 0; i < 360; i++) {
    //             Pose2d circlePoint = upright
    //                     .relativeTo(new Pose2d(marker.poseMeters.getTranslation(), Rotation2d.fromDegrees(i)));
    //             // .plus(new Transform2d(marker.poseMeters.getTranslation(),
    //             // Rotation2d.fromDegrees(0)));
    //             Pose2d circlePointField = new Pose2d(marker.poseMeters.getX() + circlePoint.getX(),
    //                     marker.poseMeters.getY()
    //                             + circlePoint.getY(),
    //                     circlePoint.getRotation());
    //             poses.add(circlePointField);
    //         }
    //         poses.add(marker.poseMeters);
    //     }
    //     field.getObject("markers").setPoses(poses);
    // }

    public void startPlottingRobotPath() {
        this._isPlottingRobotPose = true;
    }

    public void stopPlottingRobotPath() {
        this._isPlottingRobotPose = false;
    }

    private void plotRobotPathPoint() {
        List<Pose2d> temp = field.getObject("robotpath").getPoses();
        temp.add(getPoseMeters());

        field.getObject("robotpath").setPoses(temp);
    }

    public void clearPath() {
        System.out.println("Disabled");
        field.getObject("path").setPoses(Arrays.asList(new Pose2d[] { new Pose2d(0, 0, Rotation2d.fromDegrees(0)) }));
        field.getObject("robotpath")
                .setPoses(Arrays.asList(new Pose2d[] { new Pose2d(0, 0, Rotation2d.fromDegrees(0)) }));
        field.getObject("markers")
                .setPoses(Arrays.asList(new Pose2d[] { new Pose2d(0, 0, Rotation2d.fromDegrees(0)) }));
        this._isPlottingRobotPose = false;
    }

    // public ChassisSpeeds getTrajectoryFollowerOutput(Trajectory.State target,
    // double angle) {
    // return controller.calculate(getPoseMeters(), target,
    // Rotation2d.fromDegrees(angle));
    // }

    public Pose2d getPoseMeters() {
        return RobotBase.isReal() ? new Pose2d(odometry.getEstimatedPosition().getX(), odometry.getEstimatedPosition().getY(), getAngleRotation2d()) : simPose;
    }

    public ChassisSpeeds getVelocity() {
        return this.currentVelocity;
    }

    public Rotation2d getGyroRotationalVelocity() {
        double[] xyz_dps = new double[3];
        imu.getRawGyro(xyz_dps);
        return Rotation2d.fromDegrees(xyz_dps[2]);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Robot X: ", getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Y: ", getPoseMeters().getY());
        SmartDashboard.putNumber("Robot Heading: ", getAngleDegrees());
        SmartDashboard.putNumber("Robot Pitch: ", getPitchDegrees());
        SmartDashboard.putNumber("Robot Roll: ", getRollDegrees());
        SmartDashboard.putString("SWERVE CONTROL STATE: ",
                this.swerveControlState.name());
        SmartDashboard.putBoolean("Is Snapping: ", isSnapping);
        SmartDashboard.putNumber("StablilizationHeading", stabilizationHeading);
        SmartDashboard.putNumber("DELTA TIME", this.dt);

        // SmartDashboard.putNumber("Stabalization Target: ",
        // stabilizationHeading.getDegrees());
        // // SmartDashboard.putNumber("Driver Left Stick X: ",
        // OI.getDriverLeftXValue());
        // // SmartDashboard.putNumber("Driver Left Stick Y: ",
        // OI.getDriverLeftYValue());
        // // SmartDashboard.putNumber("Driver Right Stick X: ",
        // // OI.getDriverRightXValue());

        // SmartDashboard.putNumber("Measured Angular Velocity: ",
        // currentVelocity.getRotation().getDegrees());
        // SmartDashboard.putNumber("Measured X Velocity: ", currentVelocity.getX());
        // SmartDashboard.putNumber("Measured Y Velocity: ", currentVelocity.getX());

        // SmartDashboard.putBoolean("Robot Centric Enabled", isRobotCentric);
        // SmartDashboard.putBoolean("Slow Mode Enabled", slowmodeEnabled);

        // SmartDashboard.putNumber("Fetching chassis speeds X",
        // this.fetchingChassisSpeeds.vxMetersPerSecond);

        for (SwerveModule module : this.modules) {
            module.outputTelemetry();
        }
    }

    public void setSwerveModules(SwerveModule frontLeftSwerveModule, SwerveModule frontRightSwerveModule,
            SwerveModule backLeftSwerveModule, SwerveModule backRightSwerveModule) {
        this.frontLeftModule = frontLeftSwerveModule;
        this.frontRightModule = frontRightSwerveModule;
        this.backLeftModule = backLeftSwerveModule;
        this.backRightModule = backRightSwerveModule;
    }

    public SwervePathFollower getPathFollower() {
        return this.pathFollower;
    }

    public enum ModuleSnapPositions {
        STRAIGHT(0, 0, 0, 0), DEFENSE(45, 315, 315, 45), NINETY(90, 90, 90, 90);

        private final double frontLeft;
        private final double frontRight;
        private final double backLeft;
        private final double backRight;

        ModuleSnapPositions(double fL, double fR, double bL, double bR) {
            this.frontLeft = fL;
            this.frontRight = fR;
            this.backLeft = bL;
            this.backRight = bR;
        }

        public double[] getSnapPositions() {
            return new double[] { frontLeft, frontRight, backLeft, backRight };
        }
    }

    public enum SwerveControlState {
        STOPPED(false), TELEOP(true), PATH_FOLLOWING(true), BACKUP(true), LINEUP(true, LEDLights.RED), PARK(true, LEDLights.TEAL), FETCHING(true, LEDLights.ORANGE), CHARACTERIZATION(true), SNAPPING(false);

        public boolean rememberLastPosition;
        public LEDLights ledColor;

        SwerveControlState(boolean rememberLastPosition, LEDLights ledColor) {
            this.rememberLastPosition = rememberLastPosition;
            this.ledColor = ledColor;
        }  
        
        SwerveControlState(boolean rememberLastPosition) {
            this(rememberLastPosition, null);
        }
    }

    public enum ModuleLocation {
        FrontLeft(0), FrontRight(1), BackLeft(2), BackRight(3), TestStandModule(-1);
        public int index;
        private ModuleLocation(int index) {
            this.index = index;
        }

    }

    @Override
    public void onStart() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub
        for (SwerveModule module : this.modules) {
            module.onStop();
        }

        this.stabilizationHeading = 503.503 * Math.PI / 180;

        this.pathFollowerSpeeds = ZERO_CHASSIS_SPEED;
        this.teleopChassisSpeeds = ZERO_CHASSIS_SPEED;
        this.lineupChassisSpeeds = ZERO_CHASSIS_SPEED;

        this.setControlState(SwerveControlState.STOPPED);
        
    }
}