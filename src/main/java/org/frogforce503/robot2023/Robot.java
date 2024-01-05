// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2023;

import org.frogforce503.robot2023.subsystems.Intake;
// import org.frogforce503.robot2023.subsystems.Pixy;
import org.frogforce503.robot2023.subsystems.DriverFeedback;
import org.frogforce503.robot2023.subsystems.Escalator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;

import java.nio.channels.NetworkChannel;
import java.sql.Driver;

import javax.sound.sampled.Line;

import org.ejml.data.ZMatrix;
import org.frogforce503.lib.auto.AutoChooser;
// import org.frogforce503.lib.auto.OldAutoChooser;
import org.frogforce503.lib.auto.AutonomousExecutor;
import org.frogforce503.lib.util.TaskScheduler;
import org.frogforce503.lib.util.snUtil;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.RobotState.GameState;
import org.frogforce503.robot2023.StateEngine.RobotStates;
import org.frogforce503.robot2023.auto.AutoUtil;
import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.fields.FieldConfig.VENUE;
import org.frogforce503.robot2023.planners.LineupPlanner;
import org.frogforce503.robot2023.planners.ParkPlanner;
//import org.frogforce503.robot2023.planners.ParkPlanner;
import org.frogforce503.robot2023.subsystems.DriverFeedback.LEDLights;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.ModuleSnapPositions;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.SwerveControlState;
import org.frogforce503.robot2023.subsystems.vision.AprilTagHandler;
import org.frogforce503.robot2023.subsystems.vision.cameras.Jetson;
import org.frogforce503.robot2023.subsystems.vision.cameras.Photon;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  boolean autonDone;
  public static RobotHardware bot;
  // private OldAutoChooser autoChooser;
  // private OldAutoChooser autoChooser1;
  private AutoChooser autoChooser;
  private boolean autoHasRun = false;
  private boolean allowAuto = true;
 
  // NetworkTableEntry xTrnslEntry, yTrnslEntry;
  // NetworkTable mainTable;
  private Compressor c;

  snUtil snUtil = new snUtil();
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  double startTime = 0;
  boolean hasGenerated = false;
  
  @Override
  public void robotInit() {
    autonDone= false;
    // mainTable = NetworkTableInstance.getDefault().getTable("jetson").getSubTable("targets").getSubTable("mainTarget");
    // xTrnslEntry = mainTable.getEntry("xTrnsl");
    // yTrnslEntry = mainTable.getEntry("yTrnsl");
    
    RobotState.getInstance().setCurrentRobot(RobotState.Bot.CompBot);
    bot = RobotHardware.getInstance();
    c = new Compressor(PneumaticsModuleType.REVPH);

    Swerve.getInstance().enableBrakeMode(false);
    Swerve.getInstance().initTelemetry();

    RobotState.getInstance().overrideAllianceColor(AllianceColor.BLUE);

    FieldConfig.setVenue(VENUE.KETTERING); // must be done before autchooser initialized
    
    autoChooser = new AutoChooser();
    // autoChooser1 = new OldAutoChooser();

    double startTime = Timer.getFPGATimestamp();
    // autoChooser.buildPaths();
    // autoChooser.addToShuffleboard(Shuffleboard.getTab("DriverStation"));
    AprilTagHandler.detectionMethod = AprilTagHandler.DetectionMethod.PHOTONVISION;
    LineupPlanner.getInstance().initialize();
    ParkPlanner.getInstance().initialize();

    autoChooser.initialize();
    autoChooser.onReset(() -> autoHasRun = false);

    AprilTagHandler.getInstance().initializeFieldLayout(AllianceColor.BLUE);
    
    Swerve.getInstance().setAngle(0);

    startTime = Timer.getFPGATimestamp();

    Photon.getInstance().initBackCamera();
    Swerve.getInstance();

    DataLogManager.start();
    SmartDashboard.putString("LOG DIRECTORY", DataLogManager.getLogDir());
   
    // Intake.getInstance();
    // Escalator.getInstance();
    // PixyTest.getInstance();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */


  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Timer", Timer.getMatchTime());
  
    SmartDashboard.putNumber("intake left current", Intake.getInstance().bottomIntakeRoller.getOutputCurrent());
    SmartDashboard.putNumber("intake right current", Intake.getInstance().topIntakeRoller.getOutputCurrent());
    SmartDashboard.putNumber("Time of Flight", Intake.getInstance().getToFRange());
      //  Jetson.getInstance().getConePose();  
      // Jetson.getInstance().getPoseToTarget();
      // SmartDashboard.putString("PIxy cam", Pixy.getInstance().getObject()+"");
      SmartDashboard.putBoolean("Zero Switch",  Escalator.getInstance().zeroSwitch.get());
      // SmartDashboard.putBoolean("Claw open", Escalator.getInstance().claw.get() )
      Intake.getInstance().outputTelemetry();
      SmartDashboard.putString("Intake State", Intake.getInstance().getState()+"");

      SmartDashboard.putNumber("Extender Absolute Position", Intake.getInstance().getAbsoluteExtensionAngle());
      SmartDashboard.putNumber("Extender Internal Position", Intake.getInstance().getExtensionAngle());

      SmartDashboard.putNumber("Escalator internal postion", Escalator.getInstance().getCurPos());
      SmartDashboard.putNumber("Escalator position cancoder", Escalator.getInstance().getCancoderPos());

      SmartDashboard.putNumber("Belt Motor Temp", Intake.getInstance().beltMotor.getMotorTemperature());
      SmartDashboard.putNumber("Suction Wheel Temp", Intake.getInstance().suctionWheels.getMotorTemperature());
      SmartDashboard.putNumber("Left Intake Wheel Temp", Intake.getInstance().bottomIntakeRoller.getMotorTemperature());
      SmartDashboard.putNumber("Extender Temp", Intake.getInstance().extender.getMotorTemperature());

      if (Photon.getInstance().getEstimatedRobotPose() != null) {
        SmartDashboard.putNumber("photon pose X ", Photon.getInstance().getEstimatedRobotPose().getX());
        SmartDashboard.putNumber("photon pose Y", Photon.getInstance().getEstimatedRobotPose().getY());
      }

      if (LineupPlanner.getInstance().getCurrentTarget() != null) {
        SmartDashboard.putNumber("photon line up pose X ", LineupPlanner.getInstance().getCurrentTarget().getPoint().getX());
        SmartDashboard.putNumber("photon line up pose Y ", LineupPlanner.getInstance().getCurrentTarget().getPoint().getY());
      }
  }
  

  public static double startAutoTime = 0;

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

  public static double timeSinceAutoStart() {
    return Timer.getFPGATimestamp() - Robot.startAutoTime;
  }


  @Override
  public void autonomousInit() {

    startAutoTime = Timer.getFPGATimestamp();

    RobotState.getInstance().setGameState(RobotState.GameState.AUTON);
    
    // initialize state engine
    Intake.getInstance().onStart();
    Escalator.getInstance().onStart();

    // System.out.println("Checkpoint 1: " + timeSinceAutoStart());

    //AutonomousExecutor.getInstance().setSelectedAutoMode(new RightJuggle3()); // TODO: REMOVE THIS FOR SAFETY

    if (!autoHasRun) 
      AutonomousExecutor.getInstance().startAuton();      
    else 
      System.out.println("RESTART ROBOT CODE BEFORE RERUNNING AUTO");

    
    // System.out.println("Checkpoint 2: " + timeSinceAutoStart());
    
    this.allowAuto = !this.autoHasRun;
    this.autoHasRun = true;

    c.enableAnalog(110,120);


    // // System.out.println("Checkpoint 2: " + timeSinceAutoStart());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // // System.out.println("Checkpoint 4: " + timeSinceAutoStart());

    if (this.allowAuto)
      AutonomousExecutor.getInstance().periodic();

    // // System.out.println("Checkpoint 5: " + timeSinceAutoStart());

    StateEngine.getInstance().updateState();

    // // System.out.println("Checkpoint 6: " + timeSinceAutoStart());

    Swerve.getInstance().onLoop();

    // // System.out.println("Checkpoint 7: " + timeSinceAutoStart());
    Swerve.getInstance().outputTelemetry();

    // // System.out.println("Checkpoint 8: " + timeSinceAutoStart());

    // if (AutoUtil.isOnCable)
    //   AprilTagHandler.getInstance().onLoop();
    
    // if (AutoUtil.isOnCable /* || AutoUtil.useFrontOnly*/) {
    //   AprilTagHandler.getInstance().onLoop();
    //   AprilTagHandler.getInstance().outputTelemetry();
    // }


    // // System.out.println("Checkpoint 9: " + timeSinceAutoStart());
    // AprilTagHandler.getInstance().outputTelemetry();


    // // System.out.println("Checkpoint 10: " + timeSinceAutoStart());
    if(Timer.getMatchTime() < 1){
      autonDone = true;
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Intake.getInstance().onStart();
    // DriverFeedback.getInstance().setColor(LEDLights.BLUE);
    OI.flushOICache();
    RobotState.getInstance().setGameState(GameState.TELEOP);

    Swerve.getInstance().setControlState(SwerveControlState.TELEOP);
    Swerve.getInstance().snapModulesTo(ModuleSnapPositions.STRAIGHT);
    Swerve.getInstance().enableBrakeMode(true); // when input is 0, motors will forceably stop
    Swerve.getInstance().setToDefaultStabilizationHeading(); //
    // StateEngine.getInstance().setRobotState(RobotStates.IDLE);
    // Escalator.getInstance().onStart();

    if(StateEngine.getInstance().getRobotState() != RobotStates.ZERO){
      StateEngine.getInstance().setRobotState(RobotStates.IDLE);
    }

    Intake.getInstance().updateDriverFeebackLEDs();

    c.enableAnalog(100, 120);
  
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Pressure", c.getPressure());

    OI.buttonCheck();
    
    Swerve.getInstance().onLoop();
    Swerve.getInstance().outputTelemetry();

    AprilTagHandler.getInstance().onLoop();
    AprilTagHandler.getInstance().outputTelemetry();
    StateEngine.getInstance().updateState();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    RobotState.getInstance().setGameState(GameState.DISABLED);
    Swerve.getInstance().onStop();
    
    DriverFeedback.getInstance().setOff();
    DriverFeedback.getInstance().setColor(LEDLights.RAINBOW);
    AutonomousExecutor.getInstance().stop();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // if(autonDone){
    //   DriverFeedback.getInstance().setColor(LEDLights.RAINBOW);
  
    // }
    autoChooser.periodic();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    c.enableAnalog(110, 120);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Pressure", c.getPressure());
  }

  /** This function is called once when the robot is fir st started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
