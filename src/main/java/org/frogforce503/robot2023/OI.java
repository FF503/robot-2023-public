package org.frogforce503.robot2023;


import org.frogforce503.robot2023.subsystems.Escalator;
import org.frogforce503.robot2023.subsystems.Intake;
import org.frogforce503.robot2023.subsystems.Escalator.EscalatorStates;
import org.frogforce503.robot2023.subsystems.Intake.IntakeStates;
import org.frogforce503.robot2023.RobotState.GamePiece;
import org.frogforce503.robot2023.StateEngine.RobotStates;

import java.sql.Driver;
import java.util.HashMap;
import java.util.Map;

import javax.print.attribute.standard.Fidelity;
//import javax.xml.catalog.GroupEntry.PreferType;

import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.planners.LineupPlanner;
import org.frogforce503.robot2023.planners.ParkPlanner;
import org.frogforce503.robot2023.planners.LineupPlanner.ScoringHeight;
import org.frogforce503.robot2023.planners.LineupPlanner.Position.LineupMethod;
import org.frogforce503.robot2023.subsystems.DriverFeedback;
import org.frogforce503.robot2023.subsystems.DriverFeedback.LEDLights;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.ModuleSnapPositions;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.SwerveControlState;
import org.frogforce503.robot2023.subsystems.vision.AprilTagHandler;
import org.frogforce503.robot2023.subsystems.vision.cameras.Jetson;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

@SuppressWarnings("unused")
public class OI {
    static GenericHID driver = new GenericHID(0);
    static GenericHID buttonBoard = new GenericHID(1);
    static GenericHID overrideController = new GenericHID(2);


    // static DoubleSolenoid intakeShifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    // static DoubleSolenoid claw3 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 5);
    // static DoubleSolenoid claw4 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 2);
    // static DoubleSolenoid wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 6);

    static final int buttonA = 1;
    static final int buttonB = 2;
    static final int buttonX = 3;
    static final int buttonY = 4;
    static final int buttonLB = 5;
    static final int buttonRB = 6;
    static final int buttonSelect = 7;
    static final int buttonMenu = 8;
    static final int buttonLeftJoystick = 9;
    static final int buttonRightJoystick = 10;
    
    //button box
    static final int bbGridLeft = 11;
    static final int bbGridCenter = 10;
    static final int bbGridRight = 5;
    static final int bbCellLeft = 6;
    static final int bbCellCenter = 9;
    static final int bbCellRight = 4;
    static final int bbHigh = 3;
    static final int bbMid = 2;
    static final int bbLow = 1;
    static final int bbSignalCone = 8;
    static final int bbSignalCube = 12;
    static final int bbScoreButton = 7;

    static int grid = 3;
    static int cell = 0;
    static boolean high = false;

    static int targetPos = 0;
    
    static boolean driverRTWasPressed = false;
    static boolean overrideRTWasPressed = false;
    static boolean driverLTWasPressed = false;

    public static void buttonCheck(){
        // Runnable agitate = () -> StateEngine.getInstance().setRobotState(RobotStates.AGITATE);
        // Runnable grab = () -> StateEngine.getInstance().setRobotState(RobotStates.GRAB);

        // whenPressedHeldAndReleased(overrideController, buttonX, agitate, agitate, grab);

        whileHeldAndReleased(overrideController, buttonA, ()-> StateEngine.getInstance().setRobotState(RobotStates.REINDEX), ()-> StateEngine.getInstance().setRobotState(RobotStates.IDLE));
        whileHeldAndReleased(overrideController, buttonLB, () -> StateEngine.getInstance().setRobotState(RobotStates.MANUAL_SCORE), ()-> StateEngine.getInstance().setRobotState(RobotStates.ZERO)); // TODO Change controls back
        whileHeldAndReleased(driver, buttonX, ()-> StateEngine.getInstance().setRobotState(RobotStates.EJECT_THROUGH_INTAKE), ()-> StateEngine.getInstance().setRobotState(RobotStates.IDLE));
        // ------ GAME SPEC ------
        // intake
        whenTriggerPressedHeldAndReleased(getDriverLeftTrigger(), driverLTWasPressed, () -> {
            StateEngine.getInstance().setRobotState(RobotStates.INTAKING);
            Intake.getInstance().intakeLog("BUTTON PRESSED");
        }, () -> {}, ()-> {
            StateEngine.getInstance().setRobotState(Intake.getInstance().getDesiredGamePiece() == GamePiece.CUBE ? RobotStates.INDEXING : RobotStates.IDLE);
            
            Intake.getInstance().updateDriverFeebackLEDs();
        });

        // Extender
       
        // whenPressed(overrideController, buttonB, () -> Intake.getInstance().setIntakeState(IntakeStates.STOWED));

        
        // zero escalator
        whenPressed(overrideController, buttonY, ()->StateEngine.getInstance().setRobotState(RobotStates.ZERO));
        
        // true agitate jiggle
        // whileHeldAndReleased(overrideController, buttonX, ()-> {
        //     Intake.getInstance().resetAgitateSign();
        //     StateEngine.getInstance().setRobotState(RobotStates.AGITATE);
        // }, ()-> StateEngine.getInstance().setRobotState(RobotStates.GRAB));

        // whenPressed(driver, buttonX, () -> Swerve.getInstance().setControlState(SwerveControlState.PARK));

        // manual claw control 
        // whenPressed(driver, buttonRB, () ->Escalator.getInstance().claw.set(Value.kForward));
        // whenPressedAndReleased(overrideController, buttonLB, ()->Escalator.getInstance().setEscalatorState(EscalatorStates.SPIT_CONE),()->Escalator.getInstance().setEscalatorState(EscalatorStates.ZEROING));
        
        // start scoring sequence (will not drop)
        // whenPressed(driver, buttonSelect, () -> { // should become buttonBoard, bbScoreButton
        //     StateEngine.getInstance().setRobotState(RobotStates.PRE_DROP);
        // });

        whenPressedHeldAndReleased(driver, buttonA, 
            () -> {
                if(LineupPlanner.getInstance().getSelectedScoringHeight() == ScoringHeight.LOW && Intake.getInstance().getDesiredGamePiece() == GamePiece.CUBE){
                    StateEngine.getInstance().setRobotState(RobotStates.CUBE_LOW_SCORE);
                }else{
                StateEngine.getInstance().setRobotState(RobotStates.PRE_DROP);
                }
            },
            () -> {}, // dont do anything new while held
            StateEngine.getInstance()::allowDrop
        );

        whenPressedAndReleased(driver, buttonRB, Swerve.getInstance()::startBackup, Swerve.getInstance()::disableBackup);
        
        whileHeldAndReleased(buttonBoard, bbScoreButton, ()-> StateEngine.getInstance().setRobotState(RobotStates.REINDEX), ()-> StateEngine.getInstance().setRobotState(RobotStates.IDLE));


        // whenTriggerPressedHeldAndReleased(getOverrideRightTrigger(), overrideRTWasPressed, 
        //     () -> StateEngine.getInstance().setRobotState(RobotStates.PRE_DROP),
        //     () -> {}, // dont do anything new while held
        //     StateEngine.getInstance()::allowDrop
        // );

        // ---- BUTTON BOARD ----
        // whenPressed(buttonBoard, bbGridLeft, () -> LineupPlanner.getInstance().selectGrid(3));
        // whenPressed(buttonBoard, bbGridCenter, () -> LineupPlanner.getInstance().selectGrid(2));
        // whenPressed(buttonBoard, bbGridRight, () -> LineupPlanner.getInstance().selectGrid(1));

        // whenPressed(buttonBoard, bbCellLeft, () -> LineupPlanner.getInstance().selectCell(3));
        // whenPressed(buttonBoard, bbCellCenter, () -> LineupPlanner.getInstance().selectCell(2));
        // whenPressed(buttonBoard, bbCellRight, () -> LineupPlanner.getInstance().selectCell(1));

        whenPressed(buttonBoard, bbGridLeft, () -> DriverFeedback.getInstance().setColorCone(LEDLights.RED));
        whenPressed(buttonBoard, bbGridCenter, () -> DriverFeedback.getInstance().setColorCone(LEDLights.ORANGE));
        whenPressed(buttonBoard, bbGridRight, () -> DriverFeedback.getInstance().setColorCone(LEDLights.YELLOW));

        whenPressed(buttonBoard, bbCellLeft, () -> DriverFeedback.getInstance().setColorCone(LEDLights.GREEN));
        whenPressed(buttonBoard, bbCellCenter, () -> DriverFeedback.getInstance().setColorCone(LEDLights.TEAL));
        whenPressed(buttonBoard, bbCellRight, () -> DriverFeedback.getInstance().setColorCone(LEDLights.PURPLE_CONE));


        whenPressed(buttonBoard, bbHigh, () -> LineupPlanner.getInstance().selectHeight(ScoringHeight.HIGH));
        whenPressed(buttonBoard, bbMid, () -> LineupPlanner.getInstance().selectHeight(ScoringHeight.MID));
        whenPressed(buttonBoard, bbLow, () -> LineupPlanner.getInstance().selectHeight(ScoringHeight.LOW));

        whenPressed(buttonBoard, bbSignalCone, () -> Intake.getInstance().setDesiredGamePiece(GamePiece.CONE));
        whenPressed(buttonBoard, bbSignalCube, () -> Intake.getInstance().setDesiredGamePiece(GamePiece.CUBE));

        // --- SWERVE ----

        // zero gyro
        whenPressed(driver, buttonB, () -> Swerve.getInstance().setAngle(0)); // put back

        whenPressed(driver, buttonSelect, Swerve.getInstance()::toggleSlowMode);
        whenPressed(driver, buttonMenu, Swerve.getInstance()::toggleRobotCentric);

        // Swerve.getInstance().checkSlowMode(driver.getRawButton(buttonRB));
        // Swerve.getInstance().checkRobotCentric(driver.getRawButton(buttonA));

        // Jetson autopickup
        // whenPressedHeldAndReleased(driver, buttonLB,
        //     Jetson.getInstance()::savePose, 
        //     () -> Swerve.getInstance().setControlState(SwerveControlState.FETCHING), 
        //     () -> Swerve.getInstance().disableFetching()
        // );

        // whenPressedHeldAndReleased(driver, buttonA, 
        //     Jetson.getInstance()::savePoseFromPreference, 
        //     () -> Swerve.getInstance().setControlState(SwerveControlState.FETCHING), 
        //     () -> Swerve.getInstance().disableFetching()
        // );
        // whenPressedHeldAndReleased(driver, buttonLB, ()-> Jetson.getInstance().savePose(true), () -> Swerve.getInstance().setControlState(SwerveControlState.FETCHING), () -> Swerve.getInstance().disableFetching());
        // whenPressedHeldAndReleased(driver, buttonRB, ()-> Jetson.getInstance().savePose(false), () -> Swerve.getInstance().setControlState(SwerveControlState.FETCHING), () -> Swerve.getInstance().disableFetching());

        // whenPressed(driver, buttonX, Swerve.getInstance()::snapToLoad);
        
        // Automagic align
        whenTriggerPressedHeldAndReleased(getDriverRightTrigger(), driverRTWasPressed, () -> { },
            () -> LineupPlanner.getInstance().alignToSelected(), 
            () -> LineupPlanner.getInstance().stopTrying()
        );

        driverLTWasPressed = getDriverLeftTrigger();
        driverRTWasPressed = getDriverRightTrigger();
        overrideRTWasPressed = getOverrideRightTrigger();
    }

    public static void whileHeld(GenericHID controller, int button, Runnable whileHeld){
        if(controller.getRawButton(button)){
          whileHeld.run();
        }
    }

    public static void whenPressed(GenericHID controller, int button, Runnable onPress){
        if(controller.getRawButtonPressed(button)){
            onPress.run();
        }
    }

    public static void whenReleased(GenericHID controller, int button, Runnable onRelease){
        if(controller.getRawButtonReleased(button)){
            onRelease.run();
        }
    }

    public static void whenTriggerPressed(boolean current, boolean last, Runnable onPress) {
        if (!last && current) {
            onPress.run();
        }
    }

    public static void whenTriggerReleased(boolean current, boolean last, Runnable onRelease) {
        if (last && !current) {
            onRelease.run();
        }
    }

    public static void whileTriggerHeld(boolean current,  Runnable whileHeld) {
        if (current) {
            whileHeld.run();
        }
    }

    public static void whenPressedAndReleased(GenericHID controller, int button, Runnable onPress, Runnable whenReleased){
        whenPressed(controller, button, onPress);
        whenReleased(controller, button, whenReleased);
    }
    

    public static void whenPressedHeldAndReleased(GenericHID controller, int button, Runnable onPress, Runnable whileHeld, Runnable whenReleased){
        whenPressed(controller, button, onPress);
        whileHeld(controller, button, whileHeld);
        whenReleased(controller, button, whenReleased);
    }

    public static void whenTriggerPressedAndReleased(boolean current, boolean last, Runnable whenPressed, Runnable whenRelesaed) {
        whenTriggerPressed(current, last, whenPressed);
        whenTriggerReleased(current, last, whenRelesaed);
    }


    public static void whenTriggerPressedHeldAndReleased(boolean current, boolean last, Runnable whenPressed, Runnable whileHeld, Runnable whenRelesaed) {
        whenTriggerPressed(current, last, whenPressed);
        whileTriggerHeld(current, whileHeld);
        whenTriggerReleased(current, last, whenRelesaed);
    }

    public static void whileHeldAndReleased(GenericHID controller, int button, Runnable whileHeld, Runnable whenReleased){
        whileHeld(controller, button, whileHeld);
        whenReleased(controller, button, whenReleased);
    }

    public static boolean getButton(GenericHID controller, int button){
        return controller.getRawButton(button);
    }

    public static double getDriverLeftYValue() {
        return driver.getRawAxis(1);
    }

    public static double getDriverLeftXValue() {
        return driver.getRawAxis(0);
    }

    public static double getDriverRightYValue() {
        return driver.getRawAxis(5);
    }

    public static double getDriverRightXValue() {
        return driver.getRawAxis(4);
    }

    public static boolean getDriverLeftTrigger() {
        return driver.getRawAxis(2) >= 0.2;
    }

    public static boolean getDriverRightTrigger() {
        return driver.getRawAxis(3) >= 0.2;
    }

    public static boolean getOverrideRightTrigger() {
        return overrideController.getRawAxis(3) >= 0.5;
    }

    public static double getOperatorLeftYValue() {
        return overrideController.getRawAxis(1);
    }

    public static double getOperatorLeftXValue() {
        return overrideController.getRawAxis(0);
    }

    public static boolean tryingToAutoAlign() {
        return getDriverRightTrigger();
    }

    public static boolean tryingToDrive() {
        return (Math.abs(Math.hypot(getDriverLeftXValue(), getDriverLeftYValue())) >= Swerve.JOYSTICK_DRIVE_TOLERANCE)
                || (Math.abs(getDriverRightXValue()) > Swerve.JOYSTICK_TURN_TOLERANCE);
    }

    public static void flushOICache(){
        for(int i = 0; i <= 2;i++){
            GenericHID controller = new GenericHID(i);
            for(int j = 1; j <= 12; j++){
                controller.getRawButton(j);
            }
        }
    }

}
