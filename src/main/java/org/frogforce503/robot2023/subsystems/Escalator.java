package org.frogforce503.robot2023.subsystems;

import java.sql.Driver;
import java.util.Map;

import org.frogforce503.lib.auto.actions.base.WaitUntil;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.TalonSRXWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.lib.math.MathUtils;
import org.frogforce503.lib.util.TaskScheduler;
import org.frogforce503.lib.util.snUtil;
import org.frogforce503.robot2023.OI;
import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.StateEngine;
import org.frogforce503.robot2023.RobotState.GamePiece;
import org.frogforce503.robot2023.StateEngine.RobotStates;
import org.frogforce503.robot2023.planners.LineupPlanner.ScoringHeight;
import org.frogforce503.robot2023.subsystems.DriverFeedback.LEDLights;
import org.frogforce503.robot2023.subsystems.Intake.IntakeStates;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Escalator extends Subsystem {
    private static Escalator instance = null;

    public final int MIDDLE_CONE = 66; //46;// 66;
  //  private final int HIGH_CONE = 58;

    public snUtil snUtil = new snUtil();

    boolean alreadyRun = false;
    boolean hasGrabbed = false;

    private final double EXTEND_TIMEOUT = 3.0;

    private double inchesToDegrees =  ((1/1.003) * (1/Math.PI) * 360); 

    private double conversionFactor = 4.724955 *(1/3.65) * (65.0 / 67.0);

    private CANSparkMaxWrapper mEscalator;
    private CANCoder escalatorEncoder;
    private PIDController extendController;
    private PIDController coneIntakeController;
    private TalonSRXWrapper wristIntake;
    public DoubleSolenoid claw;
    public DoubleSolenoid wrist;

    public DigitalInput zeroSwitch;

    /** Value in inches */
    private double escalatorSetpoint = 0;
    private double desiredScoringHeight = 0;

    private double startExtendTime = -1;

    /** Value in inches */
    public double posTolerance = 0.25; //for internal: 0.25
   
    public enum EscalatorStates {
        OFF, MANUAL, ZEROING,
        GRAB_LOW,
        EXTEND, EJECT, CLEARANCE_FOR_INTAKING, HOLD, REINDEX, GRAB_HIGH, INTAKE_CONE, HOLD_CONE, RETRACT_WITH_CONE, SPIT_GAME_PIECE, INTAKE_CUBE, HOLD_CUBE, SCORE_CUBE_LOW
    }

    public EscalatorStates curState = EscalatorStates.OFF;

    public static Escalator getInstance() {
        if (instance == null) { instance = new Escalator(); }
        return instance;
    }

    public Escalator() {

        mEscalator = new CANSparkMaxWrapper(Robot.bot.escalatorID, MotorType.kBrushless);
      
        mEscalator.restoreFactoryDefaults();

       // claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, Robot.bot.intakeLockForwardID, Robot.bot.intakeLockReverseID);
    //    //TODO check solenoid locations/length
        wrist = new DoubleSolenoid(PneumaticsModuleType.REVPH, Robot.bot.wristShiftForwardID, Robot.bot.wristShiftReverseID);

        mEscalator.setP(0, Robot.bot.escalatorP);
        mEscalator.setI(0, Robot.bot.escalatorI);
        mEscalator.setD(0, Robot.bot.escalatorD);
        mEscalator.setFF(0, Robot.bot.escalatorFF);  

        mEscalator.setP(1, Robot.bot.escalatorConeP);
        mEscalator.setI(1, Robot.bot.escalatorConeI);
        mEscalator.setD(1, Robot.bot.escalatorConeD);
        mEscalator.setFF(1, Robot.bot.escalatorConeFF);

        escalatorEncoder = new CANCoder(Robot.bot.escalatorEncoderID);
        escalatorEncoder.configFactoryDefault();
        escalatorEncoder.configSensorDirection(true);
        
        escalatorEncoder.configSensorDirection(true, 0);
        SmartDashboard.putBoolean("Had to change zero", false);
        escalatorEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        
      //  escalatorEncoder.setPosition(escalatorEncoder.getPosition());
        

        extendController = new PIDController(Robot.bot.escalatorP, Robot.bot.escalatorI, Robot.bot.escalatorD);
        extendController.setTolerance(posTolerance);
    


        // mEscalator.setIzone(0, Robot.bot.escalatorIzone);
     
        resetEscalatorEncoder();
        mEscalator.setIdleMode(IdleMode.kBrake);

        mEscalator.setPositionConversionFactor(conversionFactor);

        zeroSwitch = new DigitalInput(Robot.bot.escalatorZeroID);

        snUtil.initLayout("GameSpec", "Escalator", BuiltInLayouts.kList, 0, 0, 3, 5);
        snUtil.initLayout("GameSpec", "Escalator PIDs", BuiltInLayouts.kList, 3, 0, 3, 5);
        snUtil.initLayout("GameSpec", "Escalator Telemetry", BuiltInLayouts.kList, 6, 0, 3, 5);

        mEscalator.burnFlash();

        wristIntake = new TalonSRXWrapper(2);
        wristIntake.configFactoryDefault();
        wristIntake.setNeutralMode(NeutralMode.Coast);
        wristIntake.configOpenloopRamp(0.50);
    }

    public void initTelemetry() {
        snUtil.add("Escalator", "Cur pos", 0, 1, 1, BuiltInWidgets.kGraph, Map.of("min", -1000, "max", 1000));
        snUtil.add("Escalator", "Setpoint", 0, 1, 1);
        snUtil.add("Escalator", "Pos err", 0, 1, 1);

        snUtil.add("Escalator PIDs", "P", 0, 1, 1, BuiltInWidgets.kNumberSlider, Map.of("min", 0, "max", 3));
        snUtil.add("Escalator PIDs", "I", 0, 1, 1, BuiltInWidgets.kNumberSlider, Map.of("min", 0, "max", 3));
        snUtil.add("Escalator PIDs", "D", 0, 1, 1, BuiltInWidgets.kNumberSlider, Map.of("min", 0, "max", 3));

        snUtil.add("Escalator Telemetry", "Escalator Telemetry Toggle", outputtingTelemetry(), 2, 2, BuiltInWidgets.kToggleSwitch);
        snUtil.add("Escalator Telemetry", "Escalator Telemetry Toggled", outputtingTelemetry(), 2, 2, BuiltInWidgets.kBooleanBox);
    }

    public void outputTelemetry() {
        snUtil.set("Escalator Telemetry", "Escalator Telemetry Toggle", outputtingTelemetry());
        snUtil.set("Escalator Telemetry", "Escalator Telemetry Toggled", outputtingTelemetry());

        if (outputtingTelemetry()) {
            snUtil.set("Escalator", "Setpoint", escalatorSetpoint);
            snUtil.set("Escalator", "Cur pos", getCurPos());
            snUtil.set("Escalator", "Pos err", (escalatorSetpoint - getCurPos()));
        }
    }

    private boolean outputtingTelemetry() {
        return snUtil.get("Escalator Telemetry", "Escalator Telemetry Toggle", false);
    }

    public EscalatorStates getState() {
        return curState;
    }

    public void setEscalatorState(EscalatorStates state) {
        if (state != curState) {
            DriverFeedback.getInstance().getHUDTable()
                .getEntry("escalatorState")
                .setString(curState.name());
        }

        curState = state;
    }

    // public void toggleClaw(){
    //     if(curClawState == ClawStates.GRAB){
    //         setClawState(ClawStates.RELEASE);
    //     }else{
    //         setClawState(ClawStates.GRAB);
    //     }
    // }
    // public void setClawState(ClawStates state) {
       

    //     switch(curClawState) {
    //         case GRAB:
    //             clawRight.set(Value.kForward);
    //             break;
    //         case RELEASE:
    //             clawRight.set(Value.kReverse);
    //             break;
    //         default:
    //             break; 
    //     }
    
    // }?\

    public void setWristShifter(Value value) {
        if (value != wrist.get()) {
            wrist.set(value);
        }
    }

    public void clawOpen(boolean t){
        claw.set((t) ? Value.kForward : Value.kReverse);
    }

    private void updatePIDs() {
        mEscalator.setP(0, snUtil.get("Escalator PIDs", "P", 0));
        mEscalator.setI(0, snUtil.get("Escalator PIDs", "I", 0));
        mEscalator.setD(0, snUtil.get("Escalator PIDs", "D", 0));
    }

    @Override
    public void onStart() {
        System.out.println("ON START");
        setEscalatorState(EscalatorStates.OFF);
        setDesiredScoringHeight(ScoringHeight.MID);
        // claw.set(Value.kReverse);
    }
    
    @Override
    public void onLoop() {
        SmartDashboard.putString("Escalator State", curState.name());
        SmartDashboard.putNumber("Setpoint", getEscalatorSetpoint());
        SmartDashboard.putNumber("Escalator Desired Scoring Height", desiredScoringHeight);
        SmartDashboard.putNumber("Escalator Wrist Threshold", (desiredScoringHeight * (RobotState.isAutonomous() ? 0.40 : 0.60)));
        SmartDashboard.putNumber("Escalator internal Position", mEscalator.getEncoderPosition());
        SmartDashboard.putNumber("Escalator Velocity", mEscalator.getEncoderVelocity());
        SmartDashboard.putNumber("Escalator Percent", mEscalator.getAppliedOutput());
        SmartDashboard.putNumber("Escalator Intake current", wristIntake.getStatorCurrent());

        switch(curState) {
            case OFF:
                TaskScheduler.getInstance().allow("clearance_drop");
                alreadyRun = false;
                stopEscalator();
                 //etClawState(ClawStates.RELEASE);
                setWristShifter(Value.kReverse);
                wristIntake.set(TalonSRXControlMode.PercentOutput, 0.0);
                
                break;

            // case MANUAL:
            //     setEscalatorPosToSetpoint(MathUtils.clamp(OI.getOperatorLeftYValue(), 0, 1)
            //                                 * Robot.bot.HIGH_SCORING_HEIGHT);

            //     break;

            case ZEROING:
                wristIntake.set(TalonSRXControlMode.PercentOutput, 0.0);
                TaskScheduler.getInstance().allow("clearance_drop");
                alreadyRun = false;
                double escalatorPower = getCancoderPos() > 9.0 ? -0.8 : -0.10;

                if (!isEscalatorZeroed()) {
                    mEscalator.set(ControlMode.PercentOutput, escalatorPower);
                
                    
                        setWristShifter(Value.kReverse);
                    
                  
                } else {
                    stopEscalator();
                    resetEscalatorEncoder();
                }
                break;

            case CLEARANCE_FOR_INTAKING:
                alreadyRun = false;
                hasGrabbed = false;
                clawOpen(true);

                double cheight = Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE ? 6.0 : 10.5;
                
                // TaskScheduler.getInstance().schedule("clearance_drop", ()->{
                // if(Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE){
                //     setEscalatorSetpoint(6);
                // }else{
                //     setEscalatorSetpoint(10.5);
                // }

                setEscalatorPos(cheight);

                if (inPosition()) {
                    Escalator.getInstance().setEscalatorState(EscalatorStates.HOLD);
                } else {
                    setEscalatorSetpoint(cheight);
                    setWristShifter(Value.kReverse);
                    setEscalatorPosToSetpoint(getEscalatorSetpoint());
                }
                // }, () -> , 0.25);
              
                break;

            case INTAKE_CONE:
                extendController.setP(0.13);
                extendController.setD(0.0);
                extendController.setI(0.0);
              
                wrist.set(Value.kReverse);
                setEscalatorSetpoint(6.0); //TODO 3.75 for Cancoder
                wristIntake.set(TalonSRXControlMode.PercentOutput, -0.8);
               

                if(inPosition() || getCancoderPos() > 3.75){
                    mEscalator.set(ControlMode.PercentOutput, 0.05);
                }

                if(Math.abs(wristIntake.getStatorCurrent()) < 12){ // 6
                    wristIntake.set(TalonSRXControlMode.PercentOutput, -0.8);
                }
                else{
                    setEscalatorState(EscalatorStates.HOLD_CONE);
                }
                   
                  //  wrist.set(null);
                // }

                setEscalatorPosToSetpoint(getEscalatorSetpoint());
                break;

            case INTAKE_CUBE:
                setWristShifter(Value.kReverse);
                // force elevator down
                double retractPower = getCurPos() > 9.0 ? -0.4 : -0.1;
                if (!isEscalatorZeroed()) {
                    mEscalator.set(ControlMode.PercentOutput, retractPower);
                  //  setWristShifter(Value.kReverse);
                } else {
                    stopEscalator();
                    resetEscalatorEncoder();

                    
                    if(Intake.getInstance().getToFRange() > 170){
                        wristIntake.set(TalonSRXControlMode.PercentOutput, 0.8);

                        // if (Intake.getInstance().isClearForPneumatic()) {
                        //   //  setWristShifter(Value.kForward);
                        // } else {
                        //     setWristShifter(Value.kReverse);  
                        // }
                    } else{
                        setEscalatorState(EscalatorStates.HOLD_CUBE);
                    }
                }
            // }
                
                break;

            case REINDEX: 
                setWristShifter(Value.kReverse);
                wristIntake.set(TalonSRXControlMode.PercentOutput, 0.8);
                break;
            
            case HOLD_CUBE:
            
                wristIntake.set(TalonSRXControlMode.PercentOutput, 0.00); //TODO was orginally 0.05
                TaskScheduler.getInstance().schedule("flashing", () -> Intake.getInstance().updateDriverFeebackLEDs(), () -> DriverFeedback.getInstance().setColor(LEDLights.RED), 2.0);
                setWristShifter(Value.kReverse);

                break;
            
            case RETRACT_WITH_CONE:
                setEscalatorSetpoint(12);
                if(!inPosition()){
                    
                }
                break;
            
            case HOLD_CONE:
                wristIntake.set(TalonSRXControlMode.PercentOutput, 0.0);
                escalatorPower = getCancoderPos() > 11.0 ? -0.8 : -0.12;

                if (!isEscalatorZeroed()) {
                    mEscalator.set(ControlMode.PercentOutput, escalatorPower);
                    
                    
                        setWristShifter(Value.kReverse);
                    
                  
                } else {
                    stopEscalator();
                    resetEscalatorEncoder();
                }
                
                TaskScheduler.getInstance().schedule("flashing", () -> Intake.getInstance().updateDriverFeebackLEDs(), () -> DriverFeedback.getInstance().setColor(LEDLights.RED), 2.0);
                break;
            
            case SCORE_CUBE_LOW:
                wrist.set(Value.kReverse);
                wristIntake.set(TalonSRXControlMode.PercentOutput, RobotState.isAutonomous() ? -1 : -0.80);

            break;


            case SPIT_GAME_PIECE:
                wristIntake.configOpenloopRamp(0.00);
                mEscalator.set(ControlMode.PercentOutput,0.05);
                if(Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE){
                    wristIntake.set(TalonSRXControlMode.PercentOutput, 0.40);//0.35
                  
                }else{
                    wristIntake.set(TalonSRXControlMode.PercentOutput, -0.750);
                }
                break;

           
            case EXTEND:
                extendController.setP(Robot.bot.escalatorP);
                extendController.setD(Robot.bot.escalatorD);
                extendController.setI(Robot.bot.escalatorI);
                alreadyRun = false;
                hasGrabbed = false;

                TaskScheduler.getInstance().allow("flashing");

                if(getCurPos() > (desiredScoringHeight * (0.8))){
                    SmartDashboard.putBoolean("Escalator fired", true);
                    if(Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE){
                      wrist.set(Value.kForward);
                    }
                } else {
                    SmartDashboard.putBoolean("Escalator fired", false);
                }

                if (startExtendTime < 0) {
                    startExtendTime = Timer.getFPGATimestamp();
                }
                
                if (inPosition()) {
                    Escalator.getInstance().setEscalatorState(EscalatorStates.HOLD);
                }

                setEscalatorPosToSetpoint(desiredScoringHeight);
               
                break;

            case HOLD:
                TaskScheduler.getInstance().allow("clearance_drop");
                mEscalator.set(ControlMode.PercentOutput, 0.05);
                break;
          

            default:
                break;
        }

        Swerve.getInstance().checkSuperSlowMode(getCurPos() > (Robot.bot.MID_SCORING_HEIGHT * 0.75));

        if(curState != EscalatorStates.SPIT_GAME_PIECE){
            wristIntake.configOpenloopRamp(0.50);
        }
    }

    public double getEscalatorSetpoint() {
        return escalatorSetpoint;
    }
    
    public void setEscalatorSetpoint(double pos){
        escalatorSetpoint = pos;
    }

    public void setDesiredScoringHeight(double height) {
        desiredScoringHeight = height;
    }

    public void setDesiredScoringHeight(ScoringHeight height) {
        if (height == ScoringHeight.LOW){
            desiredScoringHeight = Robot.bot.LOW_SCORING_HEIGHT;
        }else if (height == ScoringHeight.MID){
            if(Intake.getInstance().getDesiredGamePiece() == GamePiece.CUBE){
                desiredScoringHeight = Robot.bot.MID_CUBE_SCORE_HEIGHT;
                
            }else{
                desiredScoringHeight = Robot.bot.MID_SCORING_HEIGHT;
            }
        }
        else{
            if(Intake.getInstance().getDesiredGamePiece() == GamePiece.CUBE){
                desiredScoringHeight = RobotState.isAutonomous() ? Robot.bot.HIGH_CUBE_SCORE_HEIGHT_AUTON : Robot.bot.HIGH_CUBE_SCORE_HEIGHT;
            }else{
                desiredScoringHeight = RobotState.isAutonomous() ? Robot.bot.HIGH_SCORING_HEIGHT_AUTON :Robot.bot.HIGH_SCORING_HEIGHT;
            }
        }
    }

    public boolean cubeGot() {
        return Intake.getInstance().getToFRange() < 170;
    }

    public double getDesiredScoringHeight() {
        return desiredScoringHeight;
    }

    public void setEscalatorPos(double pos) {
        if (getState() == EscalatorStates.EXTEND && getCurPos() < (desiredScoringHeight * 0.5))
        //    mEscalator.setOpenLoopRampRate(0.0);
          mEscalator.setClosedLoopRampRate(0.0);
        else
          mEscalator.setClosedLoopRampRate(0.0); 
            // mEscalator.setOpenLoopRampRate(0.0);

        if (getState() == EscalatorStates.EXTEND && Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE)
            mEscalator.selectProfileSlot(1);
        else
            mEscalator.selectProfileSlot(0);
        
     //   double power = extendController.calculate(getCurPos(), pos);
        mEscalator.setEncoderPosition(getCurPos());
        mEscalator.set(ControlMode.Position, pos);
    }

    public boolean isAtBottom() {
        return this.isEscalatorZeroed() || (getCurPos() < 2);
    }

    public WaitUntil waitForBottom() {
        return new WaitUntil(this::isAtBottom);
    }

    public void setEscalatorPosToSetpoint(double pos) {
        setEscalatorSetpoint(pos);
        setEscalatorPos(getEscalatorSetpoint());
    }

    public boolean inSpecificPosition(double position) {
        if (startExtendTime < 0)
            startExtendTime = Timer.getFPGATimestamp();
        
        boolean timeoutMet = false; //(Timer.getFPGATimestamp() - startExtendTime) > EXTEND_TIMEOUT;
        boolean reached = timeoutMet || (getCurPos() > (position - posTolerance) && getCurPos() < (position + posTolerance));
        
        SmartDashboard.putBoolean("extension timeout", timeoutMet);
        if (reached || timeoutMet)
            startExtendTime = -1;
        
        return reached;
    }

    public boolean pastSafePoint() {
        return getCurPos() > (getDesiredScoringHeight() * 0.8);
    }

    public boolean inPosition() {
        return this.inSpecificPosition(getEscalatorSetpoint());
    }

    public double getCurPos() {
      return escalatorEncoder.getPosition()  * (1/inchesToDegrees) * (6.0/7) * 1.022046;
      //return mEscalator.getEncoderPosition();
    }

    public double getCancoderPos(){
        return getCurPos();
    }

    public void resetEscalatorEncoder() {
        escalatorEncoder.setPosition(0.0);
        mEscalator.setEncoderPosition(0.0);
    }

    public boolean isEscalatorZeroed() {
        return !zeroSwitch.get();
    }

    public boolean hasGrabbed() {
        return this.hasGrabbed;
    }

    public void zero() {
        setEscalatorPos(0);
    }
    
    public void stopEscalator() {
        mEscalator.set(ControlMode.PercentOutput, 0.0);
    }

    public void stopClaw() {
    //    setClawState(ClawStates.RELEASE);
    }

    public void stopWrist() {
        setWristShifter(Value.kOff);
    }

    @Override
    public void onStop() {
        setEscalatorState(EscalatorStates.OFF);
    }
}