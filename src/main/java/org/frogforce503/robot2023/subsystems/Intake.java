package org.frogforce503.robot2023.subsystems;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.lib.util.TaskScheduler;
import org.frogforce503.lib.util.snUtil;
import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.StateEngine;
import org.frogforce503.robot2023.RobotState.GamePiece;
import org.frogforce503.robot2023.StateEngine.RobotStates;
import org.frogforce503.robot2023.subsystems.DriverFeedback.LEDLights;
import org.frogforce503.robot2023.subsystems.Escalator.EscalatorStates;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {

    snUtil snUtil = new snUtil();

    public enum IntakeStates {
        STOWED, INTAKE, REVERSE, FLIP, CLEARANCE_FOR_SCORE, CLEARANCE_FOR_CONE, REINDEX, OFF, AGITATE,
        TEST_RETRACT, TEST_EXTEND, HOLD_CONE, AUTON_BELT_MID
    }
    
    private boolean overridePixy = false;

    private GamePiece desiredGamePiece = GamePiece.CONE;

    // private DoubleSolenoid intakeLock;

    private double agitateSign = -1;
    Debouncer pixyHasSeenDebouncer;

    // boolean hasSeen = false;
    // boolean hadSeen = false;

    // Extender

    double extensionDistance = 20;

    public CANSparkMaxWrapper extender;
    public CANCoder extensionEncoder;

    PIDController extendController;
    PIDController retractController;
    
    


    double extenderEncoderToDegrees = (1.0/10.008) *(1/360); // get value

    double minPos = Robot.bot.INTAKE_MIN_POSITION;
    double maxPos = minPos + 94;
    double halfPos = (minPos + maxPos) / 2;

    double extensionSetPoint = minPos + 84; // + 66;
    double retractionSetPoint = minPos + 1;
    double intakeConeSetpoint = minPos + 35;
    double shootMidCubeAutoSetpoint = minPos + 77;
    double holdConeSetpoint = minPos + 10;
    double scoreClearanceSetPointCube = minPos + 88;
    double scoreClearanceSetPointCone = minPos + 71;
    double wheelExtensionClearance = minPos + 17; // when wheels dont need to be reversed anymore

    GenericEntry extenderPosition;

    double extendingP = Robot.bot.extendingP;
    double extendingI = Robot.bot.extendingI;
    double extendingD = Robot.bot.extendingD;

    double retractingP = Robot.bot.retractingP;
    double retractingI = Robot.bot.retractingI;
    double retractingD = Robot.bot.retractingD;
 

    // Intake
    public CANSparkMaxWrapper bottomIntakeRoller;
    public CANSparkMaxWrapper topIntakeRoller;

    // Indexer
    public CANSparkMaxWrapper suctionWheels;
    double suctionWheelsP = Robot.bot.suctionWheelsP;
    double suctionWheelsI = Robot.bot.suctionWheelsI;
    double suctionWheelsD = Robot.bot.suctionWheelsD;
    double suctionWheelsFF = Robot.bot.suctionWheelsFF;

    public CANSparkMaxWrapper beltMotor;
    double beltMotorP = Robot.bot.beltMotorP;
    double beltMotorI = Robot.bot.beltMotorI;
    double beltMotorD = Robot.bot.beltMotorD;
    double beltMotorFF = Robot.bot.beltMotorFF;

    IntakeStates currentIntakeState;

    // Time of Flight
    private TimeOfFlight ToF;

    private static Intake instance = null;
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance; 
    }



    public Intake() {
        currentIntakeState = IntakeStates.STOWED;

        pixyHasSeenDebouncer = new Debouncer(0.1);

        if (RobotBase.isReal()) {
            ToF = new TimeOfFlight(Robot.bot.ToFID);
            ToF.setRangingMode(RangingMode.Medium, 24);
        }


        // Extender

        extender = new CANSparkMaxWrapper(Robot.bot.extenderID, MotorType.kBrushless);
        extender.setIdleMode(IdleMode.kBrake);  // Change to kBrake during the compeitition
        extender.setSmartCurrentLimit(45);
        extender.setInverted(true);
        extendController = new PIDController(extendingP, extendingI, extendingD);
        retractController = new PIDController(retractingP, retractingI, retractingD);

        retractController.setTolerance(6);
        extendController.setTolerance(2);

        extensionEncoder = new CANCoder(11);
        extensionEncoder.configFactoryDefault();
        extensionEncoder.configSensorDirection(true);
        extensionEncoder.setPosition(extensionEncoder.getAbsolutePosition());
        extensionEncoder.configSensorDirection(true, 0);



        // Intake
        bottomIntakeRoller = new CANSparkMaxWrapper(Robot.bot.bottomIntakeRollerID, MotorType.kBrushless);
        bottomIntakeRoller.setIdleMode(IdleMode.kCoast);
        bottomIntakeRoller.setSmartCurrentLimit(45);
        bottomIntakeRoller.setInverted(Robot.bot.intakeReversed);

        bottomIntakeRoller.setP(0, Robot.bot.bottomIntakeRollerP);
        bottomIntakeRoller.setI(0, Robot.bot.bottomIntakeRollerI);
        bottomIntakeRoller.setD(0, Robot.bot.bottomIntakeRollerD);
        bottomIntakeRoller.setFF(0, Robot.bot.bottomIntakeRollerFF);
        bottomIntakeRoller.selectProfileSlot(0);

        topIntakeRoller = new CANSparkMaxWrapper(Robot.bot.topIntakeRollerID, MotorType.kBrushless);
        topIntakeRoller.setIdleMode(IdleMode.kCoast);
        topIntakeRoller.setSmartCurrentLimit(45);
        topIntakeRoller.setInverted(!Robot.bot.intakeReversed); //TODO double check

        topIntakeRoller.setP(0, Robot.bot.topIntakeRollerP);
        topIntakeRoller.setI(0, Robot.bot.topIntakeRollerI);
        topIntakeRoller.setD(0,Robot.bot.topIntakeRollerD);
        topIntakeRoller.setFF(0, Robot.bot.topIntakeRollerFF);

        topIntakeRoller.selectProfileSlot(0);

       

        // --------------- Indexer --------------- //

        suctionWheels = new CANSparkMaxWrapper(Robot.bot.suctionWheelsID, MotorType.kBrushless);
        suctionWheels.setIdleMode(IdleMode.kCoast);
        suctionWheels.setSmartCurrentLimit(35);
        suctionWheels.setInverted(false);

        suctionWheels.setP(0, Robot.bot.suctionWheelsP);
        suctionWheels.setI(0, Robot.bot.suctionWheelsI);
        suctionWheels.setD(0, Robot.bot.suctionWheelsD);
        suctionWheels.setFF(0, Robot.bot.suctionWheelsFF);

        beltMotor = new CANSparkMaxWrapper(Robot.bot.beltMotorID, MotorType.kBrushless); 
        beltMotor.setIdleMode(IdleMode.kCoast);
        
        beltMotor.setP(0, Robot.bot.beltMotorP);
        beltMotor.setI(0, Robot.bot.beltMotorI);
        beltMotor.setD(0, Robot.bot.beltMotorD);
        beltMotor.setFF(0, Robot.bot.beltMotorFF);

        beltMotor.setInverted(Robot.bot.beltMotorInverted);
        bottomIntakeRoller.setSmartCurrentLimit(45);

        // intakeLock = new DoubleSolenoid(PneumaticsModuleType.REVPH, Robot.bot.intakeLockForwardID, Robot.bot.intakeLockReverseID);
        // intakeLock.set(Value.kReverse);

        extender.setPositionConversionFactor(extenderEncoderToDegrees);
        extender.setEncoderPosition(extensionEncoder.getAbsolutePosition());
        
        if (RobotBase.isReal()) {
            ToF.setRangingMode(RangingMode.Short, 24);
            ToF.setRangeOfInterest(4, 4, 12, 12);
        }

    }


    @Override
    public void onStart() {
        setIntakeState(IntakeStates.STOWED);
        overridePixy = false;
        // intakeLock.set(Value.kReverse);
    }

    @Override
    public void outputTelemetry() {
      
    }

    public boolean getPixyOverride(){
        return overridePixy;
    }

    public void togglePixyOverride(){
        overridePixy = !overridePixy;
    }

    public double getToFRange() {
        return RobotBase.isReal() ? ToF.getRange() : 0;
    }

    boolean stowed = false;

    @Override
    public void onLoop() {
        // updatePIDs();


        SmartDashboard.putString("Intake State", currentIntakeState.name());
        SmartDashboard.putNumber("Extender Current", extender.getOutputCurrent());
        SmartDashboard.putNumber("Extender Car Wash Current", suctionWheels.getOutputCurrent());
        SmartDashboard.putNumber("Belt Speed", beltMotor.getEncoderVelocity());
        SmartDashboard.putNumber("Car Wash Speed", suctionWheels.getEncoderVelocity());

        SmartDashboard.putNumber("Top Intake Speed", topIntakeRoller.getEncoderVelocity()); //TODO fix smartdashboard data
        SmartDashboard.putNumber("Bottom Intake Speed", bottomIntakeRoller.getEncoderVelocity());
        SmartDashboard.putNumber("Intake Velocity Setpoint", getTargetVelocity());
        
        SmartDashboard.putNumber("Intake Bottom Velocity Error", bottomIntakeRoller.getClosedLoopError());
        SmartDashboard.putNumber("Intake Top Velocity Error", bottomIntakeRoller.getClosedLoopError());

        SmartDashboard.putNumber("Intake Bottom Current", bottomIntakeRoller.getOutputCurrent());
        SmartDashboard.putNumber("Intake Top Current", topIntakeRoller.getOutputCurrent());

        SmartDashboard.putNumber("Intake Position", getExtensionAngle());
        SmartDashboard.putNumber("Intake Top Velocity", topIntakeRoller.getEncoderVelocity());
        SmartDashboard.putNumber("Intake Bottom Velocity", bottomIntakeRoller.getEncoderVelocity());
        SmartDashboard.putBoolean("Intake toggleflip", overridePixy);
        // snUtil.updateComponent("TOF Distance (inches)", ToF.getRange() / 25.4);

        switch (currentIntakeState) {
            case OFF:
                bottomIntakeRoller.set(ControlMode.PercentOutput, 0.0);
                topIntakeRoller.set(ControlMode.PercentOutput, 0.0);

                suctionWheels.set(ControlMode.PercentOutput, 0.0);
                beltMotor.set(ControlMode.PercentOutput, 0.0);
                extender.set(ControlMode.PercentOutput, 0);

                // Retract the extender
                // extender.set(ControlMode.Position, 0);
                extender.set(ControlMode.PercentOutput, retractController.calculate(getExtensionAngle(), retractionSetPoint)); 
                break;

            case STOWED:
                // extender.set(Value.kReverse);
                // extender.set(ControlMode.Position, 0);
                if (Escalator.getInstance().getCancoderPos() < 3.0)
                    setIntakePosition(retractionSetPoint);

                
                beltMotor.set(ControlMode.PercentOutput, 0);

                suctionWheels.set(ControlMode.PercentOutput, 0);

                TaskScheduler.getInstance().allow("belt_delay");
                TaskScheduler.getInstance().allow("intake_reverse_out");

                bottomIntakeRoller.set(ControlMode.PercentOutput, 0);

                if (!isClearToIntakeCone())
                    topIntakeRoller.set(ControlMode.PercentOutput,0.0);

                stowed = true;
              
           
                break;
            case INTAKE:
               
                TaskScheduler.getInstance().allow("belt_stop");
                TaskScheduler.getInstance().allow("pull_down");


                setIntakePosition(extensionSetPoint);
             

                bottomIntakeRoller.set(ControlMode.Velocity, getTargetVelocity());
                topIntakeRoller.set(ControlMode.Velocity, getTargetVelocity());
            

                stowed = false;
                

                // extender.set(ControlMode.PercentOutput, extendController.calculate(getExtensionAngle(), extensionSetPoint));
                // extender.set(ControlMode.Position, extensionDistance);
                
                // TaskScheduler.getInstance().schedule("intake_reverse_out",
                //     () -> leftIntakeMotor.set(ControlMode.Velocity, getTargetVelocity()),
                //     0.50
                // );  \

                // TaskScheduler.getInstance().schedule("belt_delay", ()->{
                suctionWheels.set(ControlMode.Velocity, 8000);
                beltMotor.set(ControlMode.Velocity, 3000);
                    
                // }, 0.1);

                break;
            case CLEARANCE_FOR_CONE:
                setIntakePosition(intakeConeSetpoint);
               // bottomIntakeRoller.set(ControlMode.PercentOutput, 0.0);
               
                topIntakeRoller.set(ControlMode.Velocity, -2500);

                break;

            case HOLD_CONE:
                if(Escalator.getInstance().isEscalatorZeroed()){
                //     TaskScheduler.getInstance().schedule("intake_cone_push_out_routine", () -> {
                //         TaskScheduler.getInstance().schedule("intake_cone_push_out", () -> {
                //             setIntakePosition(holdConeSetpoint);
                //         }, () -> setIntakePosition(intakeConeSetpoint + 10), 0.3);
                //     }, 1.0);
                    setIntakePosition(holdConeSetpoint);
                }else{
                    setIntakePosition(intakeConeSetpoint);
                }
                 bottomIntakeRoller.set(ControlMode.PercentOutput, 0.0);
                 topIntakeRoller.set(ControlMode.PercentOutput, 0.0);
                break;
            
            case CLEARANCE_FOR_SCORE:
              
                setIntakePosition(Intake.getInstance().getDesiredGamePiece() == GamePiece.CUBE ? scoreClearanceSetPointCube : scoreClearanceSetPointCone);
                bottomIntakeRoller.set(ControlMode.PercentOutput, 0.0);
                topIntakeRoller.set(ControlMode.PercentOutput, 0.0);

                // if (getExtensionAngle() > wheelExtensionClearance) {
                //     bottomIntakeRoller.set(ControlMode.PercentOutput, 0.0);
                // } else {
                //     bottomIntakeRoller.set(ControlMode.PercentOutput, -0.45);
                // }
                beltMotor.set(ControlMode.PercentOutput, 0.0);

               suctionWheels.set(ControlMode.PercentOutput, 0.0);
            
            break;

            case REINDEX:
                // extender.set(ControlMode.Position, 0);
                setIntakePosition(retractionSetPoint);

                bottomIntakeRoller.set(ControlMode.PercentOutput, 0.0);
                topIntakeRoller.set(ControlMode.PercentOutput, 0.0);

                suctionWheels.set(ControlMode.Velocity, 3000);
                beltMotor.set(ControlMode.Velocity, 3000);

                TaskScheduler.getInstance().allow("belt_stop");
                TaskScheduler.getInstance().allow("pull_down");
                break;
            
            case AGITATE:
                // extender.set(ControlMode.Position, 0);
                setIntakePosition(retractionSetPoint);
                

                bottomIntakeRoller.set(ControlMode.PercentOutput, 0.0);
                suctionWheels.set(ControlMode.Velocity, 3000 * agitateSign);
                // beltMotor.set(ControlMode.Velocity, 3000 * agitateSign);

                TaskScheduler.getInstance().schedule("agitate_sign_swap", () -> {
                    agitateSign *= -1;
                    TaskScheduler.getInstance().allow("agitate_sign_swap");
                }, agitateSign == 1 ? 1.0 : 0.75);

                TaskScheduler.getInstance().allow("belt_stop");
                TaskScheduler.getInstance().allow("pull_down");
                break;

            case REVERSE:
                // extender.set(ControlMode.Position, extensionDistance);
                setIntakePosition(intakeConeSetpoint);

                if (RobotState.isAutonomous()) {
                    bottomIntakeRoller.set(ControlMode.PercentOutput,1.0);
                    topIntakeRoller.set(ControlMode.PercentOutput, -1.0);
    
                    suctionWheels.set(ControlMode.Velocity, -3000);
                    beltMotor.set(ControlMode.Velocity, -3000);
                } else {
                    bottomIntakeRoller.set(ControlMode.PercentOutput, 1.0);
                    topIntakeRoller.set(ControlMode.PercentOutput, -1.0);
    
                    suctionWheels.set(ControlMode.PercentOutput, -1.0); //300
                    beltMotor.set(ControlMode.PercentOutput, -1.0);
                }
                break;

            case AUTON_BELT_MID:
                setIntakePosition(shootMidCubeAutoSetpoint);

                // if (isClearForPneumatic())
                topIntakeRoller.set(ControlMode.PercentOutput, 1.0);

                break;
                
            default:
                break;
        }
    }

    @Override
    public void onStop() {
        currentIntakeState = IntakeStates.STOWED;
    }

    public void setIntakePosition(double setpoint) {

        SmartDashboard.putNumber("Intake Position Setpoint", setpoint);
        // SmartDashboard.putBoolean("Intake Lock Enabled", intakeLock.get() == Value.kForward);

        // if (setpoint == retractionSetPoint && Math.abs(getExtensionAngle() - retractionSetPoint) < 3) {
        //     intakeLock.set(Value.kForward);
        //     extender.set(ControlMode.PercentOutput, 0.0);
        // } else {
            // intakeLock.set(Value.kReverse);

        boolean intakeLocked = false;

        if (setpoint == retractionSetPoint && Math.abs(getExtensionAngle() - retractionSetPoint) < 3) {
            intakeLocked = true;
            stowed = true;
        } else {
            intakeLocked = false;
        }

        SmartDashboard.putBoolean("Intake Locked", intakeLocked);

        if (Math.abs(getExtensionAngle() - setpoint) < Robot.bot.INTAKE_EXTENSION_TOLERANCE) {
            // if (getExtensionAngle() > halfPos) {
                // if (maxPos - getExtensionAngle() > 5) {
                //     extender.set(ControlMode.PercentOutput, 0.08);
                // } else {
                extender.set(ControlMode.PercentOutput, 0);
                // }
            // } else {
            //     if (Math.abs(getExtensionAngle() - minPos) > 15) {
            //         extender.set(ControlMode.PercentOutput, -0.0625);
            //     } else {
            //         extender.set(ControlMode.PercentOutput, 0);
            //     }
            // }
        } else {
            // System.out.println("running intake");
           
            double PIDoutput = (setpoint > getExtensionAngle() ? extendController : retractController).calculate(getExtensionAngle(), setpoint);
            SmartDashboard.putNumber("Extension PID Output", PIDoutput);
            if (stowed) {
                intakeLog("DEPLOY");
                // DriverFeedback.getInstance().setColor(LEDLights.BLUE);
                stowed = false;
            }
            extender.set(ControlMode.PercentOutput, PIDoutput);
        }
        // }

    }

    public void intakeLog(String message) {
        // System.out.println("INTAKE " + message + " AT " + Timer.getFPGATimestamp());
    }

    public boolean isClearToIntakeCone() {
        return getExtensionAngle() >= (intakeConeSetpoint - 5);
    }

    public boolean isClearForPneumatic() {
        return getExtensionAngle() >= (intakeConeSetpoint + 10);
    }


    public boolean isClearForScore(){
        boolean cube = Intake.getInstance().getDesiredGamePiece() == GamePiece.CUBE;
        return getExtensionAngle() >= ((cube ? scoreClearanceSetPointCube : scoreClearanceSetPointCone) - (cube ? 80 : 60));
    }


    private double getHoldingSpeed() {
        double omega = Swerve.getInstance().getVelocity().omegaRadiansPerSecond;
        double m = 0.2 / (.0 * Math.PI);
        double output = (m * omega) + 0.0625;
        return Math.min(output, 0.2);
    }

    private double getTargetVelocity() {
        if (this.desiredGamePiece == GamePiece.CUBE){
            return RobotState.isAutonomous() ?  Robot.bot.intakeCubeAutoVelocity : Robot.bot.intakeCubeVelocity;
           
        }
        return Robot.bot.intakeConeVelocity;
    }

    public double getAbsoluteExtensionAngle() {
        return this.extensionEncoder.getAbsolutePosition();
    }

    public double getExtensionAngle() {
        return extensionEncoder.getAbsolutePosition();
    }


    public void setIntakeState(IntakeStates state) {
        if (state != currentIntakeState) {
            DriverFeedback.getInstance().getHUDTable()
                .getEntry("intakeState")
                .setString(currentIntakeState.name());
        }
        
        currentIntakeState = state;
    }

    public IntakeStates getControlState() {
        return currentIntakeState;
    }

    public void setDesiredGamePiece(GamePiece desiredGamePiece) {
        //if (this.desiredGamePiece != desiredGamePiece) {
            // bottomIntakeRoller.selectProfileSlot(desiredGamePiece == GamePiece.CONE ? 0 : 1);
            this.desiredGamePiece = desiredGamePiece;

            updateDriverFeebackLEDs();
            
        //}
    }

    public void updateDriverFeebackLEDs() {
        DriverFeedback.getInstance().getHUDTable()
                .getEntry("selectedGameElement")
                .setString(this.desiredGamePiece.name());
        
        DriverFeedback.getInstance().setColor(desiredGamePiece.color);
    }

    public void resetAgitateSign() {
        agitateSign = 1;
        TaskScheduler.getInstance().allow("agitate_sign_swap");
    }

    public void cycleAgitateSign() {
        TaskScheduler.getInstance().allow("agitate_sign_swap");
    }

    public GamePiece getDesiredGamePiece() {
        return this.desiredGamePiece;
    }

    public IntakeStates getState() {
        return currentIntakeState;
    }

    public boolean isExtended() {
        if (Math.abs(getExtensionAngle() - extensionSetPoint) < 5) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isRetracted() {
        if (Math.abs(getExtensionAngle() - retractionSetPoint) < 5) {
            return true;
        } else {
            return false;
        }
    }

    public double calculatePercentOutput(double setPoint) {
        return extendController.calculate(getExtensionAngle(), setPoint);
    }
}