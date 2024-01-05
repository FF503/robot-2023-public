package org.frogforce503.robot2023;

import org.frogforce503.lib.util.TaskScheduler;
import org.frogforce503.robot2023.RobotState.GamePiece;
import org.frogforce503.robot2023.subsystems.Escalator;
// import org.frogforce503.robot2023.subsystems.Pixy;
import org.frogforce503.robot2023.subsystems.Escalator.EscalatorStates;
import org.frogforce503.robot2023.subsystems.Intake;
import org.frogforce503.robot2023.subsystems.Intake.IntakeStates;
// import org.frogforce503.robot2023.subsystems.Pixy.ConeOrientation;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StateEngine {
  //  Notifier thread2 = new Notifier(() -> executeThread2());
    boolean runOnce = false;

    String grabHeight ="";

    int counter = 0;
    boolean dropAllowed = false;

    private static StateEngine instance;

    public StateEngine() {
      
    }

    public static StateEngine getInstance() {
        if (instance == null) { instance = new StateEngine(); }
        return instance;
    }

    RobotStates curState = RobotStates.DISABLED;
    RobotStates prevState = RobotStates.DISABLED;

    public RobotStates getRobotState() {
        return curState;
    }

    public RobotStates getPrevState() {
        return prevState;
    }

    public void setToPreviousState() {
        setRobotState(prevState);
    }

    public void setRobotState(RobotStates newState) {
        if(newState != curState){
            prevState = curState;
            curState = newState;
        }
    }

    public void executeThread1() {
        TaskScheduler.getInstance().periodic();
        
        Intake.getInstance().onLoop();
        Escalator.getInstance().onLoop();

    }

    public void initThread2() {
        // thread2.startPeriodic(0.02);
    }

    public void executeThread2() {
        //add subsystems(call onLoop)
    }

    public void updateState() {
        SmartDashboard.putString("StateEngine State", curState.name());
        // SmartDashboard.putBoolean("Grab height bool", Pixy.getInstance().getConeOrientation() == ConeOrientation.BASE_BACK && Pixy.getInstance().getObject() == GamePiece.CONE);

        switch(curState) {
            case DISABLED:
            dropAllowed = false;
               // Intake.getInstance().setIntakeState(IntakeStates.STOWED);
                Escalator.getInstance().setEscalatorState(EscalatorStates.OFF);
                break;

            case IDLE:
            dropAllowed = false;
                TaskScheduler.getInstance().allow("intake_clearance_rub");
                TaskScheduler.getInstance().allow("clearance_drop");
                TaskScheduler.getInstance().allow("INDEX_CANCEL");

                
              
                Escalator.getInstance().setEscalatorState(EscalatorStates.OFF);

                if(Intake.getInstance().getState() == IntakeStates.HOLD_CONE){}
                else{
                   
                    Intake.getInstance().setIntakeState(IntakeStates.STOWED);
                }

                break;

            case INTAKING:
                dropAllowed = false;
                TaskScheduler.getInstance().allow("INDEX_CANCEL");

                // TaskScheduler.getInstance().schedule("intake_wait", () -> {
                if(Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE){

                    if(Escalator.getInstance().getState() == EscalatorStates.HOLD_CONE){
                        Intake.getInstance().setIntakeState(IntakeStates.HOLD_CONE);
                    }else{
                        Intake.getInstance().setIntakeState(IntakeStates.CLEARANCE_FOR_CONE);
                        // TaskScheduler.getInstance().allow("intake_cone_push_out_routine");
                        // TaskScheduler.getInstance().allow("intake_cone_push_out");
                    }

                    if(!runOnce && Intake.getInstance().isClearToIntakeCone()){
                        Escalator.getInstance().setEscalatorState(EscalatorStates.INTAKE_CONE);
                        toggleRun(true);
                    }
                    // }
                }else{

                    if(!runOnce){
                        Escalator.getInstance().setEscalatorState(EscalatorStates.INTAKE_CUBE);
                        toggleRun(true);
                    }
                    
                    if(Intake.getInstance().getToFRange() < 190){
                        // setRobotState(RobotStates.INDEXING);
                        Intake.getInstance().setIntakeState(IntakeStates.STOWED);
                    }else {
                        Intake.getInstance().setIntakeState(IntakeStates.INTAKE);
                    }
                }
                // }, 0.125);

                // if (Pixy.getInstance().getConeOrientation() == ConeOrientation.BASE_BACK){
                //     grabHeight = "HIGH";
                // } else {
                //     grabHeight = "LOW";
                // }
           
                break;
         
            case REVERSE_INTAKE:
                Intake.getInstance().setIntakeState(IntakeStates.REVERSE);
                Escalator.getInstance().setEscalatorState(EscalatorStates.CLEARANCE_FOR_INTAKING);

                break;

            case REINDEX: 
                Escalator.getInstance().setEscalatorState(EscalatorStates.REINDEX);
                Intake.getInstance().setIntakeState(IntakeStates.REINDEX);

                // if (Pixy.getInstance().getConeOrientation() == ConeOrientation.BASE_BACK) {
                //     grabHeight = "HIGH";
                // } else {
                //     grabHeight = "LOW";
                // }

                break;

            case INDEXING: // automatic form of reindex

                TaskScheduler.getInstance().schedule("INDEX_CANCEL", () -> {
                    if (curState == RobotStates.INDEXING)
                        setRobotState(RobotStates.IDLE);
                }, 1.65);

                if (Escalator.getInstance().cubeGot()) {
                    setRobotState(RobotStates.IDLE);
                    break;
                }
                
                Escalator.getInstance().setEscalatorState(EscalatorStates.REINDEX);
                Intake.getInstance().setIntakeState(IntakeStates.REINDEX);

                // if (Pixy.getInstance().getConeOrientation() == ConeOrientation.BASE_BACK) {
                //     grabHeight = "HIGH";
                // } else {
                //     grabHeight = "LOW";
                // }

                break;


            case AGITATE:
                Escalator.getInstance().setEscalatorState(EscalatorStates.CLEARANCE_FOR_INTAKING);
                Intake.getInstance().setIntakeState(IntakeStates.AGITATE);

                // if (Pixy.getInstance().getConeOrientation() == ConeOrientation.BASE_BACK) {
                //     grabHeight = "HIGH";
                // } else {
                //     grabHeight = "LOW";
                // }

                break;
            case ZERO:
                TaskScheduler.getInstance().allow("retract_after_tele_drop");
                if (Escalator.getInstance().getCurPos() <= 8.0){

                    Intake.getInstance().setIntakeState(IntakeStates.STOWED);
                    
                }

                if (Escalator.getInstance().isEscalatorZeroed()){
                    setRobotState(RobotStates.IDLE);
                    break;
                }

               
                    
                Escalator.getInstance().setEscalatorState(EscalatorStates.ZEROING);
                
              
              
               
                break;

            // case GRAB:

            //     if (!Escalator.getInstance().hasGrabbed()) {
            //         if (grabHeight.equals("HIGH")) {
            //             Escalator.getInstance().setEscalatorState(EscalatorStates.GRAB_HIGH);
            //         } else {
            //             Escalator.getInstance().setEscalatorState(EscalatorStates.GRAB_LOW);
            //         }
            //     }
                
            //     Intake.getInstance().setIntakeState(IntakeStates.OFF);
            //     break;
                
            case PRE_DROP:
                TaskScheduler.getInstance().allow("drop_cone");
                TaskScheduler.getInstance().allow("belt_stop"); // both used by intake
                TaskScheduler.getInstance().allow("pull_down");
                

                // if (Escalator.getInstance().pastSafePoint()) {
                //     Intake.getInstance().setIntakeState(IntakeStates.STOWED);
                // } else {

               
                Intake.getInstance().setIntakeState(IntakeStates.CLEARANCE_FOR_SCORE);
                
               // }
                

                // counter++;
                // if(counter >= 15){
                if (Intake.getInstance().isClearForScore()) {
                    Escalator.getInstance().setEscalatorState(EscalatorStates.EXTEND);

                    if (Escalator.getInstance().inSpecificPosition(Escalator.getInstance().getDesiredScoringHeight())) {
                        
                        setRobotState(RobotStates.READY_TO_DROP);
                    }
                }
                // }

                
                break;
            
            case CUBE_LOW_SCORE:
                TaskScheduler.getInstance().allow("drop_cone");
                TaskScheduler.getInstance().allow("belt_stop"); // both used by intake
                TaskScheduler.getInstance().allow("pull_down");
                
                Escalator.getInstance().setEscalatorState(EscalatorStates.SCORE_CUBE_LOW);
                Intake.getInstance().setIntakeState(IntakeStates.REVERSE);
                if(dropAllowed){
                    setRobotState(RobotStates.ZERO);
                    dropAllowed = false;
                }

                break;

            case READY_TO_DROP:
                TaskScheduler.getInstance().allow("setToDrop");
                // TaskScheduler.getInstance().allow("is_intake_clear");

                if (dropAllowed) {
                    Escalator.getInstance().wrist.set(Value.kForward);
                    TaskScheduler.getInstance().schedule("drop_cone", () -> { 
                        Escalator.getInstance().setEscalatorState(EscalatorStates.OFF);
                       setRobotState(RobotStates.ZERO); 
                       TaskScheduler.getInstance().allow("cone_shoot_delay");
                    },()->{
                        TaskScheduler.getInstance().schedule("cone_shoot_delay", ()-> {

                            Escalator.getInstance().setEscalatorState(EscalatorStates.SPIT_GAME_PIECE);
                           
                        },  Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE ? 0.2 : 0.0);
                       

                    }, Intake.getInstance().getDesiredGamePiece() == GamePiece.CONE ? 0.8 : 0.6);
                    dropAllowed = false;
                }

                break;

            case MANUAL_SCORE:
                Escalator.getInstance().setEscalatorState(EscalatorStates.SPIT_GAME_PIECE);
               // Intake.getInstance().setIntakeState(IntakeStates.STOWED);
            break;

            case DROPPING:
                
                break;

            case AUTON_INTAKING:
                break;

            case AUTON_PRE_DROP:
                break;

            case AUTON_DROPPING:
                break;

            case AUTON_EJECTING:
                break;

            case MANUAL_EJECTING:
                break;

            case EJECT_THROUGH_INTAKE:
                Intake.getInstance().setIntakeState(IntakeStates.REVERSE);
                Escalator.getInstance().setEscalatorState(EscalatorStates.SCORE_CUBE_LOW);
                break;

            case EJECT_THROUGH_GRABBER:
                Escalator.getInstance().setEscalatorState(EscalatorStates.EJECT);
                break;

            case AUTON_MID_SHOOT:
                Intake.getInstance().setIntakeState(IntakeStates.AUTON_BELT_MID);
                Escalator.getInstance().setEscalatorState(EscalatorStates.OFF);
                break;

            default:
                curState = RobotStates.IDLE;
                break;

        }

        if (curState != RobotStates.PRE_DROP)
            counter = 0;

        if (curState != RobotStates.INTAKING){
            toggleRun(false);
        }

       

        executeThread1();
    }

    public void allowDrop() {
        this.dropAllowed = true;
    }

    public void toggleRun(boolean s){
      runOnce = s;
    }

    public enum RobotStates {
        DISABLED, IDLE,
        REVERSE_INTAKE,
        INTAKING, PRE_DROP, READY_TO_DROP, DROPPING, 
        AUTON_INTAKING, AUTON_PRE_DROP, AUTON_DROPPING, AUTON_EJECTING,
        MANUAL_EJECTING,
        EJECT_THROUGH_INTAKE, EJECT_THROUGH_GRABBER,
        ZERO, GRAB, REINDEX, AGITATE, INDEXING, CUBE_LOW_SCORE, INTAKE_CONE, MANUAL_SCORE, AUTON_MID_SHOOT
    }

}