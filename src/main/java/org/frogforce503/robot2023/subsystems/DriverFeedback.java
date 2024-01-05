package org.frogforce503.robot2023.subsystems;

import org.frogforce503.lib.drivers.CANSparkMaxWrapper;
import org.frogforce503.lib.drivers.CANSparkMaxWrapper.ControlMode;
import org.frogforce503.lib.util.TaskScheduler;
import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.RobotState.GamePiece;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;


public class DriverFeedback extends Subsystem{
    public CANdle lightLED;
    public enum LEDLights {
        RED, GREEN, BLUE, YELLOW, PURPLE, PURPLE_CONE, TEAL, WHITE, ORANGE, OFF, FLASH, RAINBOW, STARTUP
    }

    double timeStamp = 0;
    // Animation strobe = new StrobeAnimation(0, 0, 255);
    Animation strobe = new ColorFlowAnimation(0, 255, 0);

    static DriverFeedback instance;
    NetworkTable hudTable;
    public static DriverFeedback getInstance(){
        if (instance == null){
            instance =  new DriverFeedback();
        }
        return instance;
    }


    public DriverFeedback() {
        lightLED = new CANdle(Robot.bot.candleID);
        CANdleConfiguration config = new CANdleConfiguration();

        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1.0;

        lightLED.configFactoryDefault();
        hudTable = NetworkTableInstance.getDefault().getTable("hud");
    }

    public NetworkTable getHUDTable() {
        return this.hudTable;
    }

    public void setColorCone(LEDLights color) {
        Intake.getInstance().setDesiredGamePiece(GamePiece.CONE);
        setColor(color);
    }

    public void setColor(LEDLights color) {
        if (color != LEDLights.FLASH)
            lightLED.animate(null);
        
        switch (color) {
            case RED:
                lightLED.setLEDs(255, 0, 0);
                // lightLED.animate()
                break;
            case GREEN:
                lightLED.setLEDs(0, 255, 0);
                break;
            case BLUE:
                lightLED.setLEDs(0, 0, 255);
                break;
            case YELLOW:
                lightLED.setLEDs(244, 250, 12, 0, 0, 1000);
                break;
            case PURPLE_CONE:
                lightLED.setLEDs(255, 255, 255, 0,  0, 1000);
                break;
            case PURPLE:
                lightLED.setLEDs(255, 0, 255,0,  0, 1000);
                break;
            case TEAL:
                lightLED.setLEDs(0, 255, 255);
                break;
            case WHITE:
                lightLED.setLEDs(255, 255, 255);
                break;
            case ORANGE:
                lightLED.setLEDs(250, 135, 5);
                break;
            case FLASH:
                lightLED.animate(strobe, 0);
                break;
            case RAINBOW:
                // lightLED.animate(new RGBFadeAnimation())
                lightLED.animate(new RainbowAnimation(1.0, 0.5, 300), 0);
                break;

            case STARTUP:
                if(timeStamp == 0){
                    timeStamp = Timer.getFPGATimestamp();
                }

                if(Timer.getFPGATimestamp() - timeStamp < 1){
                    lightLED.setLEDs(255, 0, 0, 0, 0, 30);
                }else if(Timer.getFPGATimestamp() - timeStamp < 2){
                    lightLED.setLEDs(244, 250, 12, 0, 0, 70);
                }else{
                    lightLED.setLEDs(0, 255, 0);
                }
                break;

            case OFF:
                lightLED.setLEDs(0, 0, 0);
                break;
        }
    }



    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }


    public void setOff(){
        lightLED.setLEDs(0, 0, 0);
    }



    @Override
    public void onStart() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub
        
    }
}