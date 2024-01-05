package org.frogforce503.robot2023;



import java.util.Set;

import org.frogforce503.robot2023.subsystems.DriverFeedback;
import org.frogforce503.robot2023.subsystems.DriverFeedback.LEDLights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotState {
    private static final RobotState instance = new RobotState();
    private Bot currentRobot;
    private Pose2d currentPose = new Pose2d();
    private double currentTheta;
    private GameState gameState = GameState.DISABLED;
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    private double[] fieldCentricSpeeds = { 0, 0 };
    private AllianceColor allianceColor = AllianceColor.RED;
    private boolean allianceColorBeenOverriden = false;
   

    public static RobotState getInstance() {
        return instance;
    }

    public GameState getGameState() {
        return gameState;
    }

    public void setGameState(GameState gamestate) {
        this.gameState = gamestate;
    }

    public boolean isFMSInfoAvailable() {
        Set<String> str  =NetworkTableInstance.getDefault().getTable("FMSInfo").getKeys();
        String out = "FMS KEYS: ";
        for (String s : str) {
            out += s;
        }
        System.out.println(out);
        return false;
    }

    public AllianceColor getAllianceColor() {
        if (allianceColorBeenOverriden)
            return this.allianceColor;

        if (RobotBase.isSimulation())
            return AllianceColor.BLUE;
        
        if (NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false))
            return AllianceColor.RED;
        else
            return AllianceColor.BLUE;
    }

    public void overrideAllianceColor(AllianceColor color) {
        allianceColorBeenOverriden = true;
        this.allianceColor = color;
    }

    public Bot getCurrentRobot() {
        return this.currentRobot;
    }

    public void setCurrentRobot(final Bot currentRobot) {
        this.currentRobot = currentRobot;
    }

    public synchronized Pose2d getCurrentPose() {
        return currentPose;
    }

    public synchronized ChassisSpeeds getCurrentSpeeds() {
        return currentSpeeds;
    }

    public synchronized double[] getFieldCentricSpeeds() {
        return fieldCentricSpeeds;
    }

    public synchronized void setCurrentPose(Pose2d currentPose) {
        this.currentPose = currentPose;
    }

    public synchronized void setCurrentSpeeds(ChassisSpeeds chassisSpeeds) {
        this.currentSpeeds = chassisSpeeds;
    }

    public synchronized void setFieldCentricSpeeds(double xVelocity, double yVelocity) {
        this.fieldCentricSpeeds[0] = xVelocity;
        this.fieldCentricSpeeds[1] = yVelocity;
    }

    // TODO: Make it actually get the theta
    public synchronized double getCurrentTheta() {
        return 0;
    }

    public synchronized void setCurrentTheta(double currentTheta) {
        this.currentTheta = currentTheta;
    }

    public synchronized double getGyroOffset() {
        return 0.0;// this.startingDirection.getGyroOffset();
    }

    public enum GameState {
        AUTON, TELEOP, TEST, DISABLED
    }

    public enum AllianceColor {
        RED, BLUE
    }

    public enum Bot {
        Automatic, CompBot, PracticeBot, MiniMe
    }

    public enum GamePiece {
        CUBE(LEDLights.PURPLE), CONE(LEDLights.YELLOW), NONE(LEDLights.OFF);

        public DriverFeedback.LEDLights color;

        GamePiece(DriverFeedback.LEDLights color) {
            this.color = color;
        }
    }

}