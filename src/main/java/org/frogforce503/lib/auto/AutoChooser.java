package org.frogforce503.lib.auto;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.swing.colorchooser.ColorSelectionModel;

import org.frogforce503.lib.auto.follower.RoutineBuilder;
import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.auto.AutoUtil;
import org.frogforce503.robot2023.auto.comp.Center1;
import org.frogforce503.robot2023.auto.comp.Center2;
import org.frogforce503.robot2023.auto.comp.LeftCable;
import org.frogforce503.robot2023.auto.comp.LeftCableSpin;
import org.frogforce503.robot2023.auto.comp.LeftMaster;
import org.frogforce503.robot2023.auto.comp.RightCable;
import org.frogforce503.robot2023.auto.comp.RightCableMaster;
import org.frogforce503.robot2023.auto.comp.RightCableSpin;
import org.frogforce503.robot2023.auto.comp.RightCableStraight;
import org.frogforce503.robot2023.auto.comp.RightMaster;
import org.frogforce503.robot2023.auto.comp.WheelTestAuto;
import org.frogforce503.robot2023.auto.tuning.ComplexTestPath;
import org.frogforce503.robot2023.auto.tuning.StraightTestPath;
// import org.frogforce503.robot2023.auto.comp.blue.Center1;
// import org.frogforce503.robot2023.auto.comp.blue.Center2;
// import org.frogforce503.robot2023.auto.comp.blue.Center1;
// import org.frogforce503.robot2023.auto.comp.blue.JustBalance;
// import org.frogforce503.robot2023.auto.comp.blue.Left1;
// import org.frogforce503.robot2023.auto.comp.blue.Left2;
// import org.frogforce503.robot2023.auto.comp.blue.Left2Spin;
// import org.frogforce503.robot2023.auto.comp.blue.Right2;
// import org.frogforce503.robot2023.auto.comp.blue.Right2Lakeview;
// import org.frogforce503.robot2023.auto.comp.blue.Right2Spin;
// import org.frogforce503.robot2023.auto.comp.practicebot.CenterDirect2;
// import org.frogforce503.robot2023.auto.comp.practicebot.LeftJuggle3;
// import org.frogforce503.robot2023.auto.comp.practicebot.LeftMaster;
// import org.frogforce503.robot2023.auto.comp.practicebot.RightJuggle3;
// import org.frogforce503.robot2023.auto.comp.blue.Right1;
import org.frogforce503.robot2023.fields.FieldConfig;
import org.frogforce503.robot2023.subsystems.vision.AprilTagHandler;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoChooser {
    // private HashMap<StartingLocation, HashMap<Integer, AutoMode>> AUTO_MAP = new HashMap<>();

    private SendableChooser<StartingLocation> gridSelector;
    private SendableChooser<Integer> numGameElementSelector;
    private SendableChooser<AllianceColor> colorSelector;
    private SendableChooser<Boolean> balanceSelector;

    AutoMode selectedAuto;
    Runnable onReset;
    String selectedAutoName = "NO AUTO SELECTED";
    NetworkTable autoTable;
    private static AutoChooser instance = null;

    StartingLocation lastGrid = StartingLocation.LEFT_GRID;
    int lastGamePieceNumber = 2;
    AllianceColor lastAllianceColor = AllianceColor.RED;

    private Supplier<String> autoNameDisplay;
    private BooleanSupplier autoReadyDisplay;
    
    public static AutoChooser getInstance() {
        if (instance == null)
            instance = new AutoChooser();
        return instance;
    }
    
    public AutoMode buildAuto(StartingLocation grid, int numGamePieces, AllianceColor color, boolean shouldBalnace) {
        
        System.out.println("Selected an auto with " + grid.name() + ", " + numGamePieces + ", " + color.name() + ", " + shouldBalnace);

        // if (color != FieldConfig.getInstance().getDefaultAlliance())
        //     FieldConfig.getInstance().configureOppositeSide(); // doesnm't do anything yet, but will in the future

        // boolean isRedAlliance = color == AllianceColor.RED;

        FieldConfig.getInstance().loadConstants();

        AutoMode selected = null;

        boolean red = color == AllianceColor.RED;
        boolean left = grid == StartingLocation.LEFT_GRID;
        boolean onCable = (left && red) || (!left && !red);

        AutoUtil.isOnCable = onCable;
        
        switch (grid) {
            case LEFT_GRID: {
                switch (numGamePieces) {
                    case 1:
                        selected = new WheelTestAuto();
                        // selected = new LeftMaster(1, shouldBalnace);// new LeftJuggle3();
                    break;
                    case 2:
                        selected = onCable ? new LeftCable() : new LeftMaster(2, shouldBalnace);// new LeftJuggle3();
                    break;
                    case 3:
                        selected = onCable ? new LeftCable() : new LeftMaster(3, shouldBalnace);// new LeftJuggle3();
                        break;
                }
                break;
            }
            case CENTER_GRID: {
                switch (numGamePieces) {
                    case 1:
                        selected = new Center1();
                        break;
                    case 2:
                        selected = null;
                        break;
                    case 3:
                        selected = null;
                        break;
                }
                break;
            }
            case RIGHT_GRID: {
                switch (numGamePieces) {
                    case 1:
                        selected = new RightMaster(1, shouldBalnace);;// new LeftJuggle3();
                        break;
                    case 2:
                        selected = onCable ? new RightCable() : new RightMaster(2, shouldBalnace);// new LeftJuggle3();
                        break;
                    case 3:
                        selected = onCable ? new RightCable() : new RightMaster(3, shouldBalnace);// new LeftJuggle3();
                        break;
                }
                break;
            }
        }

        AutoUtil.isOnCable = (grid == StartingLocation.LEFT_GRID && color == AllianceColor.RED) || (grid == StartingLocation.RIGHT_GRID && color == AllianceColor.BLUE);

        System.out.println("AUTON ON CABLE SIDE: " + AutoUtil.isOnCable);
        
        if (selected != null)
            selected.setShouldBalance(shouldBalnace);

        updateDisplay();

        return selected;
    }
    
    public void initialize() {

        ShuffleboardLayout autoChooserLayout = Shuffleboard.getTab("DriverStation")
            .getLayout("Auto Chooser", BuiltInLayouts.kList).withSize(3, 5)
            .withPosition(5, 0);

        autoTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(Shuffleboard.getTab("DriverStation").getTitle())
            .getSubTable("Auto Chooser");

        
        colorSelector = new SendableChooser<AllianceColor>();
        colorSelector.setDefaultOption("Blue", AllianceColor.BLUE);
        colorSelector.addOption("Red", AllianceColor.RED);

        gridSelector = new SendableChooser<StartingLocation>();

        boolean firstDone = false;
        for (StartingLocation grid : StartingLocation.values()) {
            if (!firstDone) {
                gridSelector.setDefaultOption(grid.name().replaceAll("_", " "), grid);
                firstDone = true;
                continue;
            }
            gridSelector.addOption(grid.name().replaceAll("_", " "), grid);
        }

        numGameElementSelector = new SendableChooser<Integer>();

        firstDone = false;
        for (int i = 1; i <= 3; i++) {
            if (!firstDone) {
                numGameElementSelector.setDefaultOption(i + " Game Piece" + (i == 1 ? "s" : ""), i);
                firstDone = true;
                continue;
            }
            numGameElementSelector.addOption(i + " Game Piece", i);
        }

        balanceSelector = new SendableChooser<Boolean>();
        balanceSelector.setDefaultOption("Yes",true);
        balanceSelector.addOption("No", false);
    
        reset();

        autoChooserLayout.add("Alliance Color", colorSelector).withSize(3, 1)
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kSplitButtonChooser);

        
        autoChooserLayout.add("Number of Game Elements", numGameElementSelector).withSize(3, 1)
            .withPosition(0, 2)
            .withWidget(BuiltInWidgets.kSplitButtonChooser);
        
        autoChooserLayout.add("Balance at End?", balanceSelector).withSize(3, 1)
            .withPosition(0, 3)
            .withWidget(BuiltInWidgets.kSplitButtonChooser);

        autoReadyDisplay = () -> { return selectedAuto != null; };

        autoChooserLayout.addBoolean("Ready to run??", autoReadyDisplay)
            .withSize(3, 1)
            .withPosition(0, 6)
            .withWidget(BuiltInWidgets.kBooleanBox);

        autoTable.getEntry("Commit Auton Config").setBoolean(false);

        autoChooserLayout.addPersistent("Commit Auton Config", isCommitted())
            .withSize(1, 1)
            .withPosition(0, 4)
            .withWidget(BuiltInWidgets.kToggleButton);

        autoChooserLayout.add("Starting Grid", gridSelector).withSize(3, 1)
            .withPosition(0, 1)
            .withWidget(BuiltInWidgets.kSplitButtonChooser);


        autoNameDisplay = () -> {
            if (selectedAuto == null)
                return "INVALID AUTON";
            return selectedAuto.getName();
        };

        autoChooserLayout.addString("Selected Auto Name", autoNameDisplay)
            .withSize(10, 10)
            .withPosition(0, 5);

    }


    private boolean isCommitted() {
        return autoTable.getEntry("Commit Auton Config").getBoolean(false);
    }

    public void periodic() {
        if (gridSelector.getSelected() != lastGrid || numGameElementSelector.getSelected() != lastGamePieceNumber || colorSelector.getSelected() != lastAllianceColor) {
            reset();
        }

        RobotState.getInstance().overrideAllianceColor(colorSelector.getSelected());

        if (autoTable.getEntry("Commit Auton Config").getBoolean(false)) {
            System.out.println("COMMIT BUTTON PRESSED");
            StartingLocation grid = gridSelector.getSelected();
            int count = numGameElementSelector.getSelected();
            AllianceColor color = colorSelector.getSelected();
            FieldConfig.getInstance().loadConstants();

            AprilTagHandler.getInstance().initializeFieldLayout(color);

            boolean shouldBalance = balanceSelector.getSelected().booleanValue();

            AutoMode selected = this.buildAuto(grid, count, color, shouldBalance);

            // if (selected instanceof OptionBasedAuto) {
            //     selected = ((OptionBasedAuto) selected).load(optionChooser.getSelected());
            // }

            // if (selected != null)
            //     selected.setup();

            this.selectedAuto = selected;

            if (this.selectedAuto != null)
                AutonomousExecutor.getInstance().setSelectedAutoMode(selected);

            // drawPathOnField();

            autoTable.getEntry("Commit Auton Config").setBoolean(false);
        }

        lastGrid = gridSelector.getSelected();
        lastGamePieceNumber = numGameElementSelector.getSelected();
        lastAllianceColor = colorSelector.getSelected();
    }

    public void onReset(Runnable onReset) {
        this.onReset = onReset;
    }

    private void reset() {
        // autoTable.getSubTable("AutoPreviewField").getEntry("preview_path").setValue("");
        selectedAuto = null;
        
        if (this.onReset != null)
            this.onReset.run();
    }

    private void updateDisplay() {
        String displayName = autoNameDisplay.get() + "-" + this.colorSelector.getSelected().name();
        autoTable.getEntry("Selected Auto Name").setString(displayName);
        autoTable.getEntry("Ready to run??").setBoolean(autoReadyDisplay.getAsBoolean());

        System.out.println(this.selectedAuto);
    }

    

    // public int[] getSelectedAuton() {

        // if (colorSelector.getSelected() != FieldConfig.getInstance().getDefaultAlliance())
        //     FieldConfig.getInstance().configureOppositeSide(); // fill in this function
    //     return new int[] {
    //         colorSelector.getSelected(),
    //         gridSelector.getSelected(),
    //         numGameElementSelector.getSelected(),
    //         balanceSelector.getSelected()
    //     };
    // }

    public enum StartingLocation {
        LEFT_GRID(Units.feetToMeters(1.75)),
        CENTER_GRID(-Units.inchesToMeters(0)),
        RIGHT_GRID(-Units.feetToMeters(0.5));

        public double climbOffset;
        StartingLocation(double c) {
            this.climbOffset = c;
        }
    }
}
