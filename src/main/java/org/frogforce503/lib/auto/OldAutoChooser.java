// package org.frogforce503.lib.auto;

// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.HashMap;
// import java.util.List;

// import com.pathplanner.lib.PathPlannerTrajectory;

// import org.frogforce503.robot2023.RobotState;
// import org.frogforce503.robot2023.fields.FieldConfig;
// import org.frogforce503.robot2023.planners.LineupPlanner;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// public class OldAutoChooser {
//     private HashMap<StartingLocation, HashMap<Integer, AutoMode>> AUTO_MAP = new HashMap<>();

//     private final SendableChooser<StartingLocation> locationChooser = new SendableChooser<>();
//     private final SendableChooser<Integer> ballCountChooser = new SendableChooser<>();
//     private final SendableChooser<String> optionChooser = new SendableChooser<>();
//     private final Field2d autoPreviewField = new Field2d();

//     AutoMode selectedAuto;
//     String selectedAutoName = "NO AUTO SELECTED";
//     NetworkTable autoTable;

//     StartingLocation lastLocation = StartingLocation.LEFT;
//     int lastBallNumber = 2;
//     ShuffleboardLayout autoChooserLayout;

//     public void buildPaths() {
//         if (RobotState.getInstance().getAllianceColor() != FieldConfig.getInstance().getDefaultAlliance())
//             FieldConfig.getInstance().configureOppositeSide();
        
//         this.AUTO_MAP = new HashMap<StartingLocation, HashMap<Integer, AutoMode>>() {
//             {
//                 put(StartingLocation.LEFT, new HashMap<Integer, AutoMode>() {
//                     {
//                     }
//                 });
    
//                 put(StartingLocation.CENTER, new HashMap<Integer, AutoMode>() {
//                     {
//                     }
//                 });
    
//                 put(StartingLocation.RIGHT, new HashMap<Integer, AutoMode>() {
//                     {
//                     }
//                 });
//             }
//         };
//     }

//     public void addToShuffleboard(ShuffleboardTab tab) {
//         boolean firstDone = false;

//         for (StartingLocation location : StartingLocation.values()) {
//             if (!firstDone) {
//                 locationChooser.setDefaultOption(location.name(), location);
//                 firstDone = true;
//                 continue;
//             }
//             locationChooser.addOption(location.name(), location);
//         }

//         // populateBallChooser();

//         autoChooserLayout = tab.getLayout("Auton Chooser", BuiltInLayouts.kList).withSize(3, 5)
//                 .withPosition(0, 0);

//         autoTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tab.getTitle())
//                 .getSubTable("Auton Chooser");

//         autoChooserLayout.add("Starting Location", locationChooser).withSize(3, 1)
//                 .withPosition(1, 0)
//                 .withWidget(BuiltInWidgets.kSplitButtonChooser);

//         int lowestBallAmount = 2;
//         int highestBallAmount = 5;

//         firstDone = false;

//         // autoChooserLayout.getComponents().remove(ballCountChooser);

//         // ballCountChooser = new SendableChooser<>();

//         for (int i = lowestBallAmount; i <= highestBallAmount; i++) {
//             if (!firstDone) {
//                 ballCountChooser.setDefaultOption(i + " Ball" + (i == 1 ? "s" : ""), i);
//                 firstDone = true;
//                 continue;
//             }
//             ballCountChooser.addOption(i + " Ball", i);
//         }

//         optionChooser.addOption("A", "A");
//         optionChooser.addOption("B", "B");
//         optionChooser.addOption("C", "C");

//         autoChooserLayout.add("Number of Balls", ballCountChooser).withSize(3, 1)
//                 .withPosition(2, 0)
//                 .withWidget(BuiltInWidgets.kSplitButtonChooser);

//         autoChooserLayout.add("Choose Auton Option", optionChooser).withSize(3, 1)
//                 .withPosition(3, 0)
//                 .withWidget(BuiltInWidgets.kSplitButtonChooser);

//         autoChooserLayout.add("AutoPreviewField", autoPreviewField)
//                 .withPosition(3, 4);
//         autoChooserLayout.addBoolean("Commit Auton Config", () -> isCommitted())
//                 .withSize(1, 1)
//                 .withPosition(4, 0)
//                 .withWidget(BuiltInWidgets.kToggleButton);

//         autoChooserLayout.addBoolean("Ready to run??", () -> selectedAuto != null)
//                 .withSize(3, 1)
//                 .withPosition(5, 0)
//                 .withWidget(BuiltInWidgets.kBooleanBox);

//         autoChooserLayout.addString("Selected Path Name", () -> {
//             if (selectedAuto == null)
//                 return "INVALID AUTON";
//             return selectedAuto.getName();
//         }).withSize(10, 10).withPosition(4, 0);

//         reset();

//     }

//     // public void populateBallChooser() {
//     // // Object[] ballAmounts =
//     // // [AUTON_LIST.get(locationChooser.getSelected()).keySet().toArray();]
//     // int lowestBallAmount = 2;
//     // int highestBallAmount = 4;

//     // boolean firstDone = false;

//     // // autoChooserLayout.getComponents().remove(ballCountChooser);

//     // // ballCountChooser = new SendableChooser<>();

//     // for (int i = lowestBallAmount; i <= highestBallAmount; i++) {
//     // if (!firstDone) {
//     // ballCountChooser.setDefaultOption(i + " Ball" + (i == 1 ? "s" : ""), i);
//     // firstDone = true;
//     // continue;
//     // }
//     // ballCountChooser.addOption(i + " Ball", i);
//     // }

//     // autoChooserLayout.add(ballCountChooser).withSize(3, 1)
//     // .withPosition(2, 0)
//     // .withWidget(BuiltInWidgets.kSplitButtonChooser);
//     // }

//     public void periodic() {
//         if (locationChooser.getSelected() != lastLocation || ballCountChooser.getSelected() != lastBallNumber) {
//             reset();
//         }

//         if (autoTable.getEntry("Commit Auton Config").getBoolean(false)) {
//             System.out.println("COMMIT BUTTON PRESSED");
//             StartingLocation currLocation = locationChooser.getSelected();
//             int ballCount = ballCountChooser.getSelected();

//             AutoMode selected = AUTO_MAP.get(currLocation).get(ballCount);

//             // if (selected instanceof OptionBasedAuto) {
//             //     selected = ((OptionBasedAuto) selected).load(optionChooser.getSelected());
//             // }

//             // if (selected != null)
//             //     selected.setup();

//             this.selectedAuto = selected;
//             AutonomousExecutor.getInstance().setSelectedAutoMode(selected);

//             drawPathOnField();

//             autoTable.getEntry("Commit Auton Config").setBoolean(false);
//         }

//         lastLocation = locationChooser.getSelected();
//         lastBallNumber = ballCountChooser.getSelected();
//     }

//     private void drawPathOnField() {
//         if (this.selectedAuto != null && this.selectedAuto.getPath() != null) {
//             List<Pose2d> poses = new ArrayList<>();

//             for (PathPlannerTrajectory.State state : this.selectedAuto.getPath().getStates()) {
//                 poses.add(state.poseMeters);
//             }

//             autoPreviewField.getObject("preview_path").setPoses(poses);
//         }
//     }

//     private void reset() {
//         // autoTable.getSubTable("AutoPreviewField").getEntry("preview_path").setValue("");
//         autoPreviewField.getObject("preview_path")
//                 .setPoses(Arrays.asList(new Pose2d[] { new Pose2d(0, 0, new Rotation2d(0)),
//                         new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)) }));
//         selectedAuto = null;
//     }

//     private boolean isCommitted() {
//         return autoTable.getEntry("committed").getBoolean(false);
//     }

//     // public enum StartingLocation {
//     //     LEFT, CENTER, RIGHT
//     // }

//     public static LineupPlanner getInstance() {
//         return null;
//     }
// }