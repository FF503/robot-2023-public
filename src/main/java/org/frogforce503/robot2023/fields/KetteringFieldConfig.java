package org.frogforce503.robot2023.fields;

import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.fields.util.GameElementLocation;
import org.frogforce503.robot2023.fields.util.TagLocation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class KetteringFieldConfig extends FieldConfig {
/***
     * -------------------
     * OFFSET VARIABLE (CHANGE AT COMPETITION)
     * ALL MEASUREMENTS IN INCHES
     * -------------------
     */

    // BLUE ALLIANCE VALUES
    final double X_OriginToTag8 = 40.5; // 40.5
    final double Y_OriginToTag8 = 43; // 42.19
    final double Y_tag8Totag7 = 66; // 66
    final double Y_tag7Totag6 = 66; // 66
    final double X_OriginTotag3 = 40; // 40.45
    final double Y_OriginTotag3 = 42; // 42.19
    final double Y_tag2Totag1 = 66.5; // 66
    final double Y_tag3Totag2 = 67; // 65.5

    final double BLUE_X_tag6ToChargeCorner = 76; // 74.587
    final double BLUE_Y_tag6ToChargeCorner = -19.17; // -19.17
    final double BLUE_ChargeWidth = 75; // 75.926
    final double BLUE_ChargeLength = 97; // 94
    final double BLUE_X_ChargeBottomRightCornertoPieceD = 86.5; // 85
    final double BLUE_Y_ChargeBotomRightCornertoPieceD = 22.5; // 22.5
    final double BLUE_Y_PieceDtoPieceC = 47.5; // 49.75
    final double BLUE_Y_PieceCtoPieceB = 48; // 48
    final double BLUE_Y_PieceBtoPieceA = 47; // 48

    // RED ALLIANCE VALUES
    final double RED_X_tag1ToChargeCorner = 74; // 74.587
    final double RED_Y_tag1ToChargeCorner = -17.5; // -19.17
    final double RED_ChargeWidth = 75.5; // 75.926
    final double RED_ChargeLength = 97; // 94
    final double RED_X_ChargeBottomRightCornertoPieceD = 87; // 85
    final double RED_Y_ChargeBotomRightCornertoPieceD = 23; //v 85
    final double RED_Y_PieceDtoPieceC = 48; // 49.75
    final double RED_Y_PieceCtoPieceB = 48; // 48
    final double RED_Y_PieceBtoPieceA = 48; // 48

    public KetteringFieldConfig() {
        super(AllianceColor.BLUE);

        FIELD_DIMENSIONS = new Translation2d(16.525, 8.002);

        // boolean blue =

        /***
         * -------------------
         * APRILTAG LOCATIONS
         * -------------------
         */

        // BLUE SIDE, STARTING FROM SCORING TABLE SIDE
        TAG_8 = new TagLocation(8, Units.inchesToMeters(X_OriginToTag8), Units.inchesToMeters(Y_OriginToTag8));
        TAG_7 = TAG_8.setId(7).plusTag(new Translation2d(0, Units.inchesToMeters(Y_tag8Totag7)));
        TAG_6 = TAG_7.setId(6).plusTag(new Translation2d(0, Units.inchesToMeters(Y_tag7Totag6)));

        TAG_5 = TAG_6.setId(5)
                .plusTag(new Translation2d(Units.inchesToMeters(-26.19), Units.inchesToMeters(91.55))); // loading
                                                                                                                     // s0tation

        // RED SIDE, STARTING FROM SCORING TABLE SIDE
        TAG_3 = new TagLocation(3, Units.inchesToMeters(X_OriginTotag3), Units.inchesToMeters(Y_OriginTotag3));
        // TAG_1 = new TagLocation(1, Units.inchesToMeters(0), Units.inchesToMeters(0));
        TAG_2 = TAG_3.setId(2).plusTag(new Translation2d(0, Units.inchesToMeters(Y_tag3Totag2)));
        TAG_1 = TAG_2.setId(1).plusTag(new Translation2d(0, Units.inchesToMeters(Y_tag2Totag1)));

        TAG_4 = TAG_3.setId(4)
                .plusTag(new Translation2d(Units.inchesToMeters(-26.19), Units.inchesToMeters(-91.55))); // loading
                                                                                                               // station

        X_DISTANCE_FROM_TAG_TO_MID_JUNCTION = -Units.inchesToMeters(0);
        DISTANCE_BETWEEN_CELLS = Units.inchesToMeters(21);

        loadConstants();

        // RED SIDE, STARTING FROM SCORING TABLE SIDE
        // GAMEPIECE_1 = new GameElementLocation(1, Units.inchesToMeters(610.77),
        // Units.inchesToMeters(42.19));
        // GAMEPIECE_2 = GAMEPIECE_1.setId(2).plusTranslation(new Translation2d(0,
        // Units.inchesToMeters(66)));
        // GAMEPIECE_3 = GAMEPIECE_2.setId(3).plusTranslation(new Translation2d(0,
        // Units.inchesToMeters(66)));
        // GAMEPIECE_4 = GAMEPIECE_3.setId(4).plusTranslation(new Translation2d(0,
        // Units.inchesToMeters(91.55)));

    }

    @Override
    public void loadConstants() {
        boolean blue = RobotState.getInstance().getAllianceColor() == AllianceColor.BLUE;
        // Translation2d leftmostTag = blue ? TAG_6 : TAG_1;

        // should be identical but ok
        if (blue) {
            /***
             * ------------------------
             * CHARGE STATION LOCATIONS (FROM THE PERSPECTIVE OF THE DRIVER STATION)
             * ------------------------
             */

            CHARGE_OUTER_TOP_LEFT = TAG_6.plus(new Translation2d(Units.inchesToMeters(BLUE_X_tag6ToChargeCorner),
                    Units.inchesToMeters(BLUE_Y_tag6ToChargeCorner)));// ChargeCorner
            CHARGE_OUTER_TOP_RIGHT = CHARGE_OUTER_TOP_LEFT
                    .plus(new Translation2d(Units.inchesToMeters(BLUE_ChargeWidth), 0));
            CHARGE_OUTER_BOTTOM_RIGHT = CHARGE_OUTER_TOP_RIGHT
                    .plus(new Translation2d(0, Units.inchesToMeters(-BLUE_ChargeLength)));
            CHARGE_OUTER_BOTTOM_LEFT = CHARGE_OUTER_BOTTOM_RIGHT
                    .minus(CHARGE_OUTER_TOP_RIGHT.minus(CHARGE_OUTER_TOP_LEFT));

            // CHARGE_HP_SIDE_TAPE_EDGE = CHARGE_OUTER_TOP_LEFT.plus(new
            // Translation2d(Units.inchesToMeters(15.374), 0));
            // CHARGE_CABLE_PROTECTOR_EDGE = CHARGE_OUTER_BOTTOM_RIGHT.plus(new
            // Translation2d(-Units.inchesToMeters(37.731), 0));

            /***
             * -------------------
             * GAME ELEMENT LOCATIONS
             * -------------------
             */

            // BLUE SIDE, GOING FROM RIGHT TO LEFT FROM GRID
            GAMEPIECE_D = new GameElementLocation(8,
                    CHARGE_OUTER_BOTTOM_RIGHT.getX() + Units.inchesToMeters(BLUE_X_ChargeBottomRightCornertoPieceD),
                    CHARGE_OUTER_BOTTOM_RIGHT.getY() - Units.inchesToMeters(BLUE_Y_ChargeBotomRightCornertoPieceD));
            GAMEPIECE_C = GAMEPIECE_D.setId(7)
                    .plusTranslation(new Translation2d(0, Units.inchesToMeters(BLUE_Y_PieceDtoPieceC)));
            GAMEPIECE_B = GAMEPIECE_C.setId(6)
                    .plusTranslation(new Translation2d(0, Units.inchesToMeters(BLUE_Y_PieceCtoPieceB)));
            GAMEPIECE_A = GAMEPIECE_B.setId(5)
                    .plusTranslation(new Translation2d(0, Units.inchesToMeters(BLUE_Y_PieceBtoPieceA)));
        } else {
            /***
             * ------------------------
             * CHARGE STATION LOCATIONS (FROM THE PERSPECTIVE OF THE DRIVER STATION)
             * ------------------------
             */

            System.out.println("Configuring for red");

            CHARGE_OUTER_TOP_LEFT = TAG_1.plus(new Translation2d(Units.inchesToMeters(RED_X_tag1ToChargeCorner),
                    Units.inchesToMeters(RED_Y_tag1ToChargeCorner)));// ChargeCorner
            CHARGE_OUTER_TOP_RIGHT = CHARGE_OUTER_TOP_LEFT
                    .plus(new Translation2d(Units.inchesToMeters(RED_ChargeWidth), 0));
            CHARGE_OUTER_BOTTOM_RIGHT = CHARGE_OUTER_TOP_RIGHT
                    .plus(new Translation2d(0, Units.inchesToMeters(-RED_ChargeLength)));
            CHARGE_OUTER_BOTTOM_LEFT = CHARGE_OUTER_BOTTOM_RIGHT
                    .minus(CHARGE_OUTER_TOP_RIGHT.minus(CHARGE_OUTER_TOP_LEFT));

            // CHARGE_HP_SIDE_TAPE_EDGE = CHARGE_OUTER_TOP_LEFT.plus(new
            // Translation2d(Units.inchesToMeters(15.374), 0));
            // CHARGE_CABLE_PROTECTOR_EDGE = CHARGE_OUTER_BOTTOM_RIGHT.plus(new
            // Translation2d(-Units.inchesToMeters(37.731), 0));

            /***
             * -------------------
             * GAME ELEMENT LOCATIONS
             * -------------------
             */

            // BLUE SIDE, GOING FROM RIGHT TO LEFT FROM GRID
            GAMEPIECE_D = new GameElementLocation(8,
                    CHARGE_OUTER_BOTTOM_RIGHT.getX() + Units.inchesToMeters(RED_X_ChargeBottomRightCornertoPieceD),
                    CHARGE_OUTER_BOTTOM_RIGHT.getY() - Units.inchesToMeters(RED_Y_ChargeBotomRightCornertoPieceD));
            GAMEPIECE_C = GAMEPIECE_D.setId(7)
                    .plusTranslation(new Translation2d(0, Units.inchesToMeters(RED_Y_PieceDtoPieceC)));
            GAMEPIECE_B = GAMEPIECE_C.setId(6)
                    .plusTranslation(new Translation2d(0, Units.inchesToMeters(RED_Y_PieceCtoPieceB)));
            GAMEPIECE_A = GAMEPIECE_B.setId(5)
                    .plusTranslation(new Translation2d(0, Units.inchesToMeters(RED_Y_PieceBtoPieceA)));

        }

    }

    @Override
    public void configureOppositeSide() {
        // do not use this function
    }
}
