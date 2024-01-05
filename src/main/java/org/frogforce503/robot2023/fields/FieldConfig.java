package org.frogforce503.robot2023.fields;

import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.RobotState.AllianceColor;
import org.frogforce503.robot2023.fields.util.GameElementLocation;
import org.frogforce503.robot2023.fields.util.TagLocation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public abstract class FieldConfig {

    private final AllianceColor DEFAULT_COLOR;

    public FieldConfig(AllianceColor defaultColor) {
       this.DEFAULT_COLOR = defaultColor; 
    }
    
    // CONSTANTS
    public Translation2d FIELD_DIMENSIONS;

    // TAG LOCATIONS
    public TagLocation TAG_1, TAG_2, TAG_3, TAG_4, TAG_5, TAG_6, TAG_7, TAG_8;
    public double DISTANCE_BETWEEN_CELLS, X_DISTANCE_FROM_TAG_TO_MID_JUNCTION;

    //GAME ELEMENT LOCATIONS
    public GameElementLocation GAMEPIECE_A, GAMEPIECE_B, GAMEPIECE_C, GAMEPIECE_D; 

    // CHARGE STATION
    public Translation2d CHARGE_OUTER_TOP_LEFT, CHARGE_OUTER_TOP_RIGHT, CHARGE_OUTER_BOTTOM_RIGHT, CHARGE_OUTER_BOTTOM_LEFT;
    public Translation2d CHARGE_HP_SIDE_TAPE_EDGE, CHARGE_CABLE_EDGE, CHARGE_CABLE_PROTECTOR_EDGE; // THESE 3 ARE NEVER USED, NO POINT IN CONFIGURING FOR MILFORD

    private static FieldConfig instance;

    public abstract void loadConstants();

    // override if field is measured otherwise
    public Translation2d getFieldDimensions() {
        return new Translation2d(Units.feetToMeters(54), Units.feetToMeters(27));
    }
    
    public static void setVenue(VENUE venue) {
        switch (venue) {
            case SHOP: {
                instance = new ShopFieldConfig();
                break;
            }
            case MILFORD: {
                instance = new MilfordFieldConfig();
                break;
            }
            case DETROIT: {
                instance = new DetroitFieldConfig();
                break;
            }
            case LAKEVIEW: {
                instance = new LakeviewFieldConfig();
                break;
            }
            case STATES: {
                instance = new StatesFieldConfig();
                break;
            }
            case NEWTON: {
                instance = new NewtonFieldConfig();
                break;
            }
            case BIG_BANG: {
                instance = new BigBangFieldConfig();
                break;
            }
            case RAINBOW: {
                instance = new RainbowFieldConfig();
                break;
            }
            case MARC: {
                instance = new MarcFieldConfig();
                break;
            }
            case KETTERING: {
                instance = new KetteringFieldConfig();
                break;
            }
            default: {
                System.out.println("WARNING: USING DEFAULT SHOP FIELD CONFIG");
                instance = new ShopFieldConfig();
            }
        }
    }

    // points defined in this function will still be defined in the field coordinate system of BLUE, but they will be different relatively speaking
    public abstract void configureOppositeSide();

    public AllianceColor getDefaultAlliance() {
        return this.DEFAULT_COLOR;
    }

    public TagLocation getTagByGrid(int gridNum) {
        int grid = MathUtil.clamp(gridNum, 1, 3);
        boolean blue = RobotState.getInstance().getAllianceColor() == AllianceColor.BLUE;
        
        System.out.println("FMS INFO AVAILABLE: " + RobotState.getInstance().isFMSInfoAvailable());
        System.out.println("CURRENT ALLIANCE COLOR " + RobotState.getInstance().getAllianceColor());
        
        if (grid == 1)
            return blue ? TAG_8 : TAG_3;
        else if (grid == 2)
            return blue ? TAG_7 : TAG_2;
        return blue ? TAG_6 : TAG_1;
    }

    public static boolean isMyAlliance(int tagID) {
        boolean blue = RobotState.getInstance().getAllianceColor() == AllianceColor.BLUE;

        if (blue) {
            return tagID == 6 || tagID == 7 || tagID == 8;
        } else {
            return tagID == 1 || tagID == 2 || tagID == 3;
        }
    }

    // public boolean isA

    // FIXME: USE A MAP THIS IS SO STUPID LIKE THIS
    public TagLocation getTagById(int id) {
        if (id == 1)
            return TAG_1;
        if (id == 2)
            return TAG_2;
        if (id == 3)
            return TAG_3;
        if (id == 4)
            return TAG_4;
        if (id == 5)
            return TAG_5;
        if (id == 6)
            return TAG_6;
        if (id == 7)
            return TAG_7;
        if (id == 8)
            return TAG_8;
        return null;
    }

    // public final void invert() 
    //     this.setRedAlliance();

    //     // for (Field field : this.getClass().getFields()) {
    //     //     if (field.getType().getSimpleName().equals("Translation2d")) {
    //     //         try {
    //     //             Translation2d position = (Translation2d) field.get(this);
    //     //             if (position != null) {
    //     //                 field.set(this, getFieldDimensions().minus(position));
    //     //             }
    //     //         } catch (IllegalArgumentException | IllegalAccessException e) {
    //     //             e.printStackTrace();
    //     //         }
    //     //     }
    //     // }
    // }

    public static FieldConfig getInstance() {
        if (instance == null)
            FieldConfig.setVenue(VENUE.SHOP);
        return instance;
    }
    

    public enum VENUE {
        SHOP, MILFORD, DETROIT, LAKEVIEW, STATES, NEWTON, BIG_BANG, RAINBOW, MARC, KETTERING
    }
}
