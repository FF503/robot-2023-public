package org.frogforce503.lib.auto;

import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.RobotState.AllianceColor;

@Deprecated(since = "2023", forRemoval = true)
public class AllianceSpecificAuto extends AutoMode {

    final AutoMode RED_AUTO;
    final AutoMode BLUE_AUTO;

    public AllianceSpecificAuto(AutoMode red, AutoMode blue) {
        this.RED_AUTO = red;
        this.BLUE_AUTO = blue;
    }

    private AutoMode getCorrectAutoMode() {
        return (RobotState.getInstance().getAllianceColor() == AllianceColor.RED ? this.RED_AUTO : this.BLUE_AUTO);
    }

    @Override
    public Action routine() {
        return getCorrectAutoMode().routine();
    }

    @Override
    public PlannedPath getPath() {
        return getCorrectAutoMode().getPath();
    }
    
}
