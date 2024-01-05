package org.frogforce503.robot2023.auto;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.auto.actions.base.InlineAction;
import org.frogforce503.lib.auto.follower.RoutineBuilder;
import org.frogforce503.lib.trajectory.PlannedPath;

public class NullAuto extends AutoMode {

    @Override
    public Action routine() {
        return new RoutineBuilder(new InlineAction(() -> System.out.println("WARNING: NULL AUTO RUNNING"))).build();
    }

    @Override
    public PlannedPath getPath() {
        return null; 
    }
}
