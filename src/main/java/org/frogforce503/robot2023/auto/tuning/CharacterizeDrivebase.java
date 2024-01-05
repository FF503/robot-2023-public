package org.frogforce503.robot2023.auto.tuning;

import java.util.LinkedList;
import java.util.List;

import org.frogforce503.lib.auto.AutoMode;
import org.frogforce503.lib.auto.actions.base.Action;
import org.frogforce503.lib.auto.actions.base.InlineAction;
import org.frogforce503.lib.auto.actions.swerve.SwerveCharacterizationAction;
import org.frogforce503.lib.auto.follower.RoutineBuilder;
import org.frogforce503.lib.math.PolynomialRegression;
import org.frogforce503.lib.trajectory.PlannedPath;
import org.frogforce503.robot2023.Robot;
import org.frogforce503.robot2023.RobotHardware;
import org.frogforce503.robot2023.RobotState;
import org.frogforce503.robot2023.subsystems.swerve.Swerve;
import org.frogforce503.robot2023.subsystems.swerve.Swerve.SwerveControlState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;

public class CharacterizeDrivebase extends AutoMode {

    SwerveCharacterizationAction characterizationAction;
    SwerveCharacterizationAction.FeedForwardCharacterizationData characterizationData;

    public CharacterizeDrivebase() {
      characterizationData = new SwerveCharacterizationAction.FeedForwardCharacterizationData(RobotState.getInstance().getCurrentRobot().name());
      characterizationAction = new SwerveCharacterizationAction(true, characterizationData, Swerve.getInstance()::setCharacterizationDriveVoltage, Swerve.getInstance()::getIndividualDriveModuleVelocity);
    }
   
    @Override
    public Action routine() {
        return new RoutineBuilder(characterizationAction)
                    .first(new InlineAction(() -> Swerve.getInstance().setControlState(SwerveControlState.CHARACTERIZATION)))
                    // .addInline(() -> characterizationData.print())
                    .build();
    }

    @Override
    public PlannedPath getPath() {
        return null; 
    }   
}
