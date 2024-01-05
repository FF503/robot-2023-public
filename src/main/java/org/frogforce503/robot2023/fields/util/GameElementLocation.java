package org.frogforce503.robot2023.fields.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GameElementLocation extends Translation2d {
    private int id;
    private boolean rotate180;

    public GameElementLocation (int id, double x, double y){
        super(x,y);
        setId(id);
    }
    public GameElementLocation(int id, double d, Rotation2d angle) {
        super(d, angle);
        setId(id);
    }

    public GameElementLocation(int id, Translation2d translation) {
        this(id, translation.getX() , translation.getY());
    }
    
    public GameElementLocation plusTranslation(Translation2d other) {
        Translation2d res =  ((Translation2d) this).plus((Translation2d) other);
        return new GameElementLocation(this.id, res);
    }

    public GameElementLocation setId(int id) {
        this.id = Math.max(Math.min(id, 8), 1);
        this.rotate180 = this.id <= 4;
        return this;
    }
    
}
