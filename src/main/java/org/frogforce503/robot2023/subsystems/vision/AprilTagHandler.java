package org.frogforce503.robot2023.subsystems.vision;

import org.frogforce503.robot2023.subsystems.vision.cameras.Camera;
import org.frogforce503.robot2023.subsystems.vision.cameras.Limelight;
import org.frogforce503.robot2023.subsystems.vision.cameras.Photon;
import org.opencv.photo.Photo;

// does same as Limelight and OrangePI but just uses whichever one we actually want
public class AprilTagHandler {
    public static DetectionMethod detectionMethod = DetectionMethod.PHOTONVISION;

    // getInstance might be a misnomer because we never ACTUALLY return an instance of this class, but it is simpler this way
    public static Camera.AprilTagCamera getInstance() {
        return Photon.getInstance();
        // if (detectionMethod == DetectionMethod.PHOTONVISION) {
        //     return Photon.getInstance();
        // }
        // return Limelight.getInstance();
    }

    
    
    public enum DetectionMethod {
        LIMELIGHT, PHOTONVISION;
    }
}
