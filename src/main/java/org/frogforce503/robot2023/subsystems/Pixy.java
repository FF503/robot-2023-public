package org.frogforce503.robot2023.subsystems;

import java.util.ArrayList;

import org.frogforce503.robot2023.RobotState.GamePiece;

// import io.github.pseudoresonance.pixy2api.Pixy2;
// import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

// public class Pixy extends Subsystem {

//     private static Pixy instance = null;

//     private final double minBaseWidth = 200; // in Pixycam blocks

//     public static Pixy getInstance() {
//         if (instance == null) { instance = new Pixy(); }
//         return instance;
//     }

//     public enum ConeOrientation {
//         TOP_BACK, BASE_BACK, NONE
//     }

//     public Pixy2 pixycam;
//     boolean isCamera = false;
//     int state = -1;

//     public ArrayList<Block> blocks;
    
//     public Pixy() {
//         pixycam = Pixy2.createInstance(Pixy2.LinkType.SPI);
//         state = pixycam.init();
//         isCamera = state >= 0;
//     }

//     public boolean isCamera() {
//         return isCamera;
//     }

//     public void updateImage() {
//         try {
//             pixycam.getCCC().getBlocks( false , 255 , 255 ); //run getBlocks with arguments to have the camera acquire target data
//             blocks = pixycam.getCCC().getBlockCache();

//         } catch (Exception e) {
//             System.out.println("PixyCam not returning data. Check cable");

//         }
//     }

//     public GamePiece getObject() {
//         updateImage();

//         if(blocks.size() > 0) {
//             return (blocks.get(0).getSignature() ==1) ? GamePiece.CONE : GamePiece.CUBE;
//         }

//         return GamePiece.NONE;
//     }

//     public double getWidth() {
//         updateImage();

//         if (blocks.size() > 0) {
//             return blocks.get(0).getWidth();
//         }

//         return 0;
//     }

//     public ConeOrientation getConeOrientation() {
//         updateImage();

//         if(blocks.size() > 0 && getObject() == GamePiece.CONE) {
//             return (getWidth() > minBaseWidth) ? ConeOrientation.BASE_BACK : ConeOrientation.TOP_BACK;
//         }

//         return ConeOrientation.NONE;
//     }

//     @Override
//     public void outputTelemetry() {}

//     @Override
//     public void onStart() {}

//     @Override
//     public void onLoop() {}

//     @Override
//     public void onStop() {}
    
// }