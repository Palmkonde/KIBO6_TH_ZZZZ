package jp.jaxa.iss.kibo.rpc.sampleapk.imageProcessing;

import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Last update : 13 May 2025 (Neptune)
 * Rotate class for setting Quaternion rotations based on plane.
 *
 * How to use:
 * import jp.jaxa.iss.kibo.rpc.sampleapk.ImageProcessing.Rotate; //อย่าลืม import ก่อนนะจ๊ะ
 * Rotate.setQuaternion("Write your plane");
 *
 * Plane (case-sensitive):
 * "area1" Rotation for Area 1.
 * "area2", "area3" Rotation for Area 2 and Area 3.
 * "area4" Rotation for Area 4.
 * "astronaut" Rotation for the astronaut view.
 * "oppositeArea23" Opposite rotation of Area 2 and Area 3.
 * "oppositeArea4" Opposite rotation of Area 4.
 *
 * If you don't want to change the rotation, just use an empty name: ""
 *
 * e.g.
 * Quaternion area1Rotation = Rotate.setQuaternion("area1");
 * api.move(point, Rotate.setQuaternion("area1"));
 * api.move(point, Rotate.setQuaternion(""));    // Keeps the current quaternion
 */

public class Rotate {

    private static Quaternion defaultQuaternion;
    public static final Quaternion astronautOrientation = new Quaternion(0f, 0f,0.707f,0.707f);
    public static final Quaternion startOrientation = new Quaternion(1f, 0f,0f,0f);

    public static void setDefaultQuaternion(Quaternion quaternion) {
        defaultQuaternion = quaternion;
    }

    public static Quaternion setQuaternion(String plane) {
        if (plane == null || plane.isEmpty()) {
            return defaultQuaternion;
        }

        Quaternion quaternion = defaultQuaternion;

        switch (plane) {
            case "area1":
                quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
                setDefaultQuaternion(quaternion);
                break;

            case "area2":
            case "area3":
                quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
                setDefaultQuaternion(quaternion);
                break;

            case "area4":
                quaternion = new Quaternion(0f, 0f, 1f, 0f);
                setDefaultQuaternion(quaternion);
                break;

            case "astronaut":
            case "oppositeArea1":
                quaternion = Constants.astronautOrientation;
                setDefaultQuaternion(quaternion);
                break;

            case "start":
            case "oppositeArea4":
                quaternion = Constants.startOrientation; // (1f,0f,0f,0f)
                setDefaultQuaternion(quaternion);
                break;

            case "oppositeArea23":
                quaternion = new Quaternion(0f, -0.707f, 0f, 0.707f);
                setDefaultQuaternion(quaternion);
                break;

            default:
                break;
        }
        return quaternion;
    }
}