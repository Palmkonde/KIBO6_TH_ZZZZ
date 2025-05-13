package jp.jaxa.iss.kibo.rpc.sampleapk.imageProcessing;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;


public class Constants {

    // Start point's coordinates and Orientation
    public static final Point startPosition = new Point(9.815d, -9.806d, 4.293d);
    public static final Quaternion startOrientation = new Quaternion(1f, 0f,0f,0f);

    // Astronaut's coordinates and Orientation
    public static final Point astronautPosition = new Point(11.143d, -6.7607d, 4.9654d);
    public static final Quaternion astronautOrientation = new Quaternion(0f, 0f,0.707f,0.707f);

    // Oasis's coordinates
    public static final Point oasis1MinXYZ =  new Point(10.425d, -10.2d,4.445d);
    public static final Point oasis1MaxXYZ =  new Point(11.425d, -9.5d,4.945d);
    public static final Point oasis2MinXYZ =  new Point(10.925d, -9.5d,4.945d);
    public static final Point oasis2MaxXYZ =  new Point(11.425d, -8.45d,5.445d);
    public static final Point oasis3MinXYZ =  new Point(10.425d, -8.45d,4.945d);
    public static final Point oasis3MaxXYZ =  new Point(10.975d, -7.4d,5.445d);
    public static final Point oasis4MinXYZ =  new Point(10.925d, -7.4d,4.425d);
    public static final Point Oasis4MaxXYZ =  new Point(11.425d, -6.35d,4.945d);

    // KIZ's coordinates
    public static final Point kiz1MinPoint = new Point(10.3d,-10.2d,4.32d);
    public static final Point kiz1MaxPoint = new Point(11.55d,-6.0d,5.57d);
    public static final Point kiz2MinPoint = new Point(9.5d,-10.5d,4.02d);
    public static final Point kiz2MaxPoint = new Point(10.5d,-9.6d,4.8d);
    public static double xMinKIZ1 = kiz1MinPoint.getX();
    public static double xMaxKIZ1 = kiz1MaxPoint.getX();
    public static double yMinKIZ1 = kiz1MinPoint.getY();
    public static double yMaxKIZ1 = kiz1MaxPoint.getY();
    public static double zMinKIZ1 = kiz1MinPoint.getZ();
    public static double zMaxKIZ1 = kiz1MaxPoint.getZ();
    public static double xMinKIZ2 = kiz2MinPoint.getX();
    public static double xMaxKIZ2 = kiz2MaxPoint.getX();
    public static double yMinKIZ2 = kiz2MinPoint.getY();
    public static double yMaxKIZ2 = kiz2MaxPoint.getY();
    public static double zMinKIZ2 = kiz2MinPoint.getZ();
    public static double zMaxKIZ2 = kiz2MaxPoint.getZ();

    // Area coordinates
    // 1
    public static final Point area1MinXYZ = new Point(10.42d, -10.58d,4.82d);
    public static final Point area1MaxXYZ = new Point(11.48d, -10.58d, 5.57d);
    // 2
    public static final Point area2MinXYZ = new Point(10.3d, -9.25d,3.76203d);
    public static final Point area2MaxXYZ = new Point(11.55d, -8.5d, 3.76203d);
    // 3
    public static final Point area3MinXYZ = new Point(10.3d, -8.4d,3.76093d);
    public static final Point area3MaxXYZ = new Point(11.55d, -7.45d, 3.76093d);
    // 4
    public static final Point area4MinXYZ = new Point(9.866984d, -7.34d,4.32d);
    public static final Point area4MaxXYZ = new Point(9.866984d, -6.365d, 5.57d);

    // Astrobee measures
    public static double xAstrobee = 0.32d;
    public static double yAstrobee = 0.32d;
    public static double zAstrobee = 0.32d;

}
