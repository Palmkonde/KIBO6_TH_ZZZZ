/**
 * YourService Program
 * A program to plan mission for Astrobee in 6th Kibo rpc.
 *
 * Author : Pandaree Somnueknaitham
 * Team : ZZZZ (Thailand)
 * Start date : 29 April 2025
 * Last update : 11 May 2025
 *
 * 1st Day 29/4 study built in class and method
 * 2nd Day 30/4 improve plane method
 * 3rd Day 1/5 improve zone method
 * 4th Day 2/5 improve corner point method
 * 5th Day 4/5 improve score point position method
 * 6th Day 11/5 comment in detail
 */

package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.core.*;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.Imgproc;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;
import android.widget.Switch;

import java.util.*;
import java.io.*;



public class YourService extends KiboRpcService {

    /** Declaring Variable (use values from rulebook) */
    protected final String TAG = this.getClass().getSimpleName(); // For message log

    // Start point's coordinates and Orientation
    protected static final Point startPosition = new Point(9.815d, -9.806d, 4.293d);
    protected static final Quaternion startOrientation = new Quaternion(1f, 0f,0f,0f);

    // Astronaut's coordinates and Orientation
    protected static final Point astronautPosition = new Point(11.143d, -6.7607d, 4.9654d);
    protected static final Quaternion astronautOrientation = new Quaternion(0f, 0f,0.707f,0.707f);

    // Oasis's coordinates
    protected static final Point oasis1MinXYZ =  new Point(10.425d, -10.2d,4.445d);
    protected static final Point oasis1MaxXYZ =  new Point(11.425d, -9.5d,4.945d);
    protected static final Point oasis2MinXYZ =  new Point(10.925d, -9.5d,4.945d);
    protected static final Point oasis2MaxXYZ =  new Point(11.425d, -8.45d,5.445d);
    protected static final Point oasis3MinXYZ =  new Point(10.425d, -8.45d,4.945d);
    protected static final Point oasis3MaxXYZ =  new Point(10.975d, -7.4d,5.445d);
    protected static final Point oasis4MinXYZ =  new Point(10.925d, -7.4d,4.425d);
    protected static final Point Oasis4MaxXYZ =  new Point(11.425d, -6.35d,4.945d);

    // KIZ's coordinates
    protected static final Point kiz1MinPoint = new Point(10.3d,-10.2d,4.32d);
    protected static final Point kiz1MaxPoint = new Point(11.55d,-6.0d,5.57);
    protected static final Point kiz2MinPoint = new Point(9.5d,-10.5d,4.02);
    protected static final Point kiz2MaxPoint = new Point(10.5d,-9.6d,4.8);
    protected static double xMinKIZ1 = kiz1MinPoint.getX();
    protected static double xMaxKIZ1 = kiz1MaxPoint.getX();
    protected static double yMinKIZ1 = kiz1MinPoint.getY();
    protected static double yMaxKIZ1 = kiz1MaxPoint.getY();
    protected static double zMinKIZ1 = kiz1MinPoint.getZ();
    protected static double zMaxKIZ1 = kiz1MaxPoint.getZ();
    protected static double xMinKIZ2 = kiz2MinPoint.getX();
    protected static double xMaxKIZ2 = kiz2MaxPoint.getX();
    protected static double yMinKIZ2 = kiz2MinPoint.getY();
    protected static double yMaxKIZ2 = kiz2MaxPoint.getY();
    protected static double zMinKIZ2 = kiz2MinPoint.getZ();
    protected static double zMaxKIZ2 = kiz2MaxPoint.getZ();

    // Area coordinates
    // 1
    protected static final Point area1MinXYZ = new Point(10.42, -10.58,4.82);
    protected static final Point area1MaxXYZ = new Point(11.48, -10.58, 5.57);
    // 2
    protected static final Point area2MinXYZ = new Point(10.3, -9.25,3.76203);
    protected static final Point area2MaxXYZ = new Point(11.55, -8.5, 3.76203);
    // 3
    protected static final Point area3MinXYZ = new Point(10.3, -8.4,3.76093);
    protected static final Point area3MaxXYZ = new Point(11.55, -7.45, 3.76093);
    // 4
    protected static final Point area4MinXYZ = new Point(9.866984, -7.34,4.32);
    protected static final Point area4MaxXYZ = new Point(9.866984, -6.365, 5.57);

    // Astrobee measures
    protected static double xAstrobee = 0.32d;
    protected static double yAstrobee = 0.32d;
    protected static double zAstrobee = 0.32d;

    // For image processing part
    protected String[] TEMPLATE_FILE_NAME = {"coin.png", "compass.png", "coral.png", "crystal.png", "diamond.png", "emerald.png", "fossil.png", "key.png", "letter.png", "shell.png", "treasure_box.png"};

    /** Method to detect plane from four corner points (for KIZ1) */
    // Input : 4 corner points
    // Output : Plane name
    protected static String getPlane(Point p1, Point p2, Point p3, Point p4) {
        String plane = "";

        if((p1.getX() == p2.getX() && p2.getX() == p3.getX() && p3.getX() == p4.getX()) ) {
            if( Math.abs(kiz1MinPoint.getX() - p1.getX()) < Math.abs(kiz1MaxPoint.getX() - p1.getX()) ) {
                plane = "-YZ"; // Dock side, X- side
            } else if (Math.abs(kiz1MinPoint.getX() - p1.getX()) > Math.abs(kiz1MaxPoint.getX() - p1.getX()) ) {
                plane = "YZ"; // Opposite Dock side, X+ side
            }
        }

        if((p1.getY() == p2.getY() && p2.getY() == p3.getY() && p3.getY() == p4.getY()) ) {
            if( Math.abs(kiz1MinPoint.getY() - p1.getY()) < Math.abs(kiz1MaxPoint.getY() - p1.getY()) ) {
                plane = "-XZ"; // P start side, Y- side
            } else if (Math.abs(kiz1MinPoint.getY() - p1.getY()) > Math.abs(kiz1MaxPoint.getY() - p1.getY()) ) {
                plane = "XZ"; // P astronaut side, Y+ side
            }
        }

        if((p1.getZ() == p2.getZ() && p2.getZ() == p3.getZ() && p3.getZ() == p4.getZ()) ) {
            if( Math.abs(kiz1MinPoint.getZ() - p1.getZ()) < Math.abs(kiz1MaxPoint.getZ() - p1.getZ()) ) {
                plane = "XY"; // Z- side
            } else if (Math.abs(kiz1MinPoint.getZ() - p1.getZ()) > Math.abs(kiz1MaxPoint.getZ() - p1.getZ()) ) {
                plane = "-XY"; // Z+ side
            }
        }
        return plane;
    }

    /** Method to detect plane from six point (for KIZ1)*/
    // Input : 6 coordinates from min max table in rulebook
    // Output : Plane name
    protected static String getPlane(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax){
        String plane = "";

        if(xMin == xMax ) {
            double x = xMin;
            if( (Math.abs(xMinKIZ1 - x)) < (Math.abs(xMaxKIZ1 - x)) ) {
                plane = "-YZ"; // Dock side, X- side
            } else if ((Math.abs(xMinKIZ1 - x)) > (Math.abs(xMaxKIZ1 - x)) ) {
                plane = "YZ"; // Opposite Dock side, X+ side
            }
        }

        if( yMin == yMax ) {
            double y = yMin;
            if( (Math.abs(yMinKIZ1 - y)) < (Math.abs(yMaxKIZ1 - y)) ) {
                plane = "-XZ"; // P start side, Y- side
            } else if ((Math.abs(yMinKIZ1 - y)) > (Math.abs(yMaxKIZ1 - y))) {
                plane = "XZ"; // P astronaut side, Y+ side
            }
        }

        if(zMin == zMax ) {
            double z = zMin;
            if( Math.abs(zMinKIZ1 - z) < Math.abs(zMaxKIZ1 - z) ) {
                plane = "XY"; // Z- side
            } else if (Math.abs(zMinKIZ1 - z) > Math.abs(zMaxKIZ1 - z) ) {
                plane = "-XY"; // Z+ side
            }
        }
        return plane;
    }

    /** Method to find quaternion for area from four corner points*/
    // Input : 4 corner points
    // Output : quaternion
    protected static Quaternion setQuaternion(Point p1, Point p2, Point p3, Point p4) {
        String plane = getPlane(p1, p2, p3, p4);
        Quaternion quaternion;

        switch (plane) {
            case "XY": quaternion = new Quaternion(0f, -0.707f, 0f, 0.707f); // Area2 and Area3
                break;
            case "-XY": quaternion = new Quaternion(0f, 0.707f, 0f, 0.707f);
                break;
            case "XZ": quaternion = astronautOrientation; // new Quaternion(0f, 0f, 0.707f, 0.707f);
                break;
            case "-XZ": quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f); // Area1
                break;
            case "YZ": quaternion = new Quaternion(1f, 0f, 0f, 0f);
                break;
            case "-YZ": quaternion = new Quaternion(0f, 0f, 1f, 0f); // Area4
                break;
            default: quaternion = null;
                break;
        }
        return quaternion;
    }

    /** Method to calculate distance between area and keep in zone*/
    // Input : 4 corner points
    // Output : distance between area and keep on zone
    protected static double getDistanceAreaAndKiz(Point a1,Point a2, Point a3, Point a4) {
        String plane = getPlane(a1,a2,a3,a4);
        double dAreaKiz;

        switch (plane) {
            case "XY": dAreaKiz = Math.abs(a1.getZ()-kiz1MinPoint.getZ()); // Area2 and Area3
                break;
            case "-XY": dAreaKiz = Math.abs(a1.getZ()-kiz1MaxPoint.getZ()); // Maybe not use because don't have area on -XY
                break;
            case "XZ": dAreaKiz = Math.abs(a1.getY()-kiz1MaxPoint.getY()); // Maybe not use because don't have area on XZ
                break;
            case "-XZ": dAreaKiz = Math.abs(a1.getY()-kiz1MinPoint.getY()); // Area1
                break;
            case "YZ": dAreaKiz = Math.abs(a1.getX()-kiz1MaxPoint.getX()); // Maybe not use because don't have area on YZ
                break;
            case "-YZ": dAreaKiz = Math.abs(a1.getX()-kiz1MinPoint.getX()); // Area4
                break;
            default: dAreaKiz = 0;
                break;
        }
        return dAreaKiz;
    }

    /** Method to check that destination point in KIZ1 or KIZ2*/
    // Input : (x,y,z)
    // Output : 1 or 2 (Number of Zone)
    protected static int getZoneNumber(double x, double y, double z){
        int currentZone = 0;

        if ( xMinKIZ1 < x && x < xMaxKIZ1 && yMinKIZ1 < y && y < yMaxKIZ1 && zMinKIZ1 < z && z < zMaxKIZ1) {
            currentZone = 1;
            return currentZone;
        }
        else if (xMinKIZ2 < x && x < xMaxKIZ2 && yMinKIZ2 < y && y < yMaxKIZ2 && zMinKIZ2 < z && z < zMaxKIZ2) {
            currentZone = 2;
            return currentZone;
        }
    return currentZone;
    }

    /** Method to check that destination point make astrobee still in keep in zone */
    // Input : (x,y,z)
    // Output : true (if point is in zone)
    //          false (if point not in zone)
    protected static boolean isPointInKIZ(double x, double y, double z){
        int currentZone = getZoneNumber(x,y,z);
        boolean result = false;
        switch (currentZone){
            case 1:
                if ( (x+(xAstrobee/2) > xMinKIZ1) && (x+(xAstrobee/2) < xMaxKIZ1) &&
                     (y-(yAstrobee/2) > yMinKIZ1) && (y+(yAstrobee/2) < yMaxKIZ1) &&
                     (z+(zAstrobee/2) > zMinKIZ1) && (z+(zAstrobee/2) < zMaxKIZ1) ){
                result = true;
                }
                break;
            case  2:
                if ( (x+(xAstrobee/2) > xMinKIZ2) && (x+(xAstrobee/2) < xMaxKIZ2) &&
                     (y-(yAstrobee/2) > yMinKIZ2) && (y+(yAstrobee/2) < yMaxKIZ2) &&
                     (z+(zAstrobee/2) > zMinKIZ2) && (z+(zAstrobee/2) < zMaxKIZ2) ){
                result = true;
                }
                break;
            default:
                break;
        }
        return  result;
    }

    /** Method to find what area point in*/
    // Input : point
    // Output : area name
    protected static String getArea(Point p){
        String area = "" ;
        if (p.getX() <= area1MaxXYZ.getX() && p.getX() >= area1MinXYZ.getX() &&
            p.getY() <= area1MaxXYZ.getY() && p.getY() >= area1MinXYZ.getY() &&
            p.getZ() <= area1MaxXYZ.getZ() && p.getZ() >= area1MinXYZ.getZ()) {
            area = "area1";
        } else if (p.getX() <= area2MaxXYZ.getX() && p.getX() >= area2MinXYZ.getX() &&
                   p.getY() <= area2MaxXYZ.getY() && p.getY() >= area2MinXYZ.getY() &&
                   p.getZ() <= area2MaxXYZ.getZ() && p.getZ() >= area2MinXYZ.getZ()) {
            area = "area2";
        } else if (p.getX() <= area3MaxXYZ.getX() && p.getX() >= area3MinXYZ.getX() &&
                   p.getY() <= area3MaxXYZ.getY() && p.getY() >= area3MinXYZ.getY() &&
                   p.getZ() <= area3MaxXYZ.getZ() && p.getZ() >= area3MinXYZ.getZ()) {
            area = "area3";
        } else if (p.getX() <= area4MaxXYZ.getX() && p.getX() >= area4MinXYZ.getX() &&
                   p.getY() <= area4MaxXYZ.getY() && p.getY() >= area4MinXYZ.getY() &&
                   p.getZ() <= area4MaxXYZ.getZ() && p.getZ() >= area4MinXYZ.getZ()) {
            area = "area4";
        }
        return area;
    }

    /** Method to calculate distance in scoring base (within 0.9 m or center must within 0.45 m from KIZ1)*/
    /** Hard code use with isAstrobeeInScorePosition (below)*/
    protected static double distanceBetweenPointAndArea(Point p, String area){
        double distance = 0;
        switch (area) {
            case "area1" :
                distance = Math.abs(p.getY() - area1MinXYZ.getY());
                break;
            case "area2" :
                distance = Math.abs(p.getZ() - area2MinXYZ.getZ());
                break;
            case "area3" :
                distance = Math.abs(p.getZ() - area3MinXYZ.getZ());
                break;
            case "area4" :
                distance = Math.abs(p.getX() - area4MinXYZ.getX());
                break;
        }
        return distance;
    }

    /** TO write in runPlane*/
    /** Method to find that Astrobee still in scoring base*/
//    protected static boolean isAstrobeeInScorePosition(Point p, String area){
//        boolean astrobeeIsInScorePos = false;
//        double distance = distanceBetweenPointAndArea(p, "area1");
//        // 0.9/2 = 0.45 center of Astrobee
//        if (distance <= 0.45) {
//            astrobeeIsInScorePos = true;
//        }
//        return astrobeeIsInScorePos;
//    }


    /** Method to find corner point */
    protected static Point[] getCornerPoint(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax){
        Point leftTopPoint, leftBottomPoint, rightTopPoint, rightBottomPoint ;
        Point[] cornerPoint = new Point[4];
        String plane = getPlane(xMin, yMin, zMin, xMax, yMax, zMax);
        double x,y,z;

        switch (plane) {
            case "XY":
            case "-XY":
                z = zMin; // zMin == zMax

                // It's can change when change orientation, only for easy to understand.
                leftTopPoint= new Point(xMin, yMin, z);
                leftBottomPoint = new Point(xMax, yMin, z);
                rightBottomPoint = new Point(xMax, yMax, z);
                rightTopPoint = new Point(xMin, yMax, z);

                cornerPoint[0] = leftTopPoint;
                cornerPoint[1] = leftBottomPoint;
                cornerPoint[2] = rightBottomPoint;
                cornerPoint[3] = rightTopPoint;

                break;

            case "XZ":
            case "-XZ":
                y = yMin; // yMin == yMax

                leftTopPoint= new Point(xMin, y, zMin);
                leftBottomPoint = new Point(xMin, y, zMax);
                rightBottomPoint = new Point(xMax, y, zMax);
                rightTopPoint = new Point(xMax, y, zMin);

                cornerPoint[0] = leftTopPoint;
                cornerPoint[1] = leftBottomPoint;
                cornerPoint[2] = rightBottomPoint;
                cornerPoint[3] = rightTopPoint;

                break;

            case "YZ":
            case "-YZ":
                x = xMin; // xMin == xMax

                leftTopPoint= new Point(x, yMin, zMin);
                leftBottomPoint = new Point(x, yMin, zMax);
                rightBottomPoint = new Point(x, yMax, zMax);
                rightTopPoint = new Point(x, yMax, zMin);

                cornerPoint[0] = leftTopPoint;
                cornerPoint[1] = leftBottomPoint;
                cornerPoint[2] = rightBottomPoint;
                cornerPoint[3] = rightTopPoint;
                break;

        }
        return cornerPoint;
    }


    // For image processing part (Incomplete)
    /** Method to find the distance between two points */
    protected static double calculateDistance (org.opencv.core.Point p1, org.opencv.core.Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;

        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2)); // Pythagorean theorem
    }

    /** Method to resize image */
    protected Mat resizeImg (Mat img, int width) {
        int height= (int) (img.rows()*((double) width / img.cols())); // Calculate new height while maintaining the original ratio.
        Mat resizedImg = new Mat(); // Create new Mat to store resized image.
        Imgproc.resize(img, resizedImg, new Size(width, height)); // Resize original image to new size.

        return resizedImg; // Return Resized image
    }

    /** Method to rotate image */
    protected Mat rotImg (Mat img, int angle) {
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2.0, img.rows() / 2.0); // Find center point of the image
        Mat rotateMat = Imgproc.getRotationMatrix2D(center, angle,1.0);
        Mat rotateImg = new Mat(); // Create new mat to store rotated image

        return rotateImg; // Return rotated image
    }

    protected static List<org.opencv.core.Point> removeDuplicates (List<org.opencv.core.Point> points) {
        double length = 10; // Width 10 px
        List<org.opencv.core.Point> filteredList = new ArrayList<>();

        for (org.opencv.core.Point point : points) {
            boolean isInclude = false;
            for (org.opencv.core.Point checkPoint : filteredList) {
                double distance = calculateDistance(point, checkPoint);

                if (distance <= length) {
                    isInclude = true;
                    break;
                }
            }

            if (!isInclude) {
                filteredList.add(point);
            }
        }
        return filteredList;
    }

    protected int getMaxIndex (int[] array) {
        int max = 0;
        int maxIndex  =0;

        // Find the index of the element with the largest value
        for (int i =0; i< array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    /** Main method */
    @Override
    protected void runPlan1(){

        /** Start mission */
        Log.i(TAG,"start mission");
        api.startMission(); // Undock Astrobee.

        Log.i(TAG,"move to oasis 1");
        Log.i(TAG,"move to area 1");
        double distance = distanceBetweenPointAndArea(api.getRobotKinematics().getPosition(), "area1");
        if (!(distance <= 0.45)) {
//            api.moveTo(,api.getRobotKinematics().getOrientation(), false);
        }

        Log.i(TAG,"move to oasis 2");
        Log.i(TAG,"move to area 2");

        Log.i(TAG,"move to area 3");
        Log.i(TAG,"move to oasis 3");

        Log.i(TAG,"move to oasis 4");
        Log.i(TAG,"move to area 4");

        Log.i(TAG,"move to astronaut");
        api.moveTo(astronautPosition,astronautOrientation,false);

        Log.i(TAG,"Report rounding completion"); // Report the rounding completion.
        api.reportRoundingCompletion();

        Log.i(TAG,"Astronaut is looking for "); // Notify astronaut when recognized it.
        api.notifyRecognitionItem();

        Log.i(TAG,"Successfully found target item."); // Take a snapshot of the target item.
        api.takeTargetItemSnapshot(); // When execute this method, the mission is completed.

        // Shutdown robot
        api.shutdownFactory();

        /** Move to area */
        api.getRobotKinematics();
        // Point(double x, double y, double z)
        // Quaternion(float x, float y, float z, float w)
        // moveTo(Point goalPoint, Quaternion orientation, boolean printRobotPosition)
        Point point = new Point(10.9d, -9.92284d, 5.195d);
        Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
//        api.moveTo(point, quaternion, false);
        Result resultMove = api.moveTo(point, quaternion, false);

        final int loopMax = 5;

        int loopCounter = 0;
        while (!resultMove.hasSucceeded() && loopCounter < loopMax) {
            // Retry
            resultMove = api.moveTo(point, quaternion, false);
            ++loopCounter;
        }


        // Get a camera image.
        Mat image = api.getMatNavCam(); // get mat image form nav cam -> store in image type mat -> image processing.
        api.saveMatImage(image, "fileName.png");

        /** Detect AR */
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250); // Specify the type of AR marker
        List<Mat> corners = new ArrayList<>(); // Create ArrayList to store angles of AR marker
        Mat markerIds = new Mat(); // Create Mat to store ID of AR marker
        Aruco.detectMarkers(image, dictionary, corners, markerIds); // Detect AR marker

        /** Get camera matrix */
        Mat cameraMatrix = new Mat(3,3,CvType.CV_64F); // Create Mat to store camera matrix
        cameraMatrix.put(0,0,api.getNavCamIntrinsics()[0]);

        /** Get lens distortion parameters */
        Mat cameraCoefficients = new Mat(1,5,CvType.CV_64F);
        cameraCoefficients.put(0,0,api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        /** Undistort image*/
        Mat undistortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);

        /** Pattern matching && Load template images */
        Mat[] templates = new Mat[TEMPLATE_FILE_NAME.length];
        for (int i = 0; i< TEMPLATE_FILE_NAME.length; i++) {
            try {
                // Open template image file in Bitmap and convert to Mat
                InputStream inputStream = getAssets().open(TEMPLATE_FILE_NAME[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);

                // Convert to grayscale
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                // Assign to an array of templates
                templates[i] = mat;
                inputStream.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        /** Number of matches for each template */
        int templateMatchCnt[] = new int[11];

        /** Get the number of template matches*/
        for (int tempNum = 0; tempNum < templates.length; tempNum++) {
            // Number of matches
            int matchCnt = 0;

            // Coordinates of the matched location
            List<org.opencv.core.Point> matches = new ArrayList<>();

            // Loading template image and target image
            Mat template = templates[tempNum].clone();
            Mat targetImg = undistortImg.clone();

            // Pattern matching
            int widthMin = 20; //[px]
            int widthMax = 100; //[px]
            int changeWidth = 5; //[px]
            int changeAngle = 45; //[degree]

            for (int i = widthMin; i <= widthMax; i+= changeWidth) {
                for  (int j = 0; j <= 360; j += changeAngle ) {
                    Mat resizedTemp = resizeImg(template, i);
                    Mat rotResizedTemp = rotImg(resizedTemp, j);

                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImg, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);

                    // Get coordinates with similarity grater than or equal to the threshold
                    double threshold = 0.8;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);
                    double maxVal = mmlr.maxVal;
                    if (maxVal >= threshold ) {
                        // Extract only results grater than or equal to the threshold
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                        // Get match counts
                        for (int y = 0; y< thresholdedResult.rows(); y++) {
                            for (int x = 0; x < thresholdedResult.cols(); x++) {
                                if (thresholdedResult.get(y, x)[0] > 0) {
                                    matches.add(new org.opencv.core.Point(x, y));
                                }
                            }
                        }
                    }
                }
            }

            // Avoid detecting the same location multiple times
            List<org.opencv.core.Point> filteredMatches = removeDuplicates(matches);
            matchCnt += filteredMatches.size();

            // Number of matches for each template
            templateMatchCnt[tempNum] = matchCnt;
        }

        // When you recognize landmark items, let’s set the type and number.
        int mostMatchTemplateNum = getMaxIndex(templateMatchCnt);

        api.setAreaInfo(1,TEMPLATE_FILE_NAME[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);
        api.moveTo(oasis2MinXYZ,startOrientation, true);
        api.moveTo(oasis2MaxXYZ,startOrientation,true);

        api.setAreaInfo(2,TEMPLATE_FILE_NAME[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);

        // setAreaInfo(int areaId, String itemName, int number)
        api.setAreaInfo(1,"item_name");
        api.setAreaInfo(1, "item_name", 1);
        /* **************************************************** */
        /* Let's move to each area and recognize the items. */
        api.setAreaInfo(2, "item_name", 1);
        api.setAreaInfo(3, "item_name", 1);
        api.setAreaInfo(4, "item_name", 1);
        /* **************************************************** */


    }

}
