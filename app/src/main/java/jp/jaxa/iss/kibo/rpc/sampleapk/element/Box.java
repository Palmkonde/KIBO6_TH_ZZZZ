package jp.jaxa.iss.kibo.rpc.sampleapk.element;

import gov.nasa.arc.astrobee.types.Point;

import java.util.Arrays;
import java.util.List;

public class Box {
    public double xMin, yMin, zMin;
    public double xMax, yMax, zMax;

    public Box(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax) {
        this.xMin = xMin;
        this.yMin = yMin;
        this.zMin = zMin;
        this.xMax = xMax;
        this.yMax = yMax;
        this.zMax = zMax;
    }

    public List<Point> getVertices() {
        return Arrays.asList(
                new Point(xMin, yMin, zMin),
                new Point(xMax, yMin, zMin),
                new Point(xMin, yMax, zMin),
                new Point(xMax, yMax, zMin),
                new Point(xMin, yMin, zMax),
                new Point(xMax, yMin, zMax),
                new Point(xMin, yMax, zMax),
                new Point(xMax, yMax, zMax)
        );
    }
}
