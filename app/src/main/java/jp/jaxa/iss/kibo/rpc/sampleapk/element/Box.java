package jp.jaxa.iss.kibo.rpc.sampleapk.element;

import gov.nasa.arc.astrobee.types.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Box {
    public double xMin, yMin, zMin;
    public double xMax, yMax, zMax;

    private final double STEP = 0.1f;

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

    public List<Point> getNodes() {
        List<Point> nodes = new ArrayList<>();
        for(double x= xMin + STEP ; x < xMax; x += STEP) {
            for(double y = yMin + STEP; y < yMax; y += STEP) {
                for(double z = zMin + STEP; z < zMax; z += STEP) {
                   nodes.add(new Point(x, y, z));
                }
            }
        }

        return nodes;
    }
}
