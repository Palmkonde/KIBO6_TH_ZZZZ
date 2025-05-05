package jp.jaxa.iss.kibo.rpc.sampleapk.element;

import gov.nasa.arc.astrobee.types.Point;

import java.util.ArrayList;
import java.util.List;

public class SpatialIndex {
    private final List<Point> points = new ArrayList<>();

    public void insert(Point point) {
        points.add(point);
    }

    public List<Point> query(Point center, double radius) {
        List<Point> result = new ArrayList<>();
        double radiusSquared = radius * radius;
        for(Point point : points) {
            double dx = point.getX() - center.getX();
            double dy = point.getY() - center.getY();
            double dz = point.getZ() - center.getZ();
            if(dx * dx + dy * dy + dz * dz <= radiusSquared) {
                result.add(point);
            }
        }

        return result;
    }
}
