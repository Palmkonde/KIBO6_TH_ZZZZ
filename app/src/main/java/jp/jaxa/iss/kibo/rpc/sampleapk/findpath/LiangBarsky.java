package jp.jaxa.iss.kibo.rpc.sampleapk.findpath;

import gov.nasa.arc.astrobee.types.Point;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Box;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class LiangBarsky {

    public static double[] liangBarsky3D(Point P0, Point D, Box B) {
        double t0 = 0f, t1 = 1f;
        double[] p = {-D.getX(), D.getX(), -D.getY(), D.getY(), -D.getZ(), D.getZ()};
        double[] q = {
                P0.getX() - B.xMin, B.xMax - P0.getX(),
                P0.getY() - B.yMin, B.yMax - P0.getY(),
                P0.getZ() - B.zMin, B.zMax - P0.getZ()
        };

        for (int i = 0; i < 6; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) {
                    return null; // parallel and outside
                }
            } else {
                double t = q[i] / p[i];
                if (p[i] < 0) {
                    if (t > t1) {
                        return null; // parallel and outside
                    } else if (t > t0) {
                        t0 = t;
                    }
                } else {
                    if (t < t0) {
                        return null; // parallel and outside
                    } else if (t < t1) {
                        t1 = t;
                    }
                }
            }
        }
        return new double[]{t0, t1};
    }

    public static List<double[]> mergeIntervals(List<double[]> in) {
        Collections.sort(in, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });

        List<double[]> out = new ArrayList<>();
        for (double[] seg : in) {
            if (out.isEmpty() || seg[0] > out.get(out.size() - 1)[1]) {
                out.add(new double[]{seg[0], seg[1]});
            } else {
                out.get(out.size() - 1)[1] = Math.max(out.get(out.size() - 1)[1], seg[1]);
            }
        }

        return out;
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (Math.min(v, hi));
    }
    private static Point add(Point a, Point b) {
        return new Point(a.getX() + b.getX(), a.getY() + b.getY(), a.getZ() + b.getZ());
    }
    private static Point scale(Point a, double s) {
        return new Point(a.getX() * s, a.getY() * s, a.getZ() * s);
    }
}
