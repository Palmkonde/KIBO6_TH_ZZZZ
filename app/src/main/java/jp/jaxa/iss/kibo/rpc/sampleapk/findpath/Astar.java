package jp.jaxa.iss.kibo.rpc.sampleapk.findpath;

import android.util.Log;
import gov.nasa.arc.astrobee.types.Point;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Box;

import java.util.*;

/*
    Using Liang-Barsky algorithm to find the shortest path between two points in a grid.
 */

public class Astar {
    private final String TAG = this.getClass().getSimpleName();

    private final List<Box> OASIS = Arrays.asList(
            new Box(10.425f, -10.2f, 4.445f, 11.425f, -9.5f, 4.945f),
            new Box(10.925f, -9.5f, 4.945f, 11.425f, -8.45f, 5.445f),
            new Box(10.425f, -8.45f, 4.945f, 10.975f, -7.4f, 5.445f),
            new Box(10.925f, -7.4f, 4.425f, 11.425f, -6.35f, 4.945f)
    );

    private double[] liangBarsky3D(Point P0, Point D, Box B) {
        double t0 = 0.0, t1 = 1;
        double[] p = {
                -D.getX(), D.getX(),
                -D.getY(), D.getY(),
                -D.getZ(), D.getZ()
        };

        double[] q = {
                P0.getX() - B.xMin, B.xMax - P0.getX(),
                P0.getY() - B.yMin, B.yMax - P0.getY(),
                P0.getZ() - B.zMin, B.zMax - P0.getZ()
        };

        for (int i = 0; i < 6; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) {
                    return null; // No intersection
                }
            } else {
                double t = q[i] / p[i];
                if (p[i] < 0) {
                    if (t > t1) {
                        return null; // No intersection
                    } else if (t > t0) {
                        t0 = t;
                    }
                } else {
                    if (t < t0) {
                        return null; // No intersection
                    } else if (t < t1) {
                        t1 = t;
                    }
                }
            }
        }

        return new double[]{t0, t1};
    }

    public List<Point> findPath(Point start, Point end) {
        Log.i(TAG, "Start finding path from " + start + " to " + end);

        Point D = new Point(
                end.getX() - start.getX(),
                end.getY() - start.getY(),
                end.getZ() - start.getZ()
        );

        List<double[]> iv = new ArrayList<>();
        for(Box box : OASIS) {
            double[] t = liangBarsky3D(start, D, box);
            if(t != null) {
                double e = Math.max(0, Math.min(1, t[0]));
                double l = Math.max(0, Math.min(1, t[1]));
                if(e <= l) {
                    iv.add(new double[]{e, l});
                }
            }
        }

        // merge
        Collections.sort(iv, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });

        List<double[]> merged = new ArrayList<>();
        for(double[] interval : iv) {
            if(merged.isEmpty() || merged.get(merged.size() - 1)[1] < interval[0]) {
                merged.add(interval.clone());
            } else {
                merged.get(merged.size() - 1)[1] = Math.max(merged.get(merged.size() - 1)[1], interval[1]);
            }
        }

        TreeSet<Double> ts = new TreeSet<>();
        ts.add(0.0); ts.add(1.0);
        for(double[] interval : merged) {
            ts.add(interval[0]);
            ts.add(interval[1]);
        }

        // create Points
        List<Point> path = new ArrayList<>();
        for(double t : ts) {
            path.add(new Point(
                    start.getX() + t * D.getX(),
                    start.getY() + t * D.getY(),
                    start.getZ() + t * D.getZ()
            ));
        }

        return path;
    }
}
