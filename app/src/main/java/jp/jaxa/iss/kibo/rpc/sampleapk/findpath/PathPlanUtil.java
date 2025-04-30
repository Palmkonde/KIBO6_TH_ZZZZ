package jp.jaxa.iss.kibo.rpc.sampleapk.findpath;

import android.util.Log;
import gov.nasa.arc.astrobee.types.Point;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Box;

import java.util.*;

/*
    Using Liang-Barsky algorithm to find the shortest path between two points in a grid.
 */

public class PathPlanUtil {
    private final String TAG = this.getClass().getSimpleName();

    private final List<Box> OASIS = Arrays.asList(
            new Box(10.425f, -10.2f, 4.445f, 11.425f, -9.5f, 4.945f),
            new Box(10.925f, -9.5f, 4.945f, 11.425f, -8.45f, 5.445f),
            new Box(10.425f, -8.45f, 4.945f, 10.975f, -7.4f, 5.445f),
            new Box(10.925f, -7.4f, 4.425f, 11.425f, -6.35f, 4.945f)
    );

    private final List<Box> KIZ = Arrays.asList(
            new Box(10.3f, -10.2f, 4.32f, 11.55f, -6.0f, 5.57f),
            new Box(9.5f, -10.5, 4.02, 10.5f, -9.6f, 4.8f)
    );

    /*
     * Plan a path from start→goal:
     *  - Stays entirely in the union of kizZones.
     *  - Maximizes travel through oasisZones.
     *  - Returns the minimal List<Point> of straight-line waypoints.
     *  - If goal is unreachable, returns as far along the line as possible.
     */

    public List<Point> planPath(
            Point start,
            Point goal
    ) {
        Point D = new Point(
                goal.getX() - start.getX(),
                goal.getY() - start.getY(),
                goal.getZ() - start.getZ()
        );

        // 1) Clip against KIZ
        List<double[]> kizIv = new ArrayList<double[]>();
        for(Box kiz : KIZ)  {
            double[] tt = liangBarsky3D(start, D, kiz);
            if(tt != null) {
                double t0 = clamp(tt[0], 0, 1);
                double t1 = clamp(tt[1], 0, 1);
                if(t0 <= t1) {
                    kizIv.add(new double[]{t0, t1});
                }
            }
        }
        List<double[]> kizMerged = mergeIntervals(kizIv);

        // 2) check reachability
        if(kizMerged.isEmpty() || kizMerged.get(0)[0] > 0.0) {
            return Collections.emptyList();
        }

        double reachableEnd = kizMerged.get(0)[1];
        for(int i=1; i < kizMerged.size(); i++) {
            if(kizMerged.get(i)[0] <= reachableEnd) {
                reachableEnd = Math.max(reachableEnd, kizMerged.get(i)[1]);
            } else {
                break;
            }
        }
        if( reachableEnd < 1.0) {
            // goal is unreachable
            return Arrays.asList(start, add(start, scale(D, reachableEnd)));
        }

        // 3) Clip against OASIS
        List<double[]> oasisIv = new ArrayList<double[]>();
        for(Box oasis : OASIS) {
            double[] tt = liangBarsky3D(start, D, oasis);
            if(tt != null) {
                double t0 = clamp(tt[0], 0, 1);
                double t1 = clamp(tt[1], 0, 1);
                if(t0 <= t1) {
                    oasisIv.add(new double[]{t0, t1});
                }
            }
        }
        List<double[]> oasisMerged = mergeIntervals(oasisIv);

        // 4) collect waypoints: 0, each oasis entry/exit, 1
        TreeSet<Double> ts = new TreeSet<Double>();
        ts.add(0.0);
        ts.add(1.0);
        for(double[] seg : oasisMerged) {
            ts.add(seg[0]);
            ts.add(seg[1]);
        }

        List<Point> waypoints = new ArrayList<Point>();
        for(Double t : ts) {
            waypoints.add(add(start, scale(D, t)));
        }

        return waypoints;
    }


    private static double[] liangBarsky3D(
            Point P0, Point D, Box B
    ) {
        double t0 = 0f, t1 = 1f;
        double[] p = {-D.getX(), D.getX(), -D.getY(), D.getY(), -D.getZ(), D.getZ()};
        double[] q = {
                P0.getX() - B.xMin, B.xMax - P0.getX(),
                P0.getY() - B.yMin, B.yMax - P0.getY(),
                P0.getZ() - B.zMin, B.zMax - P0.getZ()
        };

        for(int i = 0; i < 6; i++) {
            if(p[i] == 0) {
                if(q[i] < 0) {
                    return null; // parallel and outside
                }
            } else {
                double t = q[i] / p[i];
                if(p[i] < 0) {
                    if(t > t1) {
                        return null; // parallel and outside
                    } else if(t > t0) {
                        t0 = t;
                    }
                } else {
                    if(t < t0) {
                        return null; // parallel and outside
                    } else if(t < t1) {
                        t1 = t;
                    }
                }
            }
        }
        return new double[] { t0, t1 };
    }

    private static List<double[]> mergeIntervals(List<double[]> in) {
        Collections.sort(in, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });

        List<double[]> out = new ArrayList<>();
        for(double[] seg: in) {
            if(out.isEmpty() || seg[0] > out.get(out.size() - 1)[1]) {
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
