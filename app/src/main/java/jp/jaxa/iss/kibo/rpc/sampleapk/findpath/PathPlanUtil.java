package jp.jaxa.iss.kibo.rpc.sampleapk.findpath;

import gov.nasa.arc.astrobee.types.Point;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Box;

import java.util.*;

/*
    Using Liang-Barsky algorithm to find the shortest path between two points in a grid.
 */

public class PathPlanUtil {
    private final String TAG = this.getClass().getSimpleName();
    private final double OFFSET = 0.2f;

    private final List<Box> OASIS = Arrays.asList(
            new Box(10.425f + OFFSET, -10.2f + OFFSET, 4.445f + OFFSET, 11.425f - OFFSET, -9.5f - OFFSET, 4.945f - OFFSET),
            new Box(10.925f + OFFSET, -9.5f + OFFSET, 4.945f + OFFSET, 11.425f - OFFSET, -8.45f - OFFSET, 5.445f - OFFSET),
            new Box(10.425f + OFFSET, -8.45f + OFFSET, 4.945f + OFFSET, 10.975f - OFFSET, -7.4f - OFFSET, 5.445f - OFFSET),
            new Box(10.925f + OFFSET, -7.4f + OFFSET, 4.425f + OFFSET, 11.425f - OFFSET, -6.35f - OFFSET, 4.945f - OFFSET)
    );

    private final List<Box> KIZ = Arrays.asList(
            new Box(10.3f + OFFSET, -10.2f + OFFSET, 4.32f + OFFSET, 11.55f - OFFSET, -6.0f - OFFSET, 5.57f - OFFSET),
            new Box(9.5f + OFFSET, -10.5 + OFFSET, 4.02 + OFFSET, 10.5f - OFFSET, -9.6f - OFFSET, 4.8f - OFFSET)
    );

    /*
     * Plan a path from start→goal:
     *  - Stays entirely in the union of kizZones.
     *  - Maximizes travel through oasisZones.
     *  - Returns the minimal List<Point> of straight-line waypoints.
     *  - If goal is unreachable, returns as far along the line as possible.
     */

    public List<Point> planPath(Point start, Point goal) {
        Point D = new Point(
                goal.getX() - start.getX(),
                goal.getY() - start.getY(),
                goal.getZ() - start.getZ()
        );

        // 1) Find all KIZ intersections
        List<double[]> kizIv = new ArrayList<>();
        for (Box kiz : KIZ) {
            double[] tt = liangBarsky3D(start, D, kiz);
            if (tt != null) {
                double t0 = clamp(tt[0], 0, 1);
                double t1 = clamp(tt[1], 0, 1);
                if (t0 <= t1) {
                    kizIv.add(new double[]{t0, t1});
                }
            }
        }
        List<double[]> kizMerged = mergeIntervals(kizIv);

        // 2) If no KIZ intersections, check if start/goal are inside any KIZ
        if (kizMerged.isEmpty()) {
            boolean startInKIZ = isPointInAnyZone(start, KIZ);
            boolean goalInKIZ = isPointInAnyZone(goal, KIZ);
            if (!startInKIZ || !goalInKIZ) {
                return Collections.emptyList(); // Path cannot enter KIZ
            } else {
                // Entire path is within KIZ (start and goal are inside)
                return maximizeOasisPath(start, goal, D);
            }
        }

        // 3) Clip path to the KIZ segment [t_entry, t_exit]
        double tEntry = kizMerged.get(0)[0];
        double tExit = kizMerged.get(kizMerged.size() - 1)[1];

        Point adjustedStart = add(start, scale(D, tEntry));
        Point adjustedGoal = add(start, scale(D, tExit));

        // 4) Plan path for the clipped segment, maximizing OASIS
        List<Point> clippedPath = maximizeOasisPath(adjustedStart, adjustedGoal, D);

        // 5) Reconstruct full path with entry/exit points
        List<Point> fullPath = new ArrayList<>();
        if (tEntry > 0) fullPath.add(start); // Add original start if outside KIZ
        fullPath.addAll(clippedPath);
        if (tExit < 1) fullPath.add(goal); // Add original goal if outside KIZ

        return fullPath;
    }

    private static double[] liangBarsky3D(Point P0, Point D, Box B) {
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
    private List<Point> maximizeOasisPath(Point start, Point goal, Point D) {
        List<double[]> oasisIv = new ArrayList<>();
        for (Box oasis : OASIS) {
            double[] tt = liangBarsky3D(start, D, oasis);
            if (tt != null) {
                double t0 = clamp(tt[0], 0, 1);
                double t1 = clamp(tt[1], 0, 1);
                if (t0 <= t1) {
                    oasisIv.add(new double[]{t0, t1});
                }
            }
        }
        List<double[]> oasisMerged = mergeIntervals(oasisIv);

        TreeSet<Double> ts = new TreeSet<>();
        ts.add(0.0);
        ts.add(1.0);
        for (double[] seg : oasisMerged) {
            ts.add(seg[0]);
            ts.add(seg[1]);
        }

        List<Point> waypoints = new ArrayList<>();
        for (Double t : ts) {
            waypoints.add(add(start, scale(D, t)));
        }

        return waypoints;
    }
    private boolean isPointInAnyZone(Point p, List<Box> zones) {
        for (Box zone : zones) {
            if (p.getX() >= zone.xMin && p.getX() <= zone.xMax &&
                    p.getY() >= zone.yMin && p.getY() <= zone.yMax &&
                    p.getZ() >= zone.zMin && p.getZ() <= zone.zMax) {
                return true;
            }
        }
        return false;
    }
}
