package jp.jaxa.iss.kibo.rpc.sampleapk.findpath;

import gov.nasa.arc.astrobee.types.Point;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Box;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Node;
import org.opencv.core.Mat;

import java.util.*;

/*
    Using Liang-Barsky algorithm to find the shortest path between two points in a grid.
 */

public class PathPlanUtil {
    private final String TAG = this.getClass().getSimpleName();
    private final double OFFSET = 0.1f;

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

    final double BASE_OASIS_WEIGHT = 0.3;
    final double MAX_OASIS_WEIGHT = 0.5;
    final double STEP = 0.01f;

    /*
    TODO:
        - Find the way that pruning more efficiantly that we still can construct path from `cameform` Set
     */

    public List<Point> planPath(Point start, Point goal) {
        if(isInKIZ(start)) return Collections.emptyList();
        if(isInKIZ(goal)) return Collections.emptyList(); // TODO: Implement this later to find furthurest reachable point

        // astar here
        List<Point> rawPath = aStar(start, goal);



        return Collections.emptyList();
    }

    public List<Point> aStar(Point start, Point goal) {
        double[][] directions = {
                {STEP,0.0d,0.0d},
                {-STEP,0.0d,0.0d},
                {0.0,STEP,0.0},
                {0.0,-STEP,0.0},
                {0.0,0.0,STEP},
                {0.0,0.0,-STEP},
        };

        Set<Point> closedSet = new HashSet<>();
        PriorityQueue<Node> openSet = new PriorityQueue<>(1000);
        Map<Point, Double> gScore = new HashMap<>(5000);
        Map<Point, Point> cameFrom = new HashMap<>(5000);

        Point lastestPoint = goal;

        gScore.put(start, 0.0);
        openSet.add(new Node(start, distance(start, goal)));

        int maxIterations = 10000000;
        int iterations = 0;

        int memoryCheckInterval = 200;
        int maxClosedSetSize = 5000;

        while(!openSet.isEmpty() && iterations < maxIterations) {
            iterations++;

            Node current = openSet.poll();
            if(distance(current.point, goal) < 0.1) {
                return reconstructPath(cameFrom, current.point);
            }

            closedSet.add(current.point);
            lastestPoint = current.point;

            if(iterations % memoryCheckInterval == 0) {
                pruneSearchSpace(gScore, cameFrom, current.point, goal);

                if(closedSet.size() > maxClosedSetSize) {
                    closedSet.clear();
                }
            }

            for(double[] direction : directions) {
                Point neighbor = new Point(
                        current.point.getX() + direction[0],
                        current.point.getY() + direction[1],
                        current.point.getZ() + direction[2]
                );

                if(closedSet.contains(neighbor) || !isInKIZ(neighbor)) continue;

                double tentativeGScore = gScore.getOrDefault(current.point, Double.MAX_VALUE) + calculateCost(current.point, neighbor);
                if(tentativeGScore < gScore.getOrDefault(neighbor, Double.MAX_VALUE)) {
                    cameFrom.put(neighbor, current.point);
                    gScore.put(neighbor, tentativeGScore);
                    openSet.add(new Node(neighbor, tentativeGScore + distance(neighbor, goal)));
                }
            }
        }

        // in case something wrong
        return reconstructPath(cameFrom, lastestPoint);
    }

    private double distance(Point a, Point b) {
        return Math.sqrt(
                Math.pow(a.getX() - b.getX(), 2) +
                Math.pow(a.getY() - b.getY(), 2) +
                Math.pow(a.getZ() - b.getZ(), 2)
        );
    }

    private double calculateCost(Point a, Point b) {
        double baseCost = distance(a, b);
        double oasisRatio = calculateOasisCoverage(a, b);
        double dynamicWeight = BASE_OASIS_WEIGHT +
                (MAX_OASIS_WEIGHT - BASE_OASIS_WEIGHT) * oasisRatio;

        double bonus = dynamicWeight * Math.pow(oasisRatio, 1.5);
        return baseCost * (1.0 - bonus);
    }

    private double calculateOasisCoverage(Point a, Point b) {
        double totalDistance = distance(a, b);
        double oasisDistance = 0.0;

        for(Box oasis: OASIS) {
            double[] t = LiangBarsky.liangBarsky3D(a, b, oasis);
            if(t != null) {
                oasisDistance += (t[1] - t[0]) * totalDistance;
            }
        }

        double cappedOasis = Math.min(oasisDistance, totalDistance);

        return cappedOasis / totalDistance;
    }

    private List<Point> simplyfyPathWithRayCasting(List<Point> path) {
        if(path.size() < 2) return path;

        List<Point> simplified = new ArrayList<>();
        simplified.add(path.get(0));
        int lastKeptIndex = 0;

        for(int i = 1; i < path.size(); i++)  {
            Point candidate = path.get(i);
            Point lastKept = simplified.get(simplified.size() - 1);

            if(!isSegmentInKIZ(lastKept, candidate)) {
                simplified.add(path.get(i - 1));
                lastKeptIndex = i - 1;
                continue;
            }
        }

        return simplified;
    }

    private void pruneSearchSpace(Map<Point, Double> gScore, Map<Point, Point> cameFrom, Point current, Point goal) {
        double bestScore = gScore.getOrDefault(current, Double.MAX_VALUE);

        double threshold = bestScore * 2.0;

        Iterator<Map.Entry<Point, Double>> iterator = gScore.entrySet().iterator();
        Set<Point> toRemove = new HashSet<>();

        while(iterator.hasNext()) {
            Map.Entry<Point, Double> entry = iterator.next();
            if(entry.getValue() > threshold) {
                toRemove.add(entry.getKey());
            }
        }

        for(Point point : toRemove) {
            gScore.remove(point);
            cameFrom.remove(point);
        }

        if(gScore.size() > 5000) {
            List<Map.Entry<Point, Double>> entries = new ArrayList<>(gScore.entrySet());

            int removeCount = entries.size() / 2;
            for(int i = removeCount; i<entries.size(); i++) {
                Point key = entries.get(i).getKey();
                gScore.remove(key);
                cameFrom.remove(key);
            }
        }
    }

    private boolean isSegmentInKIZ(Point a, Point b) {
        Point D = new Point(
                b.getX() - a.getX(),
                b.getY() - a.getY(),
                b.getZ() - a.getZ()
        );

        List<double[]> intervals = new ArrayList<>();

        Box kiz = KIZ.get(0);
        double[] t = LiangBarsky.liangBarsky3D(a, D, kiz);
        if(t != null) {
            intervals.add(new double[]{Math.max(t[0], 0), Math.min(t[1], 1)});
        }

        List<double[]> mergedIntervals = LiangBarsky.mergeIntervals(intervals);
        return isFullIntervalCovered(mergedIntervals);
    }

    private boolean isFullIntervalCovered(List<double[]> intervals) {
        if(intervals.isEmpty()) return false;
        double lastEnd = intervals.get(0)[0];
        for(double[] interval : intervals) {
            if(interval[0] > lastEnd) return false;

            lastEnd = Math.max(lastEnd, interval[1]);
        }
        return lastEnd >= 1.0;
    }

    private List<Point> reconstructPath(Map<Point, Point> cameFrom, Point current) {
        List<Point> path = new ArrayList<>();
        while(current != null) {
            path.add(current);
            current = cameFrom.get(current);
        }
        Collections.reverse(path);
        return path;
    }
    private boolean isInKIZ(Point p) {
        Box box = KIZ.get(0);
        if(p.getX() >= box.xMin && p.getX() <= box.xMax &&
                p.getY() >= box.yMin && p.getY() <= box.yMax &&
                p.getZ() >= box.zMin && p.getZ() <= box.zMax) {
            return true;
        }
        return false;
    }
}
