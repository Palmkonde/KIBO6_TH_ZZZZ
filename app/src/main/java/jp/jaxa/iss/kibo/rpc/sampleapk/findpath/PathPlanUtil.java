package jp.jaxa.iss.kibo.rpc.sampleapk.findpath;

import gov.nasa.arc.astrobee.types.Point;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Box;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Edge;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.Node;
import jp.jaxa.iss.kibo.rpc.sampleapk.element.SpatialIndex;
import org.opencv.core.Mat;

import java.nio.file.Path;
import java.util.*;

/*
    Usage use only `planPath` method by giving starting and ending point

    Args:
        - Point start
            starting Point normally use the robot current position
        - Point goal
            goal Point where would you like to go

    Returns:
        - List<Point>
            The list of path include [start, ..., goal]
    Note that: Don't worry about goal point isn't inside the KIZ. The algorithm will adjust it by itself.
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

    final double BASE_OASIS_WEIGHT = 0.9f;
    final double MAX_OASIS_WEIGHT = 1.0f;
    final double STEP = 0.2f;
    final double COST_SAFETY_FACTOR = 1.5f;
    final int MAX_NODE_COUNT = 5000;
    final double RADIUS_NEARBY = 0.7f;

    private static Map<Point, List<Edge>> precomputedGraph;
    private final SpatialIndex sptialIndex = new SpatialIndex();

    public PathPlanUtil() {
        this.initializeGraph();
    }

    public List<Point> planPath(Point start, Point goal) {

        if(!isInKIZ(goal)) {
            goal = changeGoal(goal);
            System.out.println("Change goal to: " + goal);
        }

        Map<Point, List<Edge>> workingGraph = new HashMap<>(precomputedGraph);
        connectToGraph(start, workingGraph, false);
        connectToGraph(goal, workingGraph, true);

        return aStar(start, goal, workingGraph);
    }

    private Point changeGoal(Point goal) {
        double[][] directions = {
                {1, 0, 0},
                {-1, 0, 0},
                {0, 1, 0},
                {0, -1, 0},
                {0, 0, 1},
                {0, 0, -1}
        };

        Point newGoal = null;

        for(int i=1; i<=5; i++) {
            for(double[] direction : directions) {
                newGoal = new Point(
                    goal.getX() + direction[0] * STEP * i,
                    goal.getY() + direction[1] * STEP * i,
                    goal.getZ() + direction[2] * STEP * i
                );
                if(isInKIZ(newGoal)) {
                    return newGoal;
                }
            }
        }

        return newGoal;
    }

    private void connectToGraph(Point point, Map<Point, List<Edge>> graph, boolean isGoal) {
        List<Point> nearbyNodes = findNearbyNodes(point, RADIUS_NEARBY);
        if(!isGoal) {
            for(Point node : nearbyNodes) {
                if(!isSegmentInKIZ(point, node)) continue;

                double cost = calculateCost(point, node);
                List<Edge> list = graph.get(point);
                if(list == null) {
                    list = new ArrayList<Edge>();
                    graph.put(point, list);
                }
                list.add(new Edge(node, cost));
            }
        }
        else {
            for(Point node : nearbyNodes) {
                if(!isSegmentInKIZ(node, point)) continue;
                double cost = calculateCost(node, point);
                List<Edge> list = graph.get(node);
                if(list == null) {
                    list = new ArrayList<Edge>();
                    graph.put(node, list);
                }
                list.add(new Edge(point, cost));
            }
        }
    }

    private List<Point> findNearbyNodes(Point query, double radius) {
        return sptialIndex.query(query, radius);
    }

    private List<Point> aStar(Point start, Point goal, Map<Point, List<Edge>> graph) {
        Set<Point> closedSet = new HashSet<>();
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        Map<Point, Double> gScore = new HashMap<>();
        Map<Point, Point> cameFrom = new HashMap<>();

        gScore.put(start, 0.0);
        cameFrom.put(start, null);
        openSet.add(new Node(start, distance(start, goal)));

        int maxIterations = 100000;
        int iterations = 0;

//        System.out.println("Check edge to goal");
//        for(Edge edge : graph.getOrDefault(goal, Collections.<Edge>emptyList())) {
//            System.out.println("Edge to goal: " + edge.toNode + " cost: " + edge.cost + " distance: " + distance(edge.toNode, goal));
//        }
//
//        System.out.println("Check edge to start");
//        for(Edge edge : graph.getOrDefault(start, Collections.<Edge>emptyList())) {
//            System.out.println("Edge to start: " + edge.toNode + " cost: " + edge.cost + " distance: " + distance(edge.toNode, start));
//        }

        while(!openSet.isEmpty() && iterations < maxIterations) {
            iterations++;

            Node current = openSet.poll();
            if(current.point.equals(goal)) {
//                System.out.println("Found goal: " + current.point);
                return reconstructPath(cameFrom, current.point);
            }

            if(closedSet.contains(current.point)) continue;
            closedSet.add(current.point);

//            System.out.println("Current Point: " + current.point + " Distance: " + distance(current.point, goal) + " fscore: " + current.fScore);

            for(Edge edge : graph.containsKey(current.point) ? graph.get(current.point) : Collections.<Edge>emptyList()) {

                if (closedSet.contains(edge.toNode)) continue;


                double gScore_now = (gScore.containsKey(current.point) ? gScore.get(current.point) : Double.MAX_VALUE);
                double gScore_to = (gScore.containsKey(edge.toNode) ? gScore.get(edge.toNode) : Double.MAX_VALUE);
                double tentativeGScore =  gScore_now + edge.cost;
//                System.out.println("Edge: " + edge.toNode + " gScore: " + gScore_to + " tentativeGScore: " + tentativeGScore);
                if(tentativeGScore < gScore_to) {
                    cameFrom.put(edge.toNode, current.point);
                    gScore.put(edge.toNode, tentativeGScore);
                    openSet.add(new Node(edge.toNode, tentativeGScore + distance(edge.toNode, goal)));
                }
            }
        }

        return reconstructPath(cameFrom, goal);
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

        double dynamicWeight = BASE_OASIS_WEIGHT + (MAX_OASIS_WEIGHT - BASE_OASIS_WEIGHT) * oasisRatio;

        dynamicWeight = Math.max(0.0, Math.min(1.0, dynamicWeight));

        return baseCost * (1.0 - dynamicWeight);
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

    private void initializeGraph() {
        Set<Point> nodes = new HashSet<>();

        for(Box oasis : OASIS) {
            nodes.addAll(oasis.getVertices());
            nodes.addAll(oasis.getNodes());
        }

        precomputedGraph = buildVisibilityGraph(nodes);

        for(Point node : nodes) {
            sptialIndex.insert(node);
        }

    }

    private Map<Point, List<Edge>> buildVisibilityGraph(Set<Point> nodes) {
       Map<Point, List<Edge>>  graph = new HashMap<>();

       for(Point a : nodes) {
           for(Point b : nodes) {
               if(a.equals(b) || !isSegmentInKIZ(a, b)) continue;

               double cost = calculateCost(a, b);
               List<Edge> list = graph.get(a);
               if(list == null) {
                   list = new ArrayList<>();
                   graph.put(a, list);
               }
               list.add(new Edge(b, cost));
           }
       }

       return graph;
    }

    private void pruneSearchSpace(
            final Map<Point, Double> gScore,
            Map<Point, Point> cameFrom,
            Point current, Point start, Point goal,
            List<Point> currentBestPath
    ) {
        double bestScore = gScore.getOrDefault(current, Double.MAX_VALUE);
        double threshold = bestScore * COST_SAFETY_FACTOR;

        Iterator<Map.Entry<Point, Double>> iterator = gScore.entrySet().iterator();

        while(iterator.hasNext()) {
            Map.Entry<Point, Double> entry = iterator.next();
            Point p = entry.getKey();
            double cost = entry.getValue();

            if(currentBestPath.contains(p)
                    || p.equals(start)
                    || p.equals(goal)
                    || cost <= threshold) continue;

            iterator.remove();
            cameFrom.remove(p);
        }

        if(gScore.size()  > MAX_NODE_COUNT) {
            List<Point> nodes = new ArrayList<>(gScore.keySet());
            Collections.sort(nodes, new Comparator<Point>() {
                @Override
                public int compare(Point p1, Point p2) {
                    double score1 = gScore.get(p1);
                    double score2 = gScore.get(p2);
                    return Double.compare(score1, score2);
                }
            });

            for(int i = MAX_NODE_COUNT; i < nodes.size(); i++) {
                Point p = nodes.get(i);
                gScore.remove(p);
                cameFrom.remove(p);
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
        return p.getX() >= box.xMin && p.getX() <= box.xMax &&
                p.getY() >= box.yMin && p.getY() <= box.yMax &&
                p.getZ() >= box.zMin && p.getZ() <= box.zMax;
    }
}
