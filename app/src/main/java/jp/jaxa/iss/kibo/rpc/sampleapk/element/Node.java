package jp.jaxa.iss.kibo.rpc.sampleapk.element;

import gov.nasa.arc.astrobee.types.Point;

public class Node implements Comparable<Node> {
    public Point point;
    public double fScore;
    public Node(Point point, double fScore) {
       this.point = point;
       this.fScore = fScore;
    }

    @Override
    public int compareTo(Node other) {
        return Double.compare(this.fScore, other.fScore);
    }
}
