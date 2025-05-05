package jp.jaxa.iss.kibo.rpc.sampleapk.element;

import gov.nasa.arc.astrobee.types.Point;

public class Edge {
    public Point toNode;
    public double cost;

    public Edge(Point b, double cost) {
       this.toNode = b;
       this.cost = cost;
    }
}
