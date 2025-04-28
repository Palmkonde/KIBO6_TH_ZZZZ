package jp.jaxa.iss.kibo.rpc.sampleapk.element;

public class Box {
    public double xMin, yMin, zMin;
    public double xMax, yMax, zMax;

    public Box(double xMin, double yMin, double zMin, double xMax, double yMax, double zMax) {
        this.xMin = xMin;
        this.yMin = yMin;
        this.zMin = zMin;
        this.xMax = xMax;
        this.yMax = yMax;
        this.zMax = zMax;
    }
}
