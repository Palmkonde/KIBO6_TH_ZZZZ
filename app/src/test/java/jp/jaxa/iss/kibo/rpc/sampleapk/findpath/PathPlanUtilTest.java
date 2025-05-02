package jp.jaxa.iss.kibo.rpc.sampleapk.findpath;

import gov.nasa.arc.astrobee.types.Point;
import org.junit.Test;

import java.util.List;


public class PathPlanUtilTest {
    @Test
    public void testPlanPathToTarget() {
        String TAG = this.getClass().getSimpleName();
        PathPlanUtil pathPlanUtil = new PathPlanUtil();

        // 10.253, -9.788, 4.289 starting point

        Point startingPoint = new Point(10.553, -9.788, 4.599);
        Point targetPoint1 = new Point(10.95f, -10.58, 5.195);
        Point targetPoint2 = new Point(10.9f, -8.875f, 3.726);
        Point targetPoint3 = new Point(10.925f, -7.95f, 3.726);
        Point targetPoint4 = new Point(9.867f, -6.85f, 4.945f);

        List<Point> path = pathPlanUtil.planPath(startingPoint, targetPoint1);

        System.out.println("Current Position: " + startingPoint.toString());
        for(Point p : path) {
            System.out.println("Point: " + p.toString());
        }
    }

}
