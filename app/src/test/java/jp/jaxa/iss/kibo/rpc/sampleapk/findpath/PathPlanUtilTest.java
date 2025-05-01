package jp.jaxa.iss.kibo.rpc.sampleapk.findpath;

import android.util.Log;
import gov.nasa.arc.astrobee.types.Point;
import org.junit.Test;

import java.util.List;


public class PathPlanUtilTest {
    @Test
    public void testPlanPath() {
        String TAG = this.getClass().getSimpleName();
        PathPlanUtil pathPlanUtil = new PathPlanUtil();

        // 10.253, -9.788, 4.289 starting point

        Point startingPoint = new Point(10.253, -9.788, 4.289);
        Point targetPoint = new Point(9.867f, -6.85f, 4.945f); //Area 4

        List<Point> path = pathPlanUtil.planPath(startingPoint, targetPoint);

        System.out.println("Current Position: " + startingPoint.toString());
        for(Point p : path) {
            System.out.println("Point: " + p.toString());
        }
    }

}
