package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.util.Log;
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.sampleapk.findpath.PathPlanUtil;

import java.util.List;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 * TODO: management about the memory again
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1(){
        api.startMission(); // Start Mission

        // memeory leaked
        final PathPlanUtil planner = new PathPlanUtil();
        List<Point> path = null;

        Point startingPoint = new Point(10.553, -9.788, 4.599);
        Point targetPoint1 = new Point(10.95f, -10.58, 5.195);
        Point targetPoint2 = new Point(10.9f, -8.875f, 3.726);
        Point targetPoint3 = new Point(10.925f, -7.95f, 3.726);
        Point targetPoint4 = new Point(9.867f, -6.85f, 4.945f);

        Log.i(TAG, "moving to startingPoint");
        path = planner.planPath(api.getRobotKinematics().getPosition(), startingPoint);
        for(int i = 1; i < path.size(); i++){
            move(path.get(i), 0, 0, 0, 1);
        }

        Log.i(TAG, "moving to targetPoint1");
        path = planner.planPath(api.getRobotKinematics().getPosition(), targetPoint1);
        for(int i = 1; i < path.size(); i++){
            move(path.get(i), 0, 0, 0, 1);
        }

        Log.i(TAG, "moving to targetPoint2");
        path = planner.planPath(api.getRobotKinematics().getPosition(), targetPoint2);
        for(int i = 1; i < path.size(); i++){
            move(path.get(i), 0, 0, 0, 1);
        }

        Log.i(TAG, "moving to targetPoint3");
        path = planner.planPath(api.getRobotKinematics().getPosition(), targetPoint3);
        for(int i = 1; i < path.size(); i++){
            move(path.get(i), 0, 0, 0, 1);
        }

        Log.i(TAG, "moving to targetPoint4");
        path = planner.planPath(api.getRobotKinematics().getPosition(), targetPoint4);
        for(int i = 1; i < path.size(); i++){
            move(path.get(i), 0, 0, 0, 1);
        }

        Log.i(TAG, "Testing Done!!!");
        api.shutdownFactory();
        return;
    }

    @Override
    protected void runPlan2(){
        api.startMission();
        /* ******************************************************************************** */
        /* Write your code to recognize the type and number of landmark items in each area! */
        /* If there is a treasure item, remember it.                                        */
        /* ******************************************************************************** */

        // When you recognize landmark items, let’s set the type and number.
        // api.setAreaInfo(1, "item_name", 1);

        /* **************************************************** */
        /* Let's move to each area and recognize the items. */
        /* **************************************************** */

        // When you move to the front of the astronaut, report the rounding completion.
        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which target item the astronaut has. */
        /* ********************************************************** */

        // Let's notify the astronaut when you recognize it.
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
        api.takeTargetItemSnapshot();
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here.
    }

    // Proper move method
    protected void move(double x, double y, double z, float qx, float qy, float qz, float qw) {
        String pos = x + "," + y + "," + z + ", ";
        String qua = "qua: " + qx + "," + qy + "," + qz + "," + qw;
        Log.i(TAG, pos + qua);

        Point p = new Point(x,y,z);
        Quaternion q = new Quaternion(qx,qy,qz,qw);
        Result r = api.moveTo(p,q,true);

        int cnt = 0;
        int LOOPMAX = 3;
        while(!r.hasSucceeded() && cnt < LOOPMAX)
        {
            r = api.moveTo(p,q, true);
            cnt++;
        }

        if(!r.hasSucceeded()) {
            Log.e(TAG, "Move failed after " + LOOPMAX + " attempts");
        } else {
            Log.i(TAG, "Move succeeded");
        }
    }

    protected void move(Point p, float x, float y, float z, float w) {
        move(p.getX(), p.getY(), p.getZ(), x, y, z, w);
    }
}
