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
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
    private final PathPlanUtil planner = new PathPlanUtil();

    @Override
    protected void runPlan1(){
        api.startMission(); // Start Mission

        Point targetPoint = new Point(9.867f, -6.85f, 4.945f); //Area 4

        List<Point> testPath = planner.planPath(api.getRobotKinematics().getPosition(), targetPoint);

        Log.i(TAG, "Current Position: " + api.getRobotKinematics().getPosition().toString());
        for(Point p : testPath) {
            Log.i(TAG, "Moving to: " + p.toString());
            move(p, 0f, 0f, 0f, 0f);
        }


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
