package jp.jaxa.iss.kibo.rpc.sampleapk.imageProcessing;

import android.content.res.AssetManager;
import android.util.Pair;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.platform.app.InstrumentationRegistry;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.List;

@RunWith(AndroidJUnit4.class)
public class ImageDetectTest {
    private ImageDetect detector;

    @Before
    public void setUp() throws Exception {
        AssetManager assetManager = InstrumentationRegistry.getInstrumentation().getContext().getAssets();
        detector = new ImageDetect(assetManager, "Kibo_float32.tflite");
    }



    /*
     เอาจิงๆยังไม่ได้เขียน Test เลยฝากหน่อยละกันหุๆๆๆๆ
     */
}