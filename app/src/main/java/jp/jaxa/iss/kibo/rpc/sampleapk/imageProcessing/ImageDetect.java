package jp.jaxa.iss.kibo.rpc.sampleapk.imageProcessing;

import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.util.Pair;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ImageDetect {
    private final Interpreter tflite;
    private final String[] CLASSNAMES = {
            "coin", "compass", "coral", "crystal", "diamond",
            "emerald", "fossil", "key", "letter", "shell", "treasure_box"
    };

    private final int INPUT_SIZE = 736;
    private final int NUM_CLASSES = 11;
    private final int NUM_BOXES = 11109;

    public ImageDetect(AssetManager assetManager, String modelPath) throws IOException {
        tflite = new Interpreter(loadModelFile(assetManager, modelPath));
    }

    private MappedByteBuffer loadModelFile(AssetManager assetManager, String modelPath) throws IOException {
        AssetFileDescriptor fileDescriptor = assetManager.openFd(modelPath);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long length = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, length);
    }

    private ByteBuffer convertMatToByteBuffer(Mat mat) {
        Mat resized = new Mat();
        Imgproc.resize(mat, resized, new Size(INPUT_SIZE, INPUT_SIZE));

        Bitmap bmp = Bitmap.createBitmap(INPUT_SIZE, INPUT_SIZE, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(resized, bmp);

        ByteBuffer input = ByteBuffer.allocateDirect(INPUT_SIZE * INPUT_SIZE * 3 * 4);
        input.order(ByteOrder.nativeOrder());
        int[] pixels = new int[INPUT_SIZE * INPUT_SIZE];
        bmp.getPixels(pixels, 0, INPUT_SIZE, 0, 0, INPUT_SIZE, INPUT_SIZE);

        for(int i = 0; i < pixels.length; i++) {
            int pixel = pixels[i];
            input.putFloat(((pixel >> 16) & 0xFF) / 255.0f); // R
            input.putFloat(((pixel >> 8) & 0xFF) / 255.0f);  // G
            input.putFloat((pixel & 0xFF) / 255.0f);         // B

        }
        input.rewind();
        return input;
    }

    public List<Pair<String, Integer>> detectObjects(Mat image) {
        ByteBuffer inputBuffer = convertMatToByteBuffer(image);
        float[][][] output = new float[1][NUM_BOXES][5 + NUM_CLASSES];

        tflite.run(inputBuffer, output);

        Map<String, Integer> counts = new HashMap<>();
        for(int i=0; i<NUM_BOXES; i++) {
            float[] row = output[0][i];
            float conf = row[4];
            if(conf < 0.5f) continue;

            int bestClass = -1;
            float bestProb = -1f;
            for(int j=0; j< NUM_CLASSES; j++) {
                float prob = row[5 + j];
                if(prob > bestProb) {
                    bestProb = prob;
                    bestClass = j;
                }
            }

            if(bestProb > 0.5f) {
                String className = CLASSNAMES[bestClass];
                counts.put(className, counts.getOrDefault(className, 0) + 1);
            }
        }

        List<Pair<String, Integer>> result = new ArrayList<>();
        for(Map.Entry<String, Integer> entry : counts.entrySet()) {
            result.add(new Pair<>(entry.getKey(), entry.getValue()));
        }
    return  result;
    }
}