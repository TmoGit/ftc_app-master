package ftc.evlib.vision.processors;

import android.util.Log;

import com.google.common.collect.ImmutableList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import ftc.evlib.util.FileUtil;
import ftc.evlib.util.StepTimer;
import ftc.evlib.vision.ImageUtil;
import ftc.evlib.vision.processors.*;
import ftc.evlib.vision.processors.BeaconColorResult;
import ftc.evlib.vision.processors.BeaconFinder;
import ftc.evlib.vision.processors.BeaconResult;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 8/25/16.
 *
 * determines the position and color (red, green, or blue) of the beacon in an image
 */
public class RGBBeaconProcessor implements ftc.evlib.vision.processors.ImageProcessor<ftc.evlib.vision.processors.BeaconResult> {
    private static final String TAG = "RGBBeaconProcessor";
    private static final int THICKNESS = 2;
    private static final double RELATIVE_BAR_HEIGHT = .03;
    private static final double MIN_MASS = 150; //minimum mass for column sum

    /**
     * Convert to hsv
     * Threshold black in hsv
     * Find contours (edge detection on binary image)
     * Sort contours to find circles
     * Find 2 circles closest in size (black buttons on the beacon)
     * Rotate the image so the beacon is upright
     *
     * @param startTime the time the frame was received
     * @param rgbaFrame the input image
     * @return a BeaconColorResult object which contains info about the beacon position and colors
     */
    @Override
    public ImageProcessorResult<ftc.evlib.vision.processors.BeaconResult> process(long startTime, Mat rgbaFrame, boolean saveImages) {
        StepTimer stepTimer = new StepTimer(TAG, Log.INFO);
        stepTimer.start();

        if (saveImages) {
            stepTimer.step("save 00");
            //save the raw camera image for logging
            ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "00_camera", startTime);
        }

        stepTimer.step("rgb2hsv");
        //convert image to hsv
        Mat hsv = new Mat();
        Imgproc.cvtColor(rgbaFrame, hsv, Imgproc.COLOR_RGB2HSV);

        ftc.evlib.vision.processors.BeaconFinder beaconFinder = new BeaconFinder();
        ImageProcessorResult<BeaconPositionResult> result = beaconFinder.process(startTime, rgbaFrame, saveImages);
        BeaconPositionResult positionResult = result.getResult();
        rgbaFrame = result.getFrame();

        //if no circle matches were found, give an unknown result
        if (positionResult == null) {
            if (saveImages) {
                stepTimer.step("save 02");
                ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "02_circles", startTime);
            }
            return new ImageProcessorResult<>(startTime, rgbaFrame, new ftc.evlib.vision.processors.BeaconResult(new ftc.evlib.vision.processors.BeaconColorResult(), null));
        }

//    Point p1 = positionResult.getLeftButton();
//    Point p2 = positionResult.getRightButton();
//    Point midpoint = positionResult.getMidpoint();

        //rotate the frame about the midpoint of the two circles
        //so that the line between them is horizontal

        stepTimer.step("rotate image");
        //draw the midpoint and line
//    Imgproc.circle(rgbaFrame, midpoint, THICKNESS, ImageUtil.MAGENTA, THICKNESS);
//    Imgproc.line(rgbaFrame, p1, p2, ImageUtil.BLACK, THICKNESS);

        //rotate the frame and hsv image
        Mat rotationMatrix2D = positionResult.getRotationMatrix2D(); //Imgproc.getRotationMatrix2D(midpoint, foundAngleDegrees, 1);
        Imgproc.warpAffine(rgbaFrame, rgbaFrame, rotationMatrix2D, rgbaFrame.size());
        Imgproc.warpAffine(hsv, hsv, rotationMatrix2D, hsv.size());

        //find start and end columns for the mass calculation
        int[] start = {
                positionResult.getLeftRect().x,
                positionResult.getRightRect().x
        };
        int[] end = {
                positionResult.getLeftRect().x + positionResult.getLeftRect().width,
                positionResult.getRightRect().x + positionResult.getRightRect().width
        };

        if (saveImages) {
            stepTimer.step("save 02");
            ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "02_rotated", startTime);
        }

        //now that the beacon is found, we need to detect the color of each side

        stepTimer.step("beacon color detection");

        //make a list of channels that are blank (used for combining binary images)
        List<Mat> rgbaChannels = new ArrayList<>();

        double[] max = new double[2];
        int[] maxIndex = new int[2];

        Arrays.fill(max, Double.MIN_VALUE);
        Arrays.fill(maxIndex, 3);

        //variables to use inside the loop
        Mat maskedImage;
        Mat colSum = new Mat();
        double mass;

        int[] data = new int[3];

        //used for logging the column sum
        int[][] colSum1 = new int[hsv.width()][4];

        //column
        for (int x = 0; x < hsv.width(); x++) {
            colSum1[x][0] = x;
        }

        //loop through the rgb channels
        for (int i = 0; i < 3; i++) {
            //apply HSV thresholds
            maskedImage = new Mat();
            ImageUtil.hsvInRange(hsv, ImageUtil.HSV_THRESHOLD_MIN.get(i), ImageUtil.HSV_THRESHOLD_MAX.get(i), maskedImage);

            //add the binary image to rgbaChannels
            rgbaChannels.add(maskedImage);

            //apply a column sum to the (unscaled) binary image
            Core.reduce(maskedImage, colSum, 0, Core.REDUCE_SUM, 4);

            //retrieve values to log the column sum
            for (int x = 0; x < hsv.width(); x++) {
                colSum.get(0, x, data);
                colSum1[x][i + 1] = data[0];
            }

            //loop through left and right to calculate mass and center of mass
            for (int j = 0; j < 2; j++) {
                //calculate the mass
                mass = 0;
                for (int x = start[j]; x < end[j]; x++) {
                    colSum.get(0, x, data);
                    mass += data[0];
                }

                if (start[j] == end[j]) {
                    mass = 0;
                } else {
                    mass /= (start[j] - end[j]) * (start[j] - end[j]); //hsv.size().area();
                }

                Log.i(TAG, "Mass: " + mass);

                //if the mass found is greater than the max for this side
                if (mass >= MIN_MASS && mass > max[j]) {
                    //this mass is the new max for this side
                    max[j] = mass;
                    //and this index is the new maxIndex for this side
                    maxIndex[j] = i;
                }
            }
        }

        ImageUtil.log2DArray(FileUtil.getLogsDir(), "RGBColSum", startTime, ".csv", ImmutableList.of("column", "red", "green", "blue"), colSum1);

        //merge the 3 binary images into one
        rgbaChannels.add(Mat.zeros(hsv.size(), CvType.CV_8UC1));
        Core.merge(rgbaChannels, rgbaFrame);

        stepTimer.step("overlaying boxes");
        //use the minIndex and maxIndex to get the left and right colors
        ftc.evlib.vision.processors.BeaconColorResult.BeaconColor[] beaconColors = ftc.evlib.vision.processors.BeaconColorResult.BeaconColor.values();
        ftc.evlib.vision.processors.BeaconColorResult.BeaconColor left = beaconColors[maxIndex[0]];
        ftc.evlib.vision.processors.BeaconColorResult.BeaconColor right = beaconColors[maxIndex[1]];

        //draw the white start and end bars
        for (int i = 0; i < 2; i++) {
            Imgproc.rectangle(rgbaFrame, new Point(start[i], 0), new Point(start[i], hsv.height()), ImageUtil.WHITE, THICKNESS);
            Imgproc.rectangle(rgbaFrame, new Point(end[i], 0), new Point(end[i], hsv.height()), ImageUtil.WHITE, THICKNESS);
        }

        //draw the color result bars
        int barHeight = (int) (RELATIVE_BAR_HEIGHT * hsv.height());
        Imgproc.rectangle(rgbaFrame, new Point(0, 0), new Point(hsv.width() / 2, barHeight), left.color, barHeight);
        Imgproc.rectangle(rgbaFrame, new Point(hsv.width() / 2, 0), new Point(hsv.width(), barHeight), right.color, barHeight);

        if (saveImages) {
            stepTimer.step("save 03");
            ImageUtil.saveImage(TAG, rgbaFrame, Imgproc.COLOR_RGBA2BGR, "03_binary", startTime);
        }
        stepTimer.stop();

        return new ImageProcessorResult<>(startTime, rgbaFrame, new BeaconResult(new BeaconColorResult(left, right), positionResult));
    }
}
