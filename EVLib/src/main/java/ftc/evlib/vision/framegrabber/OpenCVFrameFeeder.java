package ftc.evlib.vision.framegrabber;

import android.view.SurfaceView;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;

import ftc.evlib.vision.framegrabber.*;
import ftc.evlib.vision.framegrabber.RealFrameGrabber;

import static ftc.evlib.vision.framegrabber.GlobalFrameGrabber.frameGrabber;

/**
 * used to manage the frames that openCV takes
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 8/24/16.
 *
 * This initializes opencv and feeds frames to the FrameGrabber
 *
 * @see GlobalFrameGrabber
 * @see ftc.evlib.vision.framegrabber.RealFrameGrabber
 */
public class OpenCVFrameFeeder implements CameraBridgeViewBase.CvCameraViewListener2 {
    private static final String TAG = "OpenCVFrameGrabberInit";
    private final ftc.evlib.vision.framegrabber.FrameGrabber.CameraOrientation cameraOrientation;
    private final boolean ignoreOrientationForDisplay;

    public OpenCVFrameFeeder(CameraBridgeViewBase cameraBridgeViewBase, int frameWidthRequest, int frameHeightRequest) {
        this(cameraBridgeViewBase, frameWidthRequest, frameHeightRequest, ftc.evlib.vision.framegrabber.FrameGrabber.CameraOrientation.PORTRAIT_UP, false);
    }

    public OpenCVFrameFeeder(CameraBridgeViewBase cameraBridgeViewBase, int frameWidthRequest, int frameHeightRequest, ftc.evlib.vision.framegrabber.FrameGrabber.CameraOrientation cameraOrientation) {
        this(cameraBridgeViewBase, frameWidthRequest, frameHeightRequest, cameraOrientation, false);
    }

    public OpenCVFrameFeeder(CameraBridgeViewBase cameraBridgeViewBase, int frameWidthRequest, int frameHeightRequest, boolean ignoreOrientationForDisplay) {
        this(cameraBridgeViewBase, frameWidthRequest, frameHeightRequest, ftc.evlib.vision.framegrabber.FrameGrabber.CameraOrientation.PORTRAIT_UP, ignoreOrientationForDisplay);
    }

    public OpenCVFrameFeeder(CameraBridgeViewBase cameraBridgeViewBase, int frameWidthRequest, int frameHeightRequest, ftc.evlib.vision.framegrabber.FrameGrabber.CameraOrientation cameraOrientation, boolean ignoreOrientationForDisplay) {
        this.cameraOrientation = cameraOrientation;
        this.ignoreOrientationForDisplay = ignoreOrientationForDisplay;

        cameraBridgeViewBase.setVisibility(SurfaceView.VISIBLE);

        cameraBridgeViewBase.setMinimumWidth(frameWidthRequest);
        cameraBridgeViewBase.setMinimumHeight(frameHeightRequest);
        cameraBridgeViewBase.setMaxFrameSize(frameWidthRequest, frameHeightRequest);
        cameraBridgeViewBase.setCvCameraViewListener(this);
    }

    /**
     * When the opencv camera view starts, initialize the frame grabber
     *
     * @param width  the width of the frames that will be delivered
     * @param height the height of the frames that will be delivered
     */
    @Override
    public void onCameraViewStarted(int width, int height) {
        new RealFrameGrabber(TAG, width, height, cameraOrientation, ignoreOrientationForDisplay, false);
    }

    @Override
    public void onCameraViewStopped() {

    }

    /**
     * Receive a frame from opencv
     *
     * @param inputFrame the received frame
     * @return the frame to be displayed on the phone screen
     */
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        return frameGrabber.receiveFrame(inputFrame.rgba());
    }
}
