package org.firstinspires.ftc.teamcode.Rover2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

import static android.R.attr.angle;
import static android.R.attr.targetName;
import static android.view.View.X;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.Rover2018.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.opmodes.AbstractFixedAutoOp;



public class Vuforia{
    public static final int MAX_TARGETS = 4;
    private AbstractFixedAutoOp myOpMode;
    private RobotCfg2018 myRobot;
    static private VuforiaTrackables targets;
    static OpenGLMatrix targetOrientation;
    static OpenGLMatrix robotFromCamera;
    private static boolean targetFound = false;
    private static String targetString = "NULL";

    private static VuforiaTrackable targetBLP;
    private static VuforiaTrackable targetRDP;
    private static VuforiaTrackable targetFRP;
    private static VuforiaTrackable targetBKP;

    public static void activateTracking(){
        if(targets != null){
            targets.activate();
        }
    }

    public static void initVuforia(){
        OpenGLMatrix lastLocation = null;
        int captureCounter = 0;
        File captureDirectory = AppUtil.ROBOT_DATA_DIR;
        VuforiaLocalizer vuforia;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(RobotCfg2018.viewId);

        parameters.cameraName = RobotCfg2018.webcam;

        parameters.vuforiaLicenseKey = "AaDvMLn/////AAABmTZQLjsUR0whgqv7jiVLfwZ3LemI+CeNKa3ByyqPZpIM0rZtqUnW7cA2uF5XIGnJJFZ2MQol1ERxZLZeMl/mZuI2BlWuD42PO2Q3yIeB8bCtLAHXSFGE6ZM3XHOU9KDtzyzgkHBUEPF1Pw1nzLM8r2PW4JQFPOPMqYYFKOjZfHjLhgeY6n6x45L8RziscD7jLSzGVqvJgF5uVK2Xi1IZr/8MH+W6vMBjE4EUuhOZNyevM8XjhYkO50xq03ETi/yKdIDL8U+ef31+HS/kPZp6v4N1zNQFIx2geUwps9cVIftQxw3EXhrmbcGrLCB9rmHNUHVyrq/scKWvj+bCfaPn/zA554wZ+vpBI/2UDInghC4p";

        vuforia  = ClassFactory.getInstance().createVuforia(parameters);

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("RoverRuckus");
       /* targets.get(0).setName("Blue_Crater");
        targets.get(1).setName("Blue_Depot");
        targets.get(2).setName("Red_Crater");
        targets.get(3).setName("Red_Depot");
        */
       targetBLP = targets.get(0);
        targetRDP = targets.get(1);
        targetFRP = targets.get(2);
        targetBKP = targets.get(3);



        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        targetOrientation = OpenGLMatrix.translation(0,0,150)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES,
                        90,
                        0 ,
                        -90
                ));

        robotFromCamera = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }



    }
    /*
    public static void initoldVuforia(){


        OpenGLMatrix lastLocation = null;
        int captureCounter = 0;
        File captureDirectory = AppUtil.ROBOT_DATA_DIR;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(RobotCfg2018.viewId);

        parameters.cameraName = RobotCfg2018.webcam;

        parameters.vuforiaLicenseKey = "AQICQI//////AAABmeVTX5jQ80ZEmLPY3sGyPX6K7pmz/sOPUl18jBKN3GFq" +
                "wCrtS9j2Qusl4h67U0lP7PtXe/1BMC+QsdYEgzYDJp7sMNoE9zvJTv57v3uNZj+O84ZcNWwsOHvB4r/TLLahSUW0md/njq1SeVrdxh1n" +
                "ezdTTDNWw73RxfUw/41IBwULVjPjlZVxaSFMqg4Zx99ndTsJEQ0DhJhQF6R5REQwkj7yiTSZqZS7QoNDdIzikgD6CmC/" +
                "7KhHgr6j8LdO6comvfc2but1QVq+rcqErNvXjYHdnJytS6I1yZ9JxSSEsAAQUsFfQFBfwX" +
                "fxcAdx2XM3taHlPCs3qOcmwFx4YCh7saw+ydzF4y12HijkORNlgLC";

        VuforiaLocalizer vuforia  = ClassFactory.getInstance().createVuforia(parameters);

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targets = vuforia.loadTrackablesFromAsset("RoverRuckus");
        targets.get(0).setName("Blue_Crater");
        targets.get(1).setName("Blue_Depot");
        targets.get(2).setName("Red_Crater");
        targets.get(3).setName("Red_Depot");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        targetOrientation = OpenGLMatrix.translation(0,0,150)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES,
                        90,
                        0 ,
                        -90
                ));

        robotFromCamera = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }



    }
    */

    public static String targetsAreVisible(){

        int targetTestID = 0;
        /*while((targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID)){
            targetTestID++;
        }*/

       // for(int targetTestID = 0;(targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID); targetTestID++);
       /* while ((targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID)) {
            targetTestID++ ;
        }

*/
       targetIsVisible(3);
        return targetString;
    }

    public static boolean targetIsVisible(int targetID){
/*
        //VuforiaTrackable target = targets.get(targetID);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        OpenGLMatrix location  = null;

        // if we have a target, look for an updated robot position
        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetString = target.getName();

            targetFound = true;
        }
        else  {
            targetFound = false;
            targetString = "None";
        }

*/
        return targetFound;
    }
}


