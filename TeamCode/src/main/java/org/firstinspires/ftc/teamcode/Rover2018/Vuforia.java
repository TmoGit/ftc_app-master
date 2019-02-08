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

    private static List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //When called activates tracking for targets
    public static void activateTracking(){
        if(targets != null){
            targets.activate();
        }
    }

    public static void initVuforia(){

        File captureDirectory = AppUtil.ROBOT_DATA_DIR;
        VuforiaLocalizer vuforia;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(RobotCfg2018.viewId);

        parameters.cameraName = RobotCfg2018.webcam;

        parameters.vuforiaLicenseKey = "AaDvMLn/////AAABmTZQLjsUR0whgqv7jiVLfwZ3LemI+CeNKa3ByyqPZpIM0rZtqUnW7cA2uF5XIGnJJFZ2MQol1ERxZLZeMl/mZuI2BlWuD42PO2Q3yIeB8bCtLAHXSFGE6ZM3XHOU9KDtzyzgkHBUEPF1Pw1nzLM8r2PW4JQFPOPMqYYFKOjZfHjLhgeY6n6x45L8RziscD7jLSzGVqvJgF5uVK2Xi1IZr/8MH+W6vMBjE4EUuhOZNyevM8XjhYkO50xq03ETi/yKdIDL8U+ef31+HS/kPZp6v4N1zNQFIx2geUwps9cVIftQxw3EXhrmbcGrLCB9rmHNUHVyrq/scKWvj+bCfaPn/zA554wZ+vpBI/2UDInghC4p";

        vuforia  = ClassFactory.getInstance().createVuforia(parameters);

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        vuforia.enableConvertFrameToBitmap();

        targets = vuforia.loadTrackablesFromAsset("RoverRuckus");

        targetBLP = targets.get(0);
        targetBLP.setName("BlueAlliance");

        targetRDP = targets.get(1);
        targetRDP.setName("RedAlliance");

        targetFRP = targets.get(2);
        targetFRP.setName("FrontWall");

        targetBKP = targets.get(3);
        targetBKP.setName("BackWall");




        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels




        targetOrientation = OpenGLMatrix
                 .translation(0, mmFTCFieldWidth/2, 0)
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

    public static String targetsAreVisible(){


        //Setup Vuforia Trackers for each marker
        VuforiaTrackableDefaultListener listenerBLP = (VuforiaTrackableDefaultListener)targetBLP.getListener();
        VuforiaTrackableDefaultListener listenerRDP = (VuforiaTrackableDefaultListener)targetRDP.getListener();
        VuforiaTrackableDefaultListener listenerFRP = (VuforiaTrackableDefaultListener)targetFRP.getListener();
        VuforiaTrackableDefaultListener listenerBKP = (VuforiaTrackableDefaultListener)targetBKP.getListener();

        //Logic for Returning which target is visible
        if(listenerBLP.isVisible()){
            targetString = "BlueAlliance";
        }
        else if (listenerRDP.isVisible()){
            targetString = "RedAlliance";
        }
        else if (listenerFRP.isVisible()){
            targetString = "FrontWall";
        }
        else if (listenerBKP.isVisible()){
            targetString = "BackWall";
        }
        else{
            targetString = "None";
        }


        return targetString;
    }

}


