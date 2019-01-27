/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Rover2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractFixedAutoOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.Locale;



/**
 * This file was made by Cut The Red Wire, FTC team 6078
 */

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoOp2019", group="Official")

public class AutoOpTest2019 extends AbstractFixedAutoOp<RobotCfg2018>  {

    /* Declare OpMode members. */
    @Override
    protected RobotCfg2018 createRobotCfg() {
        RobotCfg2018 robot  = new RobotCfg2018(hardwareMap);
        return robot;
    }

    private ElapsedTime     runtime = new ElapsedTime();



    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     MOTOR_RUNSPEED            = 0.75;
    static final double     TURN_SPEED              = 0.5;
    boolean isRunning = false;
    boolean isStepping = false;


    double gxVal = 0;
    double gyVal = 0;
    double grVal = 0;

    public double rawHeading;






    private double[] currentVector = new double[]{

     //Initialize vectors as 0.0
            0.0, 0.0, 0.0

    };


    public int[] TIMELINE = {};

    Vector2D XYVector;

    public enum State { // Ideally, these stay in order of how we use them
        STATE_INITIAL,
        STATE_MOVE_ARM,
        STATE_DROP,
        STATE_DETACH_LANDER,
        STATE_RETRACT_LIFT,
        STATE_RETRACT_ARM,
        STATE_ESTABLISH_POSITION,
        STATE_DRIVE_ROUTE,
        STATE_DSTEP_1,
        STATE_DSTEP_2,
        STATE_DSTEP_3,
        STATE_DSTEP_4,
        STATE_DSTEP_5,
        STATE_SCAN_MINERALS,
        STATE_HIT_OBJECT,
        STATE_DROP_MARKER,
        STATE_STOP,
        STATE_COMPLETE
    }

    private State currentState = State.STATE_INITIAL;
    private int stateCounter = 0;
    public int CURRENT_DSTEP = 0;
    public int CURRENT_ROUTE = 1;
    public int CURRENT_STEP_START = 0;
    public int CURRENT_TIME_INT = 0;


    private double[][] routeVectors = new double[][]

             /*
        Route 1	          Route 2	              Route 3	       Route 4
0 - Drive Step 1	6 - Drive Step 7	12 - Drive Step 13	18 - Drive Step 19
1 - Drive Step 2	7 - Drive Step 8	13 - Drive Step 14	19 - Drive Step 20
2 - Drive Step 3	8 - Drive Step 9	14 - Drive Step 15	20 - Drive Step 21
3 - Drive Step 4	9 - Drive Step 10	15 - Drive Step 16	21 - Drive Step 22
4 - Drive Step 5	10 -Drive Step 11	16 - Drive Step 17	22 - Drive Step 23
5 - Drive Step 6	11- Drive Step 12	17 - Drive Step 18	23 - Drive Step 24

        Direction
	F	L	R	B	HL	HR

Motor Output
FL	1	1	-1	-1	1	0
FR	-1	1	-1	1	-1	0
BL	-1	1	-1	1	0	-1
Br	1	1	-1	-1	0	-1

Vector Input
X	1	0	0	-1	1	1
Y	0	1	-1	0	0	0
Z	0	0	0	0	1	-1

4 Vector array element is magnitude i.e. distance in inches



*/

            //Vector positions 0-5 is for Route 1

            {{1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0},

                    //Vector positions 6-11 is for Route 2
                    {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0},

                    //Vector positions 12-17 is for Route 3
                    {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0},

                            //Vector positions 18-23 is for Route 4
                    {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0},

    };


    private int[] stepTimes = new int[]{
            // Sequencer step intervals, these need to match drive step times and equal 30s
            2000,
            2000,
            2000,
            2000,
            2000,
            2000,
            2000,
            2000,
            2000,
            0
    };

    // Steps for driving are sequential 1 - XX
    private int[] routeTimes = new int[]{
            // Drive intervals need to equal up to 30s - time to execute other steps
            4000, 4000, 4000, 6000, 4000, 4000,
            2000, 2000, 2000, 2000, 2000, 2000,
            2000, 2000, 2000, 2000, 2000, 2000,
            2000, 2000, 2000, 2000, 2000, 2000


    };


    public double rangeClipDouble(double input, double min, double max){
        //Used for range clipping
        double output = input;

        if(input > max){
            output = max;
        }
        else if(input < min){
            output = min;
        }

        return output;
    }

    public double getGyroHeading(Orientation angles){
        //Formats Gyro reading into Degrees
        rawHeading = -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        return rawHeading;
    }

    private boolean stateTimeCheck(boolean isDStep){
        //Used for checking drive step time intervals
        boolean output = false;
        int waitTime = 0;


        for (int i = 0; i < stateCounter; i++) {
            waitTime += stepTimes[i];
        }

        if(isDStep) {
            for(int i = CURRENT_STEP_START; i < CURRENT_DSTEP; i++) {
                waitTime += routeTimes[i];
            }
        }

        if(runtime.milliseconds() >= waitTime){
            output = true;
        }

        return output;
    }


    private void stateStepper(State newState, boolean isDStep){
        // Steps sequencer
        if(stateTimeCheck(isDStep)) {
            currentState = newState;

            if(isDStep) {
                CURRENT_DSTEP++;
            }
            else{
                stateCounter++;
            }

        }



    }

    private void stateCall(){
        // Main State Sequencer
        telemetry.addData("Current State:", currentState);
        telemetry.update();

        switch (currentState){
            case STATE_INITIAL:



                    stateStepper(State.STATE_MOVE_ARM, false);

                break;
            case STATE_MOVE_ARM:


                    stateStepper(State.STATE_DROP, false);

                break;
            case STATE_DROP:


                    stateStepper(State.STATE_DETACH_LANDER, false);

                break;
            case STATE_DETACH_LANDER:


                    stateStepper(State.STATE_RETRACT_LIFT, false);

                break;
            case STATE_RETRACT_LIFT:


                    stateStepper(State.STATE_RETRACT_ARM, false);

                break;
            case STATE_RETRACT_ARM:


                    stateStepper(State.STATE_ESTABLISH_POSITION, false);

                break;
            case STATE_ESTABLISH_POSITION:


                    stateStepper(State.STATE_DRIVE_ROUTE, false);

                break;
            case STATE_DRIVE_ROUTE:

                // Set vectors
                setDriveStart(CURRENT_ROUTE);
                CURRENT_DSTEP = CURRENT_STEP_START;
                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);

                //Sleep

                    stateStepper(State.STATE_DSTEP_1, true);


                break;


            case STATE_DSTEP_1:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);
              if( (driveControl(currentVector[0], currentVector[1], currentVector[2], currentVector[3], true))){


                stateStepper(State.STATE_DSTEP_2, true);
            }

                break;
            case STATE_DSTEP_2:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);


                    stateStepper(State.STATE_DSTEP_3, true);


                break;
            case STATE_DSTEP_3:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);


                    stateStepper(State.STATE_DSTEP_4, true);


                break;
            case STATE_DSTEP_4:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);


                    stateStepper(State.STATE_DSTEP_5, true);


                break;
            case STATE_DSTEP_5:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);


                    stateStepper(State.STATE_STOP, true);


                break;
            case STATE_STOP:

                //All Stop

                driveAllStop();

                    stateStepper(State.STATE_COMPLETE, false);

                break;
        }


    }


/*
Dead code
    public boolean sleep(long sleepTime)  {
       // runtime.reset();


        if(runtime.milliseconds() <= sleepTime){
            return false;
        }


        return true;
    }
*/
    public boolean opModeIsActive(){
        return isRunning;
    }

//No longer used - technically dead code
    class ScalingInputExtractor implements InputExtractor<Double> {
        InputExtractor<Double> ext;
        private double factor;
        ScalingInputExtractor(InputExtractor<Double> ext, double f) {
            this.ext = ext;
            this.factor = f;
        }
        @Override
        public Double getValue() {
            return ext.getValue()*factor;
        }
        public void setFactor(double f) {
            this.factor = f;
        }
    }


    private void Lift_Control(double power, double inchDist, int controlState) {
        if (controlState == 0) {
            robotCfg.Motor_LiftRight.setPower(power);
            robotCfg.Motor_LiftLeft.setPower(-power);
        } else if (controlState == 1) {
            int newLiftLeftTarget = 0;
            int newLiftRightTarget = 0;

            newLiftLeftTarget = robotCfg.Motor_LiftLeft.getCurrentPosition() + (int) (-inchDist * COUNTS_PER_INCH);
            newLiftRightTarget = robotCfg.Motor_LiftRight.getCurrentPosition() + (int) (inchDist * COUNTS_PER_INCH);


            robotCfg.Motor_LiftLeft.setTargetPosition(newLiftLeftTarget);
            robotCfg.Motor_LiftRight.setTargetPosition(newLiftRightTarget);

            // Turn On RUN_TO_POSITION
            robotCfg.Motor_LiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robotCfg.Motor_LiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Move to position
            while (robotCfg.Motor_LiftLeft.getCurrentPosition() != newLiftLeftTarget && robotCfg.Motor_LiftRight.getCurrentPosition() != newLiftRightTarget && opModeIsActive()) {
                robotCfg.Motor_LiftLeft.setPower(power);
                robotCfg.Motor_LiftRight.setPower(-power);


                telemetry.addData("Lift Left Motor ENC VAL: ", robotCfg.Motor_LiftLeft.getCurrentPosition());
                telemetry.addData("Lift Right Motor ENC VAL: ", robotCfg.Motor_LiftRight.getCurrentPosition());
                telemetry.update();
            }
            if (robotCfg.Motor_LiftLeft.getCurrentPosition() >= newLiftLeftTarget && robotCfg.Motor_LiftRight.getCurrentPosition() != newLiftRightTarget) {
                robotCfg.Motor_LiftRight.setPower(0);
                robotCfg.Motor_LiftLeft.setPower(0);
            }

        }
    }

    private void setDriveStart(int ROUTE){
        // Drive steps based on routes
 /*
        Route 1	          Route 2	              Route 3	       Route 4
0 - Drive Step 1	6 - Drive Step 7	12 - Drive Step 13	18 - Drive Step 19
1 - Drive Step 2	7 - Drive Step 8	13 - Drive Step 14	19 - Drive Step 20
2 - Drive Step 3	8 - Drive Step 9	14 - Drive Step 15	20 - Drive Step 21
3 - Drive Step 4	9 - Drive Step 10	15 - Drive Step 16	21 - Drive Step 22
4 - Drive Step 5	10 -Drive Step 11	16 - Drive Step 17	22 - Drive Step 23
5 - Drive Step 6	11- Drive Step 12	17 - Drive Step 18	23 - Drive Step 24

*/

        switch (ROUTE){
            case 1:

                CURRENT_STEP_START = 0;
                break;

            case 2:

                CURRENT_STEP_START = 6;
                break;

            case 3:

                CURRENT_STEP_START = 12;
                break;

            case 4:

                CURRENT_STEP_START = 18;
                break;
        }

    }

    private void getCurrentVector(){
        // Pulls down Vectors from array
        currentVector[0] = routeVectors[CURRENT_DSTEP][0];
        currentVector[1] = routeVectors[CURRENT_DSTEP][1];
        currentVector[2] = routeVectors[CURRENT_DSTEP][2];
        CURRENT_TIME_INT = routeTimes[CURRENT_DSTEP];


    }
/*
    Dead Code
    private void Forward_Control(double xVec, double yVec, double rVec, int runTime){

        ScalingInputExtractor rightY;
        ScalingInputExtractor leftX;
        ScalingInputExtractor rightX;

        gxVal = xVec;
        gyVal = yVec;
        grVal = rVec;

       // runtime.reset();

        InputExtractor<Double> x = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return gxVal;
            }
        };

        InputExtractor<Double> y = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return gyVal;
            }
        };

        InputExtractor<Double> r = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return grVal;
            }
        };


        rightX = new ScalingInputExtractor(x, 1);
        rightY = new ScalingInputExtractor(y, 1);
        leftX = new ScalingInputExtractor(r, 1);


        if(stateTimeCheck(true) == false) {
            robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//          robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
            robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));

            telemetry.addData("Elapsed Time:", runTime);
            telemetry.update();
        }




        //Resetting global variables
        gxVal = 0;
        gyVal = 0;
        grVal = 0;
    }
*/
    public void driveAllStop(){
        //Stops and resets encoders

        robotCfg.Motor_WheelFL.setPower(0);
        robotCfg.Motor_WheelFR.setPower(0);
        robotCfg.Motor_WheelBL.setPower(0);
        robotCfg.Motor_WheelBR.setPower(0);

        robotCfg.Motor_WheelFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotCfg.Motor_WheelFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotCfg.Motor_WheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotCfg.Motor_WheelBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private boolean driveControl(double speed, double direction_speed, double rotation, double inchDist, boolean isDStep) {
        //Main Drive Routine

        double x = rangeClipDouble(speed, -1, 1);
        double y = rangeClipDouble(direction_speed, -1, 1);
        double z = rangeClipDouble(rotation, -1, 1);
        int[] current_pos = new int[]{0,0,0,0};
        int[] target_pos = new int[]{0,0,0,0};
        int[] error_pos = new int[]{0,0,0,0};

        //Capture all encoder values, only use back wheel of robot for encoder value
        current_pos[0] = robotCfg.Motor_WheelFL.getCurrentPosition();
        target_pos[0] = robotCfg.Motor_WheelFL.getTargetPosition() + (int) (inchDist * COUNTS_PER_INCH);
        error_pos[0] = Math.abs(target_pos[0] - current_pos[0]);

        current_pos[1] = robotCfg.Motor_WheelFR.getCurrentPosition();
        target_pos[1] = robotCfg.Motor_WheelFR.getTargetPosition() + (int) (inchDist * COUNTS_PER_INCH);
        error_pos[1] = Math.abs(target_pos[1] - current_pos[1]);

        current_pos[2] = robotCfg.Motor_WheelBL.getCurrentPosition();
        target_pos[2] = robotCfg.Motor_WheelBL.getTargetPosition() + (int) (inchDist * COUNTS_PER_INCH);
        error_pos[2] = Math.abs(target_pos[2] - current_pos[2]);

        current_pos[3] = robotCfg.Motor_WheelBR.getCurrentPosition();
        target_pos[3] = robotCfg.Motor_WheelBR.getTargetPosition() + (int) (inchDist * COUNTS_PER_INCH);
        error_pos[3] = Math.abs(target_pos[3] - current_pos[3]);

        /*
        Dead Code
        double V_1 = 0;
        double V_2 = 0;
        double V_3 = 0;
        double V_4 = 0;
        double Vd = speed;
        double Td = Math.toRadians(rotation);
        double Td_Comp = 0;
        double Vt = 1;
        */

        robotCfg.angles = robotCfg.Gyro_Hub.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);

        double angle = getGyroHeading(robotCfg.angles);



/*
        //Equations for wheel power
        Reference Only
        V_1 = Vd * Math.sin(-Td + (Math.PI / 4)) - Vt;
        V_2 = Vd * Math.cos(-Td + (Math.PI / 4)) + Vt;
        V_3 = Vd * Math.cos(-Td + (Math.PI / 4)) - Vt;
        V_4 = Vd * Math.sin(-Td + (Math.PI / 4)) + Vt;
*/
        double cosA = Math.cos(Math.toRadians(angle));
        double sinA = Math.sin(Math.toRadians(angle));
        double x1 = x * cosA - y * sinA;
        double y1 = x * sinA + y * cosA;

        double[] wheelPowers = new double[4];
/*
        Dead Code
        wheelPowers[0] = V_1;
        wheelPowers[1] = V_2;
        wheelPowers[2] = V_3;
        wheelPowers[3] = V_4;
        */

        //Gyro compensation NOT READY
       // if (z==0){
         //   z = getCompensation(angle);
        //}

        //Range clipped power to motors
        wheelPowers[0] = rangeClipDouble((x1 + y1 + z),-1,1);
        wheelPowers[1] = rangeClipDouble(( -x1 + y1 - z),-1,1);
        wheelPowers[2] = rangeClipDouble((-x1 + y1 + z),-1,1);
        wheelPowers[3] = rangeClipDouble((x1 + y1 - z),-1,1);



        //Check time and distance
        if ((!stateTimeCheck(isDStep)) || !(current_pos[2] >= target_pos[2])){
            robotCfg.Motor_WheelFL.setPower(wheelPowers[0]);
            robotCfg.Motor_WheelFR.setPower(wheelPowers[1]);
            robotCfg.Motor_WheelBL.setPower(wheelPowers[2]);
            robotCfg.Motor_WheelBR.setPower(wheelPowers[3]);

        }
        else{

            driveAllStop();

        }

        //remove this after debug
        telemetry.addData("FL Power:", wheelPowers[0]);
        telemetry.addData("FR Power:", wheelPowers[1]);
        telemetry.addData("BL Power:", wheelPowers[2]);
        telemetry.addData("BR Power:", wheelPowers[3]);
        telemetry.addData("Target Position:", target_pos[2]);
        telemetry.addData("Current Position:", current_pos[2]);
        telemetry.addData("Distance Error:", error_pos[2]);

        telemetry.update();
        return true;

    }

    double getCompensation(double TARGET_HEADING) {
        //Heading compensation, this is not implemented
        double rotation = 0.0;
        double currentHeading = getGyroHeading(robotCfg.angles);
        double targetHeading = TARGET_HEADING;
        double posError = currentHeading - targetHeading;
        double epsilon = 3;
        double minSpeed = .35;
        double maxSpeed = 0.5;//1


        if (Math.abs(posError) > 180) {
            posError = -360 * Math.signum(posError) + posError;
        }
        if (Math.abs(posError) > epsilon) {
            rotation = minSpeed + (Math.abs(posError) / 180) * (maxSpeed - minSpeed);
            rotation = rotation * Math.signum(posError);
        }



        return rotation;
    }

/*Not needed - Dead Code
    private void main_run(){
        //Main Loop

        stateCall();

    }
    */

    private void telemetry_update(){
     // Telemetry writes, can remove for competition

        telemetry.addData("Current State:", currentState);
        telemetry.addData("State Counter:", stateCounter);
        telemetry.addData("Current Route #", CURRENT_ROUTE);
        telemetry.addData("Current D-Step", CURRENT_DSTEP);
        telemetry.addData("Current Vector X", currentVector[0]);
        telemetry.addData("Current Vector Y", currentVector[1]);
        telemetry.addData("Current Vector Z", currentVector[2]);
        telemetry.addData("Runtime:", runtime);
        telemetry.addData("Left Arm Motor ENC VAL:", robotCfg.Motor_LiftLeft.getCurrentPosition());
        telemetry.addData("Right Arm Motor ENC VAL:", robotCfg.Motor_LiftRight.getCurrentPosition());
        telemetry.addData("Wheel FL Motor ENC VAL:", robotCfg.Motor_WheelFL.getCurrentPosition());

        //composeTelemetry();

        telemetry.update();
    }

/*
Dead code
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            robotCfg.angles   = robotCfg.Gyro_Hub.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robotCfg.gravity  = robotCfg.Gyro_Hub.getGravity();
        }
        });


        telemetry.addData("status", new Func<String>() {
                    @Override public String value() {
                        return robotCfg.Gyro_Hub.getSystemStatus().toShortString();
                    }
                });
        telemetry.addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robotCfg.Gyro_Hub.getCalibrationStatus().toString();
                    }
                });


        telemetry.addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robotCfg.angles.angleUnit, robotCfg.angles.firstAngle);
                    }
                });
        telemetry.addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robotCfg.angles.angleUnit, robotCfg.angles.secondAngle);
                    }
                });
        telemetry.addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robotCfg.angles.angleUnit, robotCfg.angles.thirdAngle);
                    }
                });


        telemetry.addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return robotCfg.gravity.toString();
    }
});
        telemetry.addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(robotCfg.gravity.xAccel*robotCfg.gravity.xAccel
                                        + robotCfg.gravity.yAccel*robotCfg.gravity.yAccel
                                        + robotCfg.gravity.zAccel*robotCfg.gravity.zAccel));
                    }
                });
    }

    */

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }





    @Override
    protected void setup_act() {

    }

    @Override
    protected void end() {
        isRunning = false;
    }

    @Override
    protected void act() {
        //Main routine
        robotCfg.Gyro_Hub.startAccelerationIntegration(new Position(), new Velocity(), 1000);

       // main_run();
        stateCall();

        telemetry_update();
    }

    @Override
    protected void go() {
        //First run, initialization

        isRunning = true;


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robotCfg.Gyro_Hub.initialize(parameters);

        runtime.reset();
    }

    //Placeholder for logger, needed for class
    @Override
    protected Logger createLogger() {
        return null;
    }

    }




