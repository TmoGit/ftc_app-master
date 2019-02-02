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

@Autonomous(name="AutoOp2019", group = "Official")

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
    static final double     SERVO_STOP              = 0.5;
    boolean isRunning = false;
    boolean isStepping = false;

    boolean liftFirst = true;
    int liftEndTime = 0;

    double gxVal = 0;
    double gyVal = 0;
    double grVal = 0;

    public double rawHeading;






    private double[] currentVector = new double[]{

     //Initialize vectors as 0.0
            0.0, 0.0, 0.0, 0.0

    };


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
    private int[] drive_target_pos = { 0, 0, 0, 0 };
    private boolean drive_first_run = true;
    private int drive_target_time = 0;

    public int CURRENT_DSTEP_BUMP_TIME = 0;

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

            {{0.35, 0.0, 0.0, 0.3}, {0.0, -0.75, 0.0, 1.5}, {1.0, 0.0, 0.0, 2.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0}, {1.0, 0.0, 0.0, 8.0},

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
            000,
            4000,
            000,
            000,
            2000,
            2000,
            2000,
            2000,
            0
    };

    // Steps for driving are sequential 1 - XX
    private int[] routeTimes = new int[]{
            // Drive intervals need to equal up to 30s - time to execute other steps
            2000, 4000, 7000, 0000, 0000, 4000,
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

                if(Lift_Control(0.5, 3000)) {
                    stateStepper(State.STATE_DETACH_LANDER, false);
                }

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

                routeDesignate();

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


                if((driveControl(currentVector[0], currentVector[1], currentVector[2], currentVector[3], CURRENT_DSTEP_BUMP_TIME,true))){
                 //   if(!robotCfg.Motor_WheelBL.isBusy()) {
                        stateStepper(State.STATE_DSTEP_2, true);
                   // }
                }

                break;
            case STATE_DSTEP_2:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);

               // if (driveControl(currentVector[0], currentVector[1], currentVector[2], currentVector[3], true)) {
                //      if(!robotCfg.Motor_WheelBL.isBusy()) {
                if( (driveControl(currentVector[0], currentVector[1], currentVector[2], currentVector[3], CURRENT_DSTEP_BUMP_TIME, true))){
                   // if(!robotCfg.Motor_WheelBL.isBusy()) {
                        stateStepper(State.STATE_DSTEP_3, true);
                   // }
                }

                //     }
                // }

                break;
            case STATE_DSTEP_3:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);

                if( (driveControl(currentVector[0], currentVector[1], currentVector[2], currentVector[3], CURRENT_DSTEP_BUMP_TIME, true))) {
                   // if (!robotCfg.Motor_WheelBL.isBusy()) {
                        stateStepper(State.STATE_DSTEP_4, true);
                   // }
                }


                break;
            case STATE_DSTEP_4:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);


                    stateStepper(State.STATE_DSTEP_5, true);


                break;
            case STATE_DSTEP_5:

                getCurrentVector();

                //Forward_Control(currentVector[0],currentVector[1], currentVector[2],CURRENT_TIME_INT);
                robotCfg.Motor_Sweeper.setPower(1);

                    stateStepper(State.STATE_STOP, true);


                break;
            case STATE_STOP:

                //All Stop

                driveAllStop();

                robotCfg.Motor_Sweeper.setPower(0);

                    stateStepper(State.STATE_COMPLETE, false);

                break;
        }

    }

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

    private boolean Lift_Control(double power, int liftTime) {
            //Only controls drop/hook lift
            int currentTime = (int) runtime.milliseconds();
            boolean output = false;

            if(liftFirst){
                liftEndTime = currentTime + liftTime;
                liftFirst = false;
            }

            if(currentTime < liftEndTime) {
                robotCfg.Motor_LiftLeft.setPower(-power);
            } else {
                robotCfg.Motor_LiftLeft.setPower(0);
                liftFirst = true;
                liftEndTime = 0;
                output = true;
            }

            return output;
    }

    private void routeDesignate(){
        int route = 0;
        String targetName = Vuforia.targetsAreVisible();

        telemetry.addData("Target ID:", targetName);
        telemetry_update();

        switch (targetName){
            case "Blue_Crater":
                CURRENT_ROUTE = 1;
                break;
            case "Blue_Depot":
                CURRENT_ROUTE = 2;
                break;
            case "Red_Crater":
                CURRENT_ROUTE = 1;
                break;
            case "Red_Depot":
                CURRENT_ROUTE = 2;
                break;
            case "None":
                CURRENT_ROUTE = 1;
                break;
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
        currentVector[3] = routeVectors[CURRENT_DSTEP][3];
        CURRENT_TIME_INT = routeTimes[CURRENT_DSTEP];

        CURRENT_DSTEP_BUMP_TIME = routeTimes[CURRENT_STEP_START + CURRENT_DSTEP - 1] - 1000;
    }

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

    private boolean driveControl(double speed, double direction_speed, double rotation, double inchDist, int drive_time, boolean isDStep) {
        //Main Drive Routine
        boolean output = false;
        double x = rangeClipDouble(speed, -1, 1);
        double y = rangeClipDouble(direction_speed, -1, 1);
        double z = rangeClipDouble(rotation, -1, 1);
        int[] current_pos = new int[]{0,0,0,0};
        int[] target_pos = new int[]{0,0,0,0};
        int[] error_pos = new int[]{0,0,0,0};
        int current_time = (int)runtime.milliseconds();

        if(drive_first_run){
            drive_first_run = false;

            drive_target_time = current_time + drive_time;

            drive_target_pos[0] = robotCfg.Motor_WheelFL.getCurrentPosition() + (int) (inchDist * COUNTS_PER_INCH);
            drive_target_pos[1] = robotCfg.Motor_WheelFR.getCurrentPosition() + (int) (inchDist * COUNTS_PER_INCH);
            drive_target_pos[2] = robotCfg.Motor_WheelBL.getCurrentPosition() + (int) (inchDist * COUNTS_PER_INCH);
            drive_target_pos[3] = robotCfg.Motor_WheelBR.getCurrentPosition() + (int) (inchDist * COUNTS_PER_INCH);
        }

        //Capture all encoder values, only use back wheel of robot for encoder value
        current_pos[0] = robotCfg.Motor_WheelFL.getCurrentPosition();
        error_pos[0] = Math.abs(drive_target_pos[0] - current_pos[0]);

        current_pos[1] = robotCfg.Motor_WheelFR.getCurrentPosition();
        error_pos[1] = Math.abs(drive_target_pos[1] - current_pos[1]);

        current_pos[2] = robotCfg.Motor_WheelBL.getCurrentPosition();
        error_pos[2] = Math.abs(drive_target_pos[2] - current_pos[2]);

        current_pos[3] = robotCfg.Motor_WheelBR.getCurrentPosition();
        error_pos[3] = Math.abs(drive_target_pos[3] - current_pos[3]);

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
        if ((!stateTimeCheck(isDStep)) && (current_time < drive_target_time || (!(current_pos[2] >= target_pos[2]) || (error_pos[2]>=100)))){
            robotCfg.Motor_WheelFL.setPower(wheelPowers[0]);
            robotCfg.Motor_WheelFR.setPower(wheelPowers[1]);
            robotCfg.Motor_WheelBL.setPower(wheelPowers[2]);
            robotCfg.Motor_WheelBR.setPower(wheelPowers[3]);
        }
        else{
            output = true;
            drive_first_run = true;

            driveAllStop();
        }

        //remove this after debug
        telemetry.addData("FL Power:", wheelPowers[0]);
        telemetry.addData("FR Power:", wheelPowers[1]);
        telemetry.addData("BL Power:", wheelPowers[2]);
        telemetry.addData("BR Power:", wheelPowers[3]);
        telemetry.addData("Target Position:", drive_target_pos[2]);
        telemetry.addData("Current Position:", current_pos[2]);
        telemetry.addData("Distance Error:", error_pos[2]);

        telemetry.update();
        return output;

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

    private void telemetry_update(){
     // Telemetry writes, can remove for competition
/*
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
*/

        telemetry.update();
    }


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

        Vuforia.initVuforia();
        Vuforia.activateTracking();
    }

    //Placeholder for logger, needed for class
    @Override
    protected Logger createLogger() {
        return null;
    }

}




