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
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractFixedAutoOp;

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
    private int currentStep;
    private int currentRoute;


    private double[][] routeVectors = new double[][]

            //Vector positions 0-5 is for Route 1

            {{1.0,2.0,3.0}, {2.0,3.0,4.5}, {2.0,3.0,4.5}, {2.0,3.0,4.5}, {2.0,3.0,4.5},

             //Vector positions 6-10 is for Route 2
                    {1.0,2.0,3.0}, {2.0,3.0,4.5}, {2.0,3.0,4.5}, {2.0,3.0,4.5}, {2.0,3.0,4.5},


    };

    // Steps for driving are sequential 1 - XX
private int[] routeTimes = new int[]{
        5000, //Step 1 Time
        2000,
        3000,
        4000,
        8000,


};




    private void stateStepper(State newState){


        currentState = newState;

        stateCounter++;

    }

    private void stateCall(){
        telemetry.addData("Current State:", currentState);
        telemetry.update();

        switch (currentState){
            case STATE_INITIAL:

                if(sleep(500)) {
                    stateStepper(State.STATE_DROP);
                }
                break;
            case STATE_DROP:

                if(sleep(500)) {
                    stateStepper(State.STATE_DETACH_LANDER);
                }
                break;
            case STATE_DETACH_LANDER:

                if(sleep(500)) {
                    stateStepper(State.STATE_RETRACT_LIFT);
                }
                break;
            case STATE_RETRACT_LIFT:

                if(sleep(500)) {
                    stateStepper(State.STATE_RETRACT_ARM);
                }
                break;
            case STATE_RETRACT_ARM:

                if(sleep(500)) {
                    stateStepper(State.STATE_ESTABLISH_POSITION);
                }
                break;
            case STATE_ESTABLISH_POSITION:

                if(sleep(500)) {
                    stateStepper(State.STATE_DRIVE_ROUTE);
                }
                break;
            case STATE_DRIVE_ROUTE:


                Forward_Control(routeVectors[0][0],routeVectors[0][1], routeVectors[0][2],routeTimes[0]);
                if(sleep(500)) {
                    stateStepper(State.STATE_DSTEP_1);
                }
                break;
            case STATE_DSTEP_1:

                if(sleep(500)) {
                    stateStepper(State.STATE_DSTEP_2);
                }
                break;
            case STATE_DSTEP_2:

                if(sleep(500)) {
                    stateStepper(State.STATE_DSTEP_3);
                }
                break;
            case STATE_DSTEP_3:

                if(sleep(500)) {
                    stateStepper(State.STATE_DSTEP_4);
                }
                break;
            case STATE_DSTEP_4:

                if(sleep(500)) {
                    stateStepper(State.STATE_DSTEP_5);
                }
                break;
            case STATE_DSTEP_5:

                if(sleep(500)) {
                    stateStepper(State.STATE_STOP);
                }
                break;
            case STATE_STOP:

                if(sleep(500)) {
                    stateStepper(State.STATE_COMPLETE);
                }
                break;
        }


    }



    public boolean sleep(long sleepTime)  {
        runtime.reset();

        if(runtime.milliseconds() <= sleepTime){
            return false;
        }


        return true;
    }

    public boolean opModeIsActive(){
        return isRunning;
    }


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


    private void Forward_Control(double xVec, double yVec, double rVec, int runTime){

        ScalingInputExtractor rightY;
        ScalingInputExtractor leftX;
        ScalingInputExtractor rightX;

        gxVal = xVec;
        gyVal = yVec;
        grVal = rVec;

        runtime.reset();

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

        while(opModeIsActive() && runtime.milliseconds() <= runTime) {
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

    private void main_run(){


        stateCall();

    }

    private void telemetry_update(){
        telemetry.addData("Current State:", currentState);
        telemetry.addData("State Counter:", stateCounter);
        telemetry.addData("Runtime:", runtime);
        telemetry.addData("Left Arm Motor ENC VAL:", robotCfg.Motor_LiftLeft.getCurrentPosition());
        telemetry.addData("Right Arm Motor ENC VAL:", robotCfg.Motor_LiftRight.getCurrentPosition());
        telemetry.addData("Wheel FL Motor ENC VAL:", robotCfg.Motor_WheelFL.getCurrentPosition());


        telemetry.update();
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
        main_run();

        telemetry_update();
    }

    @Override
    protected void go() {
        isRunning = true;


    }

    @Override
    protected Logger createLogger() {
        return null;
    }

    }




