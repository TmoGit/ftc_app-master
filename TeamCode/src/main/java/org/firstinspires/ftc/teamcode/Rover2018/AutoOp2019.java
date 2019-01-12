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

import com.google.common.collect.ImmutableList;
import com.google.common.io.BaseEncoding;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;

import org.firstinspires.ftc.teamcode.Rover2018.RobotCfg2018;

import ftc.electronvolts.util.AnalogInputScaler;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.driverstation.GamepadIEFactory;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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

public class AutoOp2019 extends LinearOpMode {

    /* Declare OpMode members. */
    RobotCfg2018         robot   = new RobotCfg2018(hardwareMap);
    private ElapsedTime     runtime = new ElapsedTime();



    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     MOTOR_RUNSPEED            = 0.75;
    static final double     TURN_SPEED              = 0.5;

    double gxVal = 0;
    double gyVal = 0;
    double grVal = 0;


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

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


/*
        robot.Motor_WheelFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor_WheelFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor_WheelBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor_WheelBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Motor_WheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor_WheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor_WheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor_WheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/


        // Wait for the game to start (driver presses PLAY)
        waitForStart();




      //  Lift_Control(MOTOR_RUNSPEED, 1, 1);

        sleep(3000);

       // Forward_Control(0, 1, 0, 5000);





        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
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
            robot.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//          robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
            robot.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));

            telemetry.addData("Elapsed Time:", runTime);
            telemetry.update();
        }


        //Resetting global variables
        gxVal = 0;
        gyVal = 0;
        grVal = 0;
    }



    private void Lift_Control(double power, double inchDist, int controlState) {
        if (controlState == 0) {
            robot.Motor_LiftRight.setPower(power);
            robot.Motor_LiftLeft.setPower(-power);
        } else if (controlState == 1) {
            int newLiftLeftTarget = 0;
            int newLiftRightTarget = 0;

            newLiftLeftTarget = robot.Motor_LiftLeft.getCurrentPosition() + (int) (-inchDist * COUNTS_PER_INCH);
            newLiftRightTarget = robot.Motor_LiftRight.getCurrentPosition() + (int) (inchDist * COUNTS_PER_INCH);


            robot.Motor_LiftLeft.setTargetPosition(newLiftLeftTarget);
            robot.Motor_LiftRight.setTargetPosition(newLiftRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Motor_LiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Motor_LiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Move to position
            while (robot.Motor_LiftLeft.getCurrentPosition() != newLiftLeftTarget && robot.Motor_LiftRight.getCurrentPosition() != newLiftRightTarget && opModeIsActive()) {
                robot.Motor_LiftLeft.setPower(power);
                robot.Motor_LiftRight.setPower(-power);


                telemetry.addData("Lift Left Motor ENC VAL: ", robot.Motor_LiftLeft.getCurrentPosition());
                telemetry.addData("Lift Right Motor ENC VAL: ", robot.Motor_LiftRight.getCurrentPosition());
                telemetry.update();
            }
            if (robot.Motor_LiftLeft.getCurrentPosition() >= newLiftLeftTarget && robot.Motor_LiftRight.getCurrentPosition() != newLiftRightTarget) {
                robot.Motor_LiftRight.setPower(0);
                robot.Motor_LiftLeft.setPower(0);
            }

        }
    }



}
