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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="K9bot: Telop Tank", group="K9bot")
@Disabled
public class TeleOp_Mec2018 extends AbstractTeleOp<RobotCfg2018> {

    ScalingInputExtractor rightY;
    ScalingInputExtractor leftX;
    ScalingInputExtractor rightX;

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
    private TeleOp2018.MotorSpeedFactor currentSpeedFactor = TeleOp2018.MotorSpeedFactor.FAST;

    enum MotorSpeedFactor {
        FAST(1.0), SLOW(0.5);
        private double factor;
        MotorSpeedFactor(double x) {
            this.factor = x;
        }
        public double getFactor() {
            return factor;
        }
    }

    @Override
    protected Function getJoystickScalingFunction() {
        return Functions.eBased(5);
    }

    @Override
    protected RobotCfg2018 createRobotCfg() {
        return new RobotCfg2018(hardwareMap);
    }

    @Override
    protected Logger createLogger() {
        //log the gamepads, and the motors, sensors, servos, etc. from MainRobotCfg
        return new Logger("", "teleop.csv", new ImmutableList.Builder<Logger.Column>()
//                .add(
//                new Logger.Column("state", new InputExtractor<StateName>() {
//                    @Override
//                    public StateName getValue() {
//                        return stateMachine.getCurrentStateName();
//                    }
//                }))
                //.add(
                //                       new Logger.Column(TeleOpPlayback.GAMEPAD_2_TITLE, new InputExtractor<String>() {
                //                           @Override
                //                           public String getValue() {
                //                               try {
                //                                   return BaseEncoding.base64Url().encode(gamepad2.toByteArray());
                //                               } catch (RobotCoreException e) {
                //                                  e.printStackTrace();
                //                                  return "";
                //                              }
                //                         }
                //                      })
                .build());
    }


    @Override
    protected void setup() {

    }

    @Override
    protected void setup_act() {

    }

    private void forwardControl() {
        double f = currentSpeedFactor.getFactor();
        rightY = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_y), f);
        leftX = new ScalingInputExtractor(InputExtractors.negative(driver1.left_stick_x), f);
        rightX = new ScalingInputExtractor(driver1.right_stick_x, f);
        //noinspection SuspiciousNameCombination
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));
    }

    @Override
    protected void go() {
        forwardControl();

    }

    @Override
    protected void act() {



        if(driver1.a.justPressed()) {
            if (currentSpeedFactor == TeleOp2018.MotorSpeedFactor.FAST) {
                currentSpeedFactor = TeleOp2018.MotorSpeedFactor.SLOW;
            } else {
                currentSpeedFactor = TeleOp2018.MotorSpeedFactor.FAST;
            }
            double f = currentSpeedFactor.getFactor();
            leftX.setFactor(f);
            rightY.setFactor(f);
            rightX.setFactor(f);
        }





        if(driver1.right_bumper.justPressed()){

        }
        if(driver1.left_bumper.justPressed()){

        }

        if(driver1.dpad_up.isPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_down.justPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_right.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_left.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_right.isPressed()){

        }


        if(driver2.right_bumper.isPressed()){

        }







    }

    @Override
    protected void end() {

    }
}
