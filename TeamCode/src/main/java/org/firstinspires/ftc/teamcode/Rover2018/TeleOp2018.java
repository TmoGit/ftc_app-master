package org.firstinspires.ftc.teamcode.Rover2018;


import com.google.common.collect.ImmutableList;
import com.google.common.io.BaseEncoding;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Rover2018.RobotCfg2018;

import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.files.Logger;
import ftc.evlib.hardware.control.RotationControls;
import ftc.evlib.hardware.control.TranslationControls;
import ftc.evlib.opmodes.AbstractTeleOp;





/**
 * This file was made by Cut The Red Wire, FTC team 6078
 */
@TeleOp(name = "TeleOp2018", group = "Official")
public class TeleOp2018 extends AbstractTeleOp<RobotCfg2018> {

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
    private MotorSpeedFactor currentSpeedFactor = MotorSpeedFactor.FAST;

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
  RobotCfg2018 robot  = new RobotCfg2018(hardwareMap);
      return robot;
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

    private void Arm_Control()
    {
        //Arm Variables
        double Arm_Down = 0.0;
        double Arm_Up = 0.0;



        Arm_Down = driver2.left_trigger.getRawValue();
        Arm_Up = driver2.right_trigger.getRawValue();

        createRobotCfg().Motor_ArmBase.setPower(Arm_Down);
        createRobotCfg().Motor_ArmBase.setPower(-Arm_Up);
    }

    @Override
    protected void go() {
        forwardControl();
        Arm_Control();
    }

    @Override
    protected void act() {



        if(driver1.a.justPressed()) {
            if (currentSpeedFactor == MotorSpeedFactor.FAST) {
                currentSpeedFactor = MotorSpeedFactor.SLOW;
            } else {
                currentSpeedFactor = MotorSpeedFactor.FAST;
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
