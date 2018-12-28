package org.firstinspires.ftc.teamcode.Rover2018;


import com.google.common.collect.ImmutableList;
import com.google.common.io.BaseEncoding;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

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
    ScalingInputExtractor Arm_rightY;
    ScalingInputExtractor Lift_leftY;

    public double MOTOR_RUNSPEED = 0.5;
    public double LSWEEPER_POWER = 0.0;
    public double RSWEEPER_POWER = 0.0;

    double lPos = 0.0;
    double rPos = 0.0;

    short constrolState =2, stepCounter =0;

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
        rightY = new ScalingInputExtractor(InputExtractors.negative(driver1.left_stick_y), f);
        leftX = new ScalingInputExtractor(InputExtractors.negative(driver1.right_stick_x), f);
        rightX = new ScalingInputExtractor(driver1.left_stick_x, f);
        //noinspection SuspiciousNameCombination
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));


       // Arm_Control();
      //  Lift_Control();
    }
    private void Bump_Timer(){

    }

    private void Sequence(){


    }

    private boolean Reset(){
        Arm_Control(0);
        Lift_Control(0);
        return true;
    }


    private void Arm_Control(double power)
    {
        robotCfg.Motor_ArmBase.setPower(power);
        /*
        double f = currentSpeedFactor.getFactor();

        Arm_rightY = new ScalingInputExtractor(driver2.left_stick_y, f);
        robotCfg.Motor_ArmBase.setPower(Arm_rightY.getValue());

        telemetry.addData("Arm Movement: ", Arm_rightY.getValue());
        telemetry.update();
        */

    }

    private void Lift_Control(double power)
    {
        robotCfg.Motor_LiftRight.setPower(-power);
        robotCfg.Motor_LiftLeft.setPower(power);

        /*
        double f = currentSpeedFactor.getFactor();
        Lift_leftY = new ScalingInputExtractor(driver2.right_stick_y, f);
        robotCfg.Motor_LiftLeft.setPower(Lift_leftY.getValue());
        robotCfg.Motor_LiftLeft.setPower(-Lift_leftY.getValue());

        telemetry.addData("Lift Movement: ", Lift_leftY.getValue());
        telemetry.update();
       */
    }
    private void Bucket_Control(String pos)
    {
        double dumpPos = 0.5, upPos = 0.0, bucketPos = upPos;


        if (pos == "DUMP") {
            bucketPos = Range.clip(dumpPos, upPos, dumpPos);
        }
        else if (pos == "UP") {
            bucketPos = Range.clip(upPos, upPos, dumpPos);
        }

        robotCfg.Servo_Out.setPosition(bucketPos);
    }

    private void Sweeper_Control(double leftPower, double rightPower)
    {
       // double lPos = robotCfg.Servo_InL.getPosition() + leftPower;
       // double rPos = robotCfg.Servo_InR.getPosition() + rightPower;



        //robotCfg.Servo_InR.setDirection();

     //   robotCfg.Servo_InL.setPosition(lPos);
     //   robotCfg.Servo_InR.setPosition(rPos);
    }
    @Override
    protected void go() {
        forwardControl();

    }

    @Override
    protected void act() {


/*
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
*/




        //Arm control
        if(driver2.right_bumper.isPressed()){
            Arm_Control(MOTOR_RUNSPEED);
            telemetry.addData("Arm Movement ", "UP");
            telemetry.update();
        }
        else if(driver2.left_bumper.isPressed()){
            Arm_Control(-MOTOR_RUNSPEED);
            telemetry.addData("Arm Movement ", "DOWN");
            telemetry.update();
        }
        else {
            Arm_Control(0.0);
        }

        //Lift Control
        if(driver2.dpad_up.isPressed()) {
            Lift_Control(MOTOR_RUNSPEED);
            telemetry.addData("Lift Movement ", "Up");
            telemetry.update();
        }
        else if(driver2.dpad_down.isPressed()) {
            Lift_Control(-MOTOR_RUNSPEED);
            telemetry.addData("Lift Movement ", "DOWN");
            telemetry.update();
        }
        else {
            Lift_Control(0.0);
        }

        //Bucket Control
        if(driver2.right_bumper.isPressed()){
            Bucket_Control("UP");
        }

        if(driver2.left_bumper.isPressed()){
            Bucket_Control("DUMP");
        }

        //Sweeper Control
        if(driver2.left_stick_y.getRawValue() >= 10){
            LSWEEPER_POWER =  1;
        }
        else if(driver2.left_stick_y.getRawValue() <= -10){
            LSWEEPER_POWER = -1;
        }
        else{
            LSWEEPER_POWER =  0;
        }


        if(driver2.right_stick_y.getRawValue() >= 10){
            RSWEEPER_POWER =  1;
        }
        else if(driver2.right_stick_y.getRawValue() <= -10){
            RSWEEPER_POWER = -1;
        }
        else{
            RSWEEPER_POWER =  0;
        }

        Sweeper_Control(LSWEEPER_POWER, RSWEEPER_POWER);

        telemetry.addData("\nLeft Stick Input", driver2.left_stick_y.getRawValue());
        telemetry.addData("\nRight Stick Input", driver2.right_stick_y.getRawValue());
        /*
        telemetry.addData("\nLeft Servo Pos", robotCfg.Servo_InL.getPosition());
        telemetry.addData("\nRight Servo Pos", robotCfg.Servo_InR.getPosition());
        telemetry.addData("\nLeft Servo Dir", robotCfg.Servo_InL.getDirection());
        telemetry.addData("\nRight Servo Dir", robotCfg.Servo_InR.getDirection());
        */
        telemetry.update();


        if(driver1.dpad_up.isPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_down.justPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_right.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_left.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_right.isPressed()){

        }



    }

    @Override
    protected void end() {

    }
}
