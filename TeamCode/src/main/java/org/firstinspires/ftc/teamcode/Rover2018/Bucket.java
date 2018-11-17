package org.firstinspires.ftc.teamcode.Rover2018;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.electronvolts.util.DeadZone;
import ftc.electronvolts.util.DigitalInputEdgeDetector;
import ftc.electronvolts.util.Function;
import ftc.electronvolts.util.Functions;
import ftc.electronvolts.util.PIDController;
import ftc.evlib.hardware.motors.MotorEnc;
import ftc.evlib.hardware.sensors.DigitalSensor;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.util.StepTimer;

/**
 * Created by ftc7393 on 12/30/2017.
 */

public class Bucket {

    public Bucket(ServoControl leftIntake, ServoControl rightIntake) {

        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;

       // this.leftRelease = leftRelease;
      //  this.rightRelease = rightRelease;
      // this.topLimit = new DigitalInputEdgeDetector(topLimit);
       // this.bottomLimit = new DigitalInputEdgeDetector(bottomLimit);
       // this.liftPID = new PIDController(0.003,0,0,1.0);


    }





   /* public enum Mode {//only reason public debugging in teleop
        UP,//release at top
        GO_UP,
        DOWN,//grab or release at bottom
        GO_DOWN,
        MIDDLE,
        GO_MIDDLE,
        GO_LITTLE_UP,
        LITTLE_UP,
        END_GAME
    }
    */

    private enum ServoAction {
        //OPEN,
       // GRAB,
       // CLOSED
        FORWARD,
        BACKWARD,
    }



   // private Mode mode = Mode.DOWN;//at the end of auto, it is at DH
  // private ServoAction servoActionBackward = ServoAction.FORWARD;
    private ServoAction servoAction;
//    private ServoAction lastServoAction = null;



   // private final ServoControl leftRelease;
   // private final ServoControl rightRelease;
    private final ServoControl leftIntake;
    private final ServoControl rightIntake;

  // private final DigitalInputEdgeDetector topLimit, bottomLimit;
    private PIDController liftPID;
  //  private static final double LIFT_SPEED = 1.5;//need to find value
 //  private  double liftSpeed = 0;//need to find value

 //   private static final double DOWN_LIFT_SPEED = .25;
//    private static final double SERVO_SPEED=2;
//   private static final double LEFT_SERVO_SPEED=.5;

 //   private static final int LIFT_ENCODER = 260;//need to find value

  //  private static final int UPMID_ENCODER = 800;
 //   private static final int LITTLE_UP = 600;
  //  private  double setPoint = 0;


  //  private boolean topPressed = false, bottomPressed = false;
  //  private double liftEncoder = 0;

  //  public double getLiftEncoderValue() {
  //      return liftEncoder;
   // }
  //  public double getSetPoint(){return setPoint;}


   // public boolean getTopLimit() {
  //      return topPressed;
  //  }

 //   public boolean getBottomLimit() {
 //       return bottomPressed;
 //   }

  //  private final StepTimer stepTimer = new StepTimer("grabber", Log.VERBOSE);

    public void act() {

        if(servoAction == ServoAction.FORWARD) {
             
        }

/*
        stepTimer.start();
        stepTimer.step("topLimit");

        topLimit.update();
        stepTimer.step("bottomLimit");

        bottomLimit.update();
        stepTimer.step("logic");


        liftEncoder = lift.getEncoderPosition();

        topPressed = topLimit.isPressed();
        bottomPressed = bottomLimit.isPressed();

        if(mode==Mode.UP||mode==Mode.MIDDLE||mode==Mode.LITTLE_UP){
            lift.setPower(liftPID.computeCorrection(setPoint,liftEncoder));
        }
        else if(mode==Mode.DOWN){
            lift.resetEncoder();
            lift.setPower(0);
        }

        else if(mode==Mode.GO_MIDDLE){

            setPoint=UPMID_ENCODER;
            mode=Mode.MIDDLE;
//            if(lift.getEncoderPosition())
//            if(lift.getEncoderPosition()>UPMID_ENCODER){
//            lift.setSpeed(-LIFT_SPEED);}
//            else if(lift.getEncoderPosition()<UPMID_ENCODER){
//                lift.setSpeed(LIFT_SPEED);
//            }
//            else if (lift.getEncoderPosition()  UPMID_ENCODER) {
//                setPoint=lift.getEncoderPosition();
//                mode=Mode.MIDDLE;
//            }
        }
        else if(mode==Mode.GO_UP){
            lift.setPower(LIFT_SPEED);
            if (topLimit.isPressed()) {
                setPoint=lift.getEncoderPosition();
                mode = Mode.UP;
            }
        }
        else if(mode==Mode.GO_DOWN){
            lift.setPower(-DOWN_LIFT_SPEED);
            if (bottomLimit.isPressed()) {
                lift.setPower(0);
                mode = Mode.DOWN;
            }
        }
        else if(mode==Mode.GO_LITTLE_UP){
            lift.setPower(LIFT_SPEED);
            setPoint=LITTLE_UP;
            mode=Mode.LITTLE_UP;
        }



        else{
            mode=Mode.DOWN;
        }

        if(servoAction!=lastServoAction) {
            lastServoAction=servoAction;

            if (servoAction == ServoAction.CLOSED) {
                leftRelease.goToPreset(RobotCfg2017.LeftReleaseServoPresets.CLOSED, LEFT_SERVO_SPEED);
                rightRelease.goToPreset(RobotCfg2017.RightReleaseServoPresets.CLOSED, SERVO_SPEED);

            } else if (servoAction == ServoAction.OPEN) {
                leftRelease.goToPreset(RobotCfg2017.LeftReleaseServoPresets.OPENED, SERVO_SPEED);
                rightRelease.goToPreset(RobotCfg2017.RightReleaseServoPresets.OPENED, SERVO_SPEED);

            } else if (servoAction == ServoAction.GRAB) {
                leftRelease.goToPreset(RobotCfg2017.LeftReleaseServoPresets.GRAB, SERVO_SPEED);
                rightRelease.goToPreset(RobotCfg2017.RightReleaseServoPresets.GRAB, SERVO_SPEED);

            }
        }


        stepTimer.step("lift");

        lift.update();


        stepTimer.stop();



    }


    public void goUp() {
        mode = Mode.GO_UP;
    }

    public void goDown() {mode = Mode.GO_DOWN;}
    public void goLittleUp() {mode = Mode.GO_LITTLE_UP;}

    public void goMiddle(){mode=Mode.GO_MIDDLE;}



    public Mode getMode() {
        return mode;
    }
    public ServoAction getServo() {
        return servoAction;
    }



    public void openServo() {
        servoAction=ServoAction.OPEN;
    }



    public void closeServo() {
        servoAction = ServoAction.CLOSED;
    }
    public void grabServo() {
        servoAction = ServoAction.GRAB;
    }
    public void toggleServo() {
        if (servoAction == ServoAction.OPEN ) {
            grabServo();
        } else  {
            openServo();
        }


*/
    }









}