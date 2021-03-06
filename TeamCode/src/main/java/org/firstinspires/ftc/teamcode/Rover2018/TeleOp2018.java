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

    boolean isRunning = false;

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: 288 - Value for Rev motors
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 1.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double     DRIVE_SPEED             = 0.6;
    //static final double     TURN_SPEED              = 0.5;

    public double MOTOR_RUNSPEED = 1.0;
    public double LIFT_RUNSPEED = 1.0;
    public double SERVO_RUNSPEED_DOWN = 0.25;
    public double SERVO_RUNSPEED_UP   = 0.75;
    public double SERVO_STOP = 0.5;
    public double LIFT_POSITION = 8.0;
    public double ARM_NEUTRAL_POSITION = 2.5;
    public double ARM_CAPTURE_POSITION = 0.39;


    public double LSWEEPER_POWER = 0.0;
    public double RSWEEPER_POWER = 0.0;
    public double SWEEPER_POWER  = 0.3; // Changed from 0.5 to 0.3
    public double BUCKET_POWER   = 0.0;

    public boolean sweeperToggle = false;



    //  double lPos = 0.0;
    // double rPos = 0.0;
    double armPowerDif = 0.1;

    int controlState = 0;
    int SequencerStepsMax = 6;
    int Step_Bump = 5000;
    int armStartTime = 0;
    int armStartPos = 0;
    int armEndPos = 0;
    int armSpeedUp = 0;
    double armSpeed = 0;

    int armSpeedParamLow = 10;
    int armSpeedParamHigh = 15;

    int Current_Seq_Step = 0;
    boolean[] boolStep_Confirm = new boolean[SequencerStepsMax];
    boolean SequencerIsActive = false;
    boolean SequencerIsComplete = false;
    boolean Spin_Active_Flag = false;
    boolean armFirstrun = true;

    boolean bucketFirstrun = true;
    int bucketEndTime = 0;
    String lastBucketPos = "CAPTURE";
    boolean bucketDumpRunToggle = false;
    boolean bucketCaptureRunToggle = false;
    int bucketMoveTime = 500;

    private ElapsedTime runtime = new ElapsedTime();

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

    private boolean opModeIsActive(){ return isRunning; }

    public void sleep(long sleepTime)
    {
        long endTime = (int)runtime.milliseconds() + sleepTime;

        while(runtime.milliseconds() <= endTime) {}
    }


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
        leftX = new ScalingInputExtractor(driver1.right_stick_x, f);//Inverted turning inputs
        rightX = new ScalingInputExtractor(driver1.left_stick_x, f);
        //noinspection SuspiciousNameCombination
        robotCfg.getMecanumControl().setTranslationControl(TranslationControls.inputExtractorXY(rightY, rightX));
//        robotCfg.getMecanumControl().setRotationControl(RotationControls.teleOpGyro(leftX, robotCfg.getGyro()));
        robotCfg.getMecanumControl().setRotationControl(RotationControls.inputExtractor(leftX));


      /*  InputExtractor<Double> z = new InputExtractor<Double>() {
            @Override
            public Double getValue() {
                return -0.5;
            }
        };
        rightX = new ScalingInputExtractor(z, f);*/

    }

    private void Bump_Timer(int timeout){
        sleep(timeout);
    }

    private void Sequencer(int step_advance, boolean Status){
        if (Current_Seq_Step != SequencerStepsMax){
            SequencerIsComplete = false;
        }
        if (Current_Seq_Step == SequencerStepsMax+1){
            SequencerIsComplete = true;
            Current_Seq_Step = 0;
            //for (int i = 0; i < SequencerStepsMax; i++){
            //   boolStep_Confirm[i] = false;
            // }
            runtime.reset();
        }
        if (Status == true){
            SequencerIsActive = true;
        }
        if (Status == false){
            SequencerIsActive = false;

        }
        Current_Seq_Step = Current_Seq_Step + step_advance;
        Bump_Timer(Step_Bump);


    }



    private boolean Reset() {
/*
        Arm_Control(-MOTOR_RUNSPEED, -ARM_NEUTRAL_POSITION);
        Bump_Timer(5000);


        Lift_Control(-MOTOR_RUNSPEED,-LIFT_POSITION);
        Bump_Timer(5000);

        Bucket_Control("CAPTURE");
        Bump_Timer(5000);
*/
        return true;
    }



    private void Arm_Control(double power, double inchDist)
    {

        if(controlState == 0) {

            robotCfg.Motor_ArmBase.setPower(power);
            //  robotCfg.Motor_ArmBase2.setPower(-power);

            /*
            if(armFirstrun == true){
                armStartTime = (int) runtime.milliseconds();
                armStartPos = robotCfg.Motor_ArmBase.getCurrentPosition();

                armFirstrun = false;
            }
            else if(armStartTime + 50 <= runtime.milliseconds()){
                armEndPos = robotCfg.Motor_ArmBase.getCurrentPosition();
                int armEndTime = (int) runtime.milliseconds();

                int armTotalTime = armEndTime - armStartTime;
                int armDistance = armEndPos - armStartPos;

                armSpeed = armDistance / armTotalTime;

                if(armSpeed < armSpeedParamLow){
                    power = power + armPowerDif;
                    armSpeedUp = 1;
                }
                else if(armSpeed > armSpeedParamHigh){
                    power = power - armPowerDif;
                    armSpeedUp = 2;
                }

                armFirstrun = true;
            }
            else if(armStartTime + 50 > runtime.milliseconds()){
                if(armSpeedUp == 1){
                    power = power + armPowerDif;
                }
                else if(armSpeedUp == 2) {
                    power = power - armPowerDif;
                }
            }

            robotCfg.Motor_ArmBase.setPower(power);
            */
        }

        else if (controlState == 1){
            int newArmTarget = 0;

            newArmTarget = robotCfg.Motor_ArmBase.getCurrentPosition() + (int) (inchDist * COUNTS_PER_INCH);

            robotCfg.Motor_ArmBase.setTargetPosition(newArmTarget);

            // Turn On RUN_TO_POSITION
            robotCfg.Motor_ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Move to position
            //while (robotCfg.Motor_ArmBase.getCurrentPosition() != newArmTarget){
            robotCfg.Motor_ArmBase.setPower(power);
            // }

            //Bump_Timer(1000);

            if (robotCfg.Motor_ArmBase.getCurrentPosition() >= newArmTarget) {
                robotCfg.Motor_ArmBase.setPower(0);

            }
        }

        /*
        double f = currentSpeedFactor.getFactor();

        Arm_rightY = new ScalingInputExtractor(driver2.left_stick_y, f);
        robotCfg.Motor_ArmBase.setPower(Arm_rightY.getValue());

        telemetry.addData("Arm Movement: ", Arm_rightY.getValue());
        telemetry.update();
        */

    }

    private void Lift_Control(double power, double inchDist, boolean leftLift)
    {
        if(controlState == 0) {
            if(leftLift){
                robotCfg.Motor_LiftLeft.setPower(-power);
            }
            else {
                robotCfg.Motor_LiftRight.setPower(power);
            }
        }


        else if (controlState == 1) {
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
            //while (robotCfg.Motor_LiftLeft.getCurrentPosition() <= newLiftLeftTarget  && robotCfg.Motor_LiftRight.getCurrentPosition() <= newLiftRightTarget && opModeIsActive()){
            robotCfg.Motor_LiftLeft.setPower(-power);
            robotCfg.Motor_LiftRight.setPower(power);
            //}
            //if (robotCfg.Motor_LiftLeft.getCurrentPosition() >= newLiftLeftTarget && robotCfg.Motor_LiftRight.getCurrentPosition() >= newLiftRightTarget) {
            robotCfg.Motor_LiftRight.setPower(0);
            robotCfg.Motor_LiftLeft.setPower(0);

            //}

        }

        /*
        double f = currentSpeedFactor.getFactor();
        Lift_leftY = new ScalingInputExtractor(driver2.right_stick_y, f);
        robotCfg.Motor_LiftLeft.setPower(Lift_leftY.getValue());
        robotCfg.Motor_LiftLeft.setPower(-Lift_leftY.getValue());

        telemetry.addData("Lift Movement: ", Lift_leftY.getValue());
        telemetry.update();
       */
    }



    private void Manual_Bucket_Control(double power, boolean baseServo)
    {
        if(baseServo){
            //robotCfg.Servo_Out.setPower(power);
        }
        else if(!baseServo){
            // robotCfg.Out2.setPower(-power);
        }


    }

    private void Preset_Bucket_Control(String pos, int moveTime)
    {
        int currentTime = (int) runtime.milliseconds();

        if(bucketFirstrun){
            bucketEndTime = currentTime + moveTime;

            bucketFirstrun = false;
        }

        if (pos == "DUMP") {
            //robotCfg.Servo_Out.setPower(-1);
        }
        else if (pos == "CAPTURE") {
            // robotCfg.Servo_Out.setPower(1);
        }

        if(currentTime >= bucketEndTime){
            if(pos == "DUMP"){
                lastBucketPos = "DUMP";
            }
            if(pos == "CAPTURE"){
                lastBucketPos = "CAPTURE";
            }

            bucketEndTime = 0;
            bucketFirstrun = true;
        }

    }

    private void Arm_Retract_Control(double power){
        robotCfg.Motor_ArmBase.setPower(power);
    }

    private void Sweeper_Control(double power, int spinTime)
    {
        // int spinTime = 8000;

        // double lPos = robotCfg.Servo_InL.getPosition() + leftPower;
        // double rPos = robotCfg.Servo_InR.getPosition() + rightPower;



        //robotCfg.Servo_InR.setDirection();

        //   robotCfg.Servo_InL.setPosition(lPos);
        //   robotCfg.Servo_InR.setPosition(rPos);
        if(controlState == 0) {

            robotCfg.Motor_Sweeper.setPower(power);
            //robotCfg.Servo_InR.setPower(rightPower);
            //robotCfg.Servo_InL.setPower(-leftPower);
        } else if(controlState == 1){
            //run for a time

            int endTime = (int)runtime.milliseconds() + spinTime;

            //while(getRuntime() <= endTime && opModeIsActive()) {

            //robotCfg.Motor_Sweeper.setPower(power);

            Spin_Active_Flag = true;
            //}

            //robotCfg.Motor_Sweeper.setPower(0);
        }

        Spin_Active_Flag = false;
    }

    private void Dump_Auto_Sequence(String status) {
        if (status == "enable" && controlState == 1) {

            if (Current_Seq_Step == 0) {
                //Setup
                // if (Reset() == true) {
                Sequencer(1, true);
                // }
            }

        }


        if (Current_Seq_Step == 1 && boolStep_Confirm[0] == true) {

            Arm_Control(MOTOR_RUNSPEED, ARM_CAPTURE_POSITION);
            //Sequencer(1, true);

            boolStep_Confirm[Current_Seq_Step] = true;
        }
        if (Current_Seq_Step == 2 && boolStep_Confirm[1] == true) {
            Sweeper_Control(1, 8000);

            //Sequencer(1, true);

            boolStep_Confirm[Current_Seq_Step] = true;
        }
        if (Current_Seq_Step == 3 && boolStep_Confirm[2] == true) {
            Arm_Control(-MOTOR_RUNSPEED, -ARM_NEUTRAL_POSITION);
            //Sequencer(1, true);

            boolStep_Confirm[Current_Seq_Step] = true;
        }
        if (Current_Seq_Step == 4 && boolStep_Confirm[3] == true) {
            //  Lift_Control(MOTOR_RUNSPEED, LIFT_POSITION);
            //Sequencer(1, true);

            boolStep_Confirm[Current_Seq_Step] = true;
        }
        if (Current_Seq_Step == 5 && boolStep_Confirm[4] == true) {

            //Sequencer(1, false);
        }

        if (status == "disable") {
            controlState = 0;
        }

    }




    @Override
    protected void go() {
        isRunning = true;

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

        if (driver2.x.isPressed()) {
            //controlState = 1;

            // telemetry.addData("Robot Control State is semi-auto, set:", controlState);
            // telemetry.update();

            //Dump_Auto_Sequence("enable");

        }

        if (driver2.y.isPressed()) {
            //controlState = 0;

            // telemetry.addData("Robot Control State is semi-auto, set:", controlState);
            // telemetry.update();

            // Dump_Auto_Sequence("disable");

        }

        if(controlState == 0) {
            //Hanging Lift Control
            if (driver1.dpad_up.isPressed()) {
                Lift_Control(LIFT_RUNSPEED, 0, true);
                telemetry.addData("Left Lift Movement ", "Up");
                telemetry.update();
            } else if (driver1.dpad_down.isPressed()) {
                Lift_Control(-(LIFT_RUNSPEED), 0, true);
                telemetry.addData("Left Lift Movement ", "DOWN");
                telemetry.update();
            } else {
                Lift_Control(0.0, 0, true);
            }

            //Arm Control
            double armPower =(-1 * driver2.left_stick_y.getRawValue()) / 2;
            Arm_Control(armPower, 0);

            //Scoring Lift Control
            double rightLiftPower = driver2.right_stick_y.getRawValue();
            Lift_Control(rightLiftPower, 0, false);

            //Arm Retract Control
            // double armRetractPower = driver2.right_stick_y.getRawValue();
            // Arm_Retract_Control(armRetractPower);

        }

        //Sweeper Control
        if(driver2.y.justPressed() && sweeperToggle == false){
            sweeperToggle = true;
        } else if(driver2.y.justPressed() && sweeperToggle){
            sweeperToggle = false;
        }

        if (driver2.x.isPressed()) {
            Sweeper_Control(-SWEEPER_POWER, 0);

        } else if (sweeperToggle) {
            Sweeper_Control(SWEEPER_POWER, 0);

        } else {
            Sweeper_Control(0,0);
        }


        //telemetry.addData("\nBucket Positon", robotCfg.Servo_Out.getPosition());
        //telemetry.addData("\nLeft Stick Input", driver2.left_stick_y.getRawValue());
        //telemetry.addData("\nRight Stick Input", driver2.right_stick_y.getRawValue());
        //telemetry.addData("Current Arm Position: ", robotCfg.Motor_ArmBase.getCurrentPosition());
        //telemetry.addData("Current Left Lift Position: ", robotCfg.Motor_LiftLeft.getCurrentPosition());
        //  telemetry.addData("Current Right Lift Position: ", robotCfg.Motor_LiftRight.getCurrentPosition());
        //telemetry.addData("Current Motor_FL Position: ", robotCfg.Motor_WheelFL.getCurrentPosition());
        //telemetry.addData("Current Motor_FR Position: ", robotCfg.Motor_WheelFR.getCurrentPosition());
        //telemetry.addData("Current Motor_BL Position: ", robotCfg.Motor_WheelBL.getCurrentPosition());
        //telemetry.addData("Current Motor_BR Position: ", robotCfg.Motor_WheelBR.getCurrentPosition());
        //telemetry.addData("Current Arm Speed: ", armSpeed);


        if (controlState == 1) {



            telemetry.addData("Current Sequencer Step:", Current_Seq_Step);
            telemetry.addData("Sweepers are running?", Spin_Active_Flag);

        }
        else if (controlState == 0) {

        }
        /*
        telemetry.addData("\nLeft Servo Pos", robotCfg.Servo_InL.getPosition());
        telemetry.addData("\nRight Servo Pos", robotCfg.Servo_InR.getPosition());
        telemetry.addData("\nLeft Servo Dir", robotCfg.Servo_InL.getDirection());
        telemetry.addData("\nRight Servo Dir", robotCfg.Servo_InR.getDirection());
        */
        telemetry.update();


      /*  if(driver1.dpad_up.isPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_down.justPressed() && !driver1.dpad_right.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_right.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_left.isPressed()){

        }
        else if(driver1.dpad_left.justPressed() && !driver1.dpad_down.isPressed() && !driver1.dpad_right.isPressed()){

        }*/



    }

    @Override
    protected void end() {
        isRunning = false;
    }
}

