package org.firstinspires.ftc.teamcode.Rover2018;

import android.util.Log;

import com.google.common.collect.ImmutableList;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Map;

import ftc.electronvolts.statemachine.StateName;
import ftc.electronvolts.util.InputExtractor;
import ftc.electronvolts.util.InputExtractors;
import ftc.electronvolts.util.TeamColor;
import ftc.electronvolts.util.files.BasicConverters;
import ftc.electronvolts.util.files.Converters;
import ftc.electronvolts.util.files.Logger;
import ftc.electronvolts.util.files.OptionsFile;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Distance;
import ftc.electronvolts.util.units.Time;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.config.RobotCfg;
import ftc.evlib.hardware.control.MecanumControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import com.qualcomm.robotcore.hardware.ColorSensor;
import ftc.evlib.hardware.motors.Motors;
import ftc.evlib.hardware.sensors.Sensors;
import ftc.evlib.hardware.sensors.SpikeDetector;
import ftc.evlib.hardware.servos.ServoCfg;
import ftc.evlib.hardware.servos.ServoControl;
import ftc.evlib.hardware.servos.ServoName;
import ftc.evlib.hardware.servos.Servos;
import ftc.evlib.statemachine.EVStateMachineBuilder;
import ftc.evlib.util.EVConverters;
import ftc.evlib.util.FileUtil;
import ftc.evlib.hardware.sensors.Gyro;
import ftc.evlib.util.StepTimer;

import static ftc.evlib.vision.framegrabber.GlobalFrameGrabber.frameGrabber;

/**
 * This file was made by the electronVolts, FTC team 7393
 *  * Date Created: 9/2/17
 */

public class RobotCfg2018 extends RobotCfg {
    private final MecanumControl mecanumControl;
    private static final Velocity MAX_ROBOT_SPEED = new Velocity(Distance.fromInches(57 * 4), Time.fromSeconds(2.83));
    private static final Velocity MAX_ROBOT_SPEED_SIDEWAYS = new Velocity(Distance.fromInches(21.2441207039), Time.fromSeconds(1));


    private final Servos servos;
    private static final String GYRO_SENSOR_NAME = "gyro";
    private static final String COLOR_SENSOR_NAME = "sensor_color";
    private final ColorSensor colorSensor;
    private final SpikeDetector rightLineSensor, leftLineSensor;
    private static final String RIGHT_LINE_SENSOR_NAME = "l1";
    private static final String LEFT_LINE_SENSOR_NAME = "l2";
    private static final double LINE_SENSOR_SPIKE_THRESHOLD = 1.05; //1.1; //1.25;

    private static final int LINE_SENSOR_LONG_READINGS = 50;
    private static final int LINE_SENSOR_SHORT_READINGS = 3;
    private final List<Logger.Column> loggerColumns;

    //Defining Motors
    public DcMotor Motor_ArmBase = null;
    public DcMotor Motor_LiftLeft = null;
    public DcMotor Motor_LiftRight = null;
    public DcMotor Motor_WheelFL = null;
    public DcMotor Motor_WheelFR = null;
    public DcMotor Motor_WheelBL = null;
    public DcMotor Motor_WheelBR = null;





//    public MainRobotCfg(HardwareMap hardwareMap) {
//        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(MainServoName.values()));
//    }

    public enum LeftReleaseServoPresets{
        OPENED,
        CLOSED,
        GRAB,

    }
    public enum RightReleaseServoPresets{
        OPENED,
        CLOSED,
        GRAB,

    }
    public enum SensorServoPresets{
        UP,
        DOWN
    }
    public enum relicServoPresets{
        OPEN,
        GRAB
    }
    public enum MainServoName implements ServoName{
        LEFTRELEASE("s0",LeftReleaseServoPresets.values()),
        RIGHTRELEASE("s2",RightReleaseServoPresets.values()),
        SENSOR("s1",SensorServoPresets.values()),
        GRAB("grab",relicServoPresets.values());
        private final String hardwareName;
        private final Enum[] presets;

        MainServoName(String hardwareName, Enum[] presets) {
            this.hardwareName = hardwareName;
            this.presets = presets;
        }

        @Override
        public String getHardwareName() {
            return hardwareName;
        }

        @Override
        public Enum[] getPresets() {
            return presets;
        }

        @Override
        public Class<? extends RobotCfg> getRobotCfg() {
            return RobotCfg2018.class;
        }
    }
    public RobotCfg2018(HardwareMap hardwareMap){
        this(hardwareMap, ServoCfg.defaultServoStartPresetMap(MainServoName.values()));
    }


    public RobotCfg2018(HardwareMap hardwareMap, Map<ServoName, Enum> servoStartPresetMap) {
        super(hardwareMap);
        colorSensor = hardwareMap.colorSensor.get(COLOR_SENSOR_NAME);

        servos=new Servos(ServoCfg.createServoMap(hardwareMap, servoStartPresetMap));

        //This is where we intilize the motors

        Motor_WheelFL = hardwareMap.get(DcMotor.class, "Motor_WheelFL");
        Motor_WheelFR = hardwareMap.get(DcMotor.class, "Motor_WheelFR");
        Motor_WheelBL = hardwareMap.get(DcMotor.class, "Motor_WheelBL ");
        Motor_WheelBR = hardwareMap.get(DcMotor.class, "Motor_WheelBR");
        Motor_ArmBase = hardwareMap.get(DcMotor.class, "Motor_ArmBase");
        Motor_LiftLeft = hardwareMap.get(DcMotor.class, "Motor_LiftLeft");
        Motor_LiftRight = hardwareMap.get(DcMotor.class, "Motor_LiftRight");

        Motor_ArmBase.setPower(0.0);
        Motor_ArmBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//         imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        Converters converters = EVConverters.getInstance();
        OptionsFile optionsFile = new OptionsFile(converters, FileUtil.getOptionsFile("teleop_options.txt"));
        double scaleFactor = 1.0; // optionsFile.get("mratio",Double.class);


        leftLineSensor = Sensors.spikeDetector(hardwareMap, LEFT_LINE_SENSOR_NAME, LINE_SENSOR_SPIKE_THRESHOLD, LINE_SENSOR_LONG_READINGS, LINE_SENSOR_SHORT_READINGS);
        rightLineSensor = Sensors.spikeDetector(hardwareMap, RIGHT_LINE_SENSOR_NAME, LINE_SENSOR_SPIKE_THRESHOLD, LINE_SENSOR_LONG_READINGS, LINE_SENSOR_SHORT_READINGS);


        //Motors setup
        ServoControl colorServo = getServo(MainServoName.SENSOR);
        mecanumControl = new MecanumControl(new MecanumMotors(
                Motors.withEncoder(Motor_WheelFL, true, true, stoppers),
                Motors.withEncoder(Motor_WheelFR, false, true, stoppers),
                Motors.scale(Motors.withEncoder(Motor_WheelBL , true, true, stoppers),scaleFactor),
                Motors.scale(Motors.withEncoder(Motor_WheelBR , false, true, stoppers),scaleFactor),
                true, MAX_ROBOT_SPEED,MAX_ROBOT_SPEED_SIDEWAYS));



        loggerColumns = ImmutableList.of(
                //robot motion
                new Logger.Column("velocityX", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getVelocityX();
                    }
                }),
                new Logger.Column("velocityY", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getVelocityY();
                    }
                }),
                new Logger.Column("velocityR", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getVelocityR();
                    }
                }),
                new Logger.Column("scaleFactor", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return mecanumControl.getMecanumMotors().getScaleFactor();
                    }
                }),


                //servos
                new Logger.Column("LEFTRELEASE", getServos().servoIE(MainServoName.LEFTRELEASE)),
                new Logger.Column("RIGHTRELEASE", getServos().servoIE(MainServoName.RIGHTRELEASE)),
                new Logger.Column("SENSOR", getServos().servoIE(MainServoName.SENSOR)),

//                new Logger.Column("distanceSensor", InputExtractors.format("%10f", distanceSensor)),

                //shooter
//                new Logger.Column("shooterSwitchInv", InputExtractors.booleanToIntIE(shooterSwitchInv)),
//                new Logger.Column("shooterSwitchBoth", InputExtractors.booleanToIntIE(shooterSwitchBoth)),



                //sensors
                new Logger.Column("leftLineSensorRaw", leftLineSensor.getRawSensor()),
                new Logger.Column("rightLineSensorRaw", rightLineSensor.getRawSensor()),
                new Logger.Column("leftLineSensor", InputExtractors.booleanToIntIE(leftLineSensor)),
                new Logger.Column("rightLineSensor", InputExtractors.booleanToIntIE(rightLineSensor)),
                new Logger.Column("frontLineSensor.ready", InputExtractors.booleanToIntIE(new InputExtractor<Boolean>() {
                    @Override
                    public Boolean getValue() {
                        return leftLineSensor.isReady();
                    }
                })),
                new Logger.Column("rightLineSensor.ready", InputExtractors.booleanToIntIE(new InputExtractor<Boolean>() {
                    @Override
                    public Boolean getValue() {
                        return rightLineSensor.isReady();
                    }
                })),
                new Logger.Column("leftLineSensor.longAverage", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return leftLineSensor.getLongAverage();
                    }
                }),
                new Logger.Column("rightLineSensor.longAverage", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return rightLineSensor.getLongAverage();
                    }
                }),
                new Logger.Column("leftLineSensor.shortAverage", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return leftLineSensor.getShortAverage();
                    }
                }),
                new Logger.Column("rightLineSensor.shortAverage", new InputExtractor<Double>() {
                    @Override
                    public Double getValue() {
                        return rightLineSensor.getShortAverage();
                    }
                }));


//
//





}
    public Servos getServos() {
        return servos;
    }

    public MecanumControl getMecanumControl() {
        return mecanumControl;
    }
    public SpikeDetector getRightLineSensor() {
        return rightLineSensor;
    }

    public SpikeDetector getLeftLineSensor() {return leftLineSensor;}



    @Override
    public void start() {

    }
    private final StepTimer stepTimer = new StepTimer("robotCfg", Log.VERBOSE);

    @Override
    public void act() {

        stepTimer.start();
        stepTimer.step("line sensors");
        leftLineSensor.act();
        rightLineSensor.act();
        stepTimer.step("mecanumControl");
        mecanumControl.act();
        //stepTimer.step("gyro");
        stepTimer.step("grabber");


        stepTimer.stop();
//        gyro.update();
//        leftLineSensor.act();
//        rightLineSensor.act();
//
//        mecanumControl.act();
//        grabber.act();


    }
    @Override
    public void stop() {
        mecanumControl.stop();

    }



}
