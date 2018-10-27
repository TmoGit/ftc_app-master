package ftc.evlib.hardware.control;

import android.util.Log;

import java.util.Objects;

import ftc.electronvolts.util.Vector2D;
import ftc.electronvolts.util.units.Angle;
import ftc.electronvolts.util.units.Velocity;
import ftc.evlib.hardware.control.XYRControl;
import ftc.evlib.hardware.motors.MecanumMotors;
import ftc.evlib.util.StepTimer;

/**
 * This file was made by the electronVolts, FTC team 7393
 *  * Date Created: 9/19/16
 *  *
 *  * Manages what algorithms control the rotation and translation of the mecanum wheels
 * This allows you to mix and match rotation and translation (x and y) controllers and change them whenever you want
 *
 * @see MecanumMotors
 * @see RotationControl
 * @see RotationControls
 * @see TranslationControl
 * @see TranslationControls
 */
public class MecanumControl {
    /**
     * the motors to be controlled
     */
    private final MecanumMotors mecanumMotors;

    //the controllers for the rotation and translation
    private RotationControl rotationControl;
    private TranslationControl translationControl;

    /**
     * the velocity in the x, y, and rotation directions
     */
    private double velocityX, velocityY, velocityR;

    /**
     * stores whether or not the translation/rotation worked
     * for example, translationWorked will be false if the line following lost the line
     */
    private boolean translationWorked = true, rotationWorked = false;

    /**
     * create a MecanumControl that is not moving
     *
     * @param mecanumMotors the motors to control
     */
    public MecanumControl(MecanumMotors mecanumMotors) {
        this(mecanumMotors, ftc.evlib.hardware.control.XYRControls.ZERO);
    }

    /**
     * create a MecanumControl and immediately start using an XYR control
     *
     * @param mecanumMotors the motors to control
     * @param xyrControl    how to command the motors' rotation and translation
     */
    public MecanumControl(MecanumMotors mecanumMotors, ftc.evlib.hardware.control.XYRControl xyrControl) {
        this(mecanumMotors, xyrControl, xyrControl);
    }

    /**
     * create a MecanumControl and immediately start using TranslationControl and RotationControl
     *
     * @param mecanumMotors      the motors to control
     * @param rotationControl    how to command the motors' rotation
     * @param translationControl how to command the motors' translation
     */
    public MecanumControl(MecanumMotors mecanumMotors, RotationControl rotationControl, TranslationControl translationControl) {
        this.rotationControl = rotationControl;
        this.mecanumMotors = mecanumMotors;
        this.translationControl = translationControl;
    }

    /**
     * @return the robot's speed when the power is set to 100%
     */
    public Velocity getMaxRobotSpeed() {
        return mecanumMotors.getMaxRobotSpeed();
    }

    /**
     * @return the robot's speed when the power is set to 100% when driving sideways
     */
    public Velocity getMaxRobotSpeedSideways() {
        return mecanumMotors.getMaxRobotSpeedSideways();
    }

    /**
     * @param angle the angle to get the max speed at
     * @return the measured maximum speed of the robot at the given angle
     */
    public Velocity getMaxRobotSpeed(Angle angle) {
        return mecanumMotors.getMaxRobotSpeed(angle);
    }

    /**
     * @param rotationControl the controller that determines the robot's rotation
     */
    public void setRotationControl(RotationControl rotationControl) {
        this.rotationControl = rotationControl;
    }

    /**
     * @param translationControl the controller that determines the robot's translation (x and y)
     */
    public void setTranslationControl(TranslationControl translationControl) {
        this.translationControl = translationControl;
    }

    /**
     * set both translation and rotation controls at once
     *
     * @param translationControl the controller that determines the robot's translation (x and y)
     * @param rotationControl    the controller that determines the robot's rotation
     */
    public void setControl(TranslationControl translationControl, RotationControl rotationControl) {
        this.translationControl = translationControl;
        this.rotationControl = rotationControl;
    }

    /**
     * set both translation and rotation controls at once
     *
     * @param rotationControl    the controller that determines the robot's rotation
     * @param translationControl the controller that determines the robot's translation (x and y)
     */
    public void setControl(RotationControl rotationControl, TranslationControl translationControl) {
        this.translationControl = translationControl;
        this.rotationControl = rotationControl;
    }

    /**
     * Set both translation and rotation controls to the same object
     *
     * @param xyrControl the controller that determines both the translation and rotation
     */
    public void setControl(XYRControl xyrControl) {
        this.translationControl = xyrControl;
        this.rotationControl = xyrControl;
    }

    /**
     * stop the motors
     */
    public void stop() {
        translationControl = TranslationControls.ZERO;
        rotationControl = RotationControls.ZERO;
        mecanumMotors.stop();
    }

    /**
     * set the drive mode to turn on or off translation normalizing
     *
     * @param mode the drive mode
     */
    public void setDriveMode(MecanumMotors.MecanumDriveMode mode) {
        mecanumMotors.setDriveMode(mode);
    }

    /**
     * update the motor powers based on the output of the translationControl and rotationControl
     */
    private final StepTimer stepTimer = new StepTimer("mecanum", Log.VERBOSE);

    public void act() {
        stepTimer.start();
        stepTimer.step("translation");

        translationWorked = translationControl.act();

        //in case the same object is passed in that implements both controllers
        stepTimer.step("rotation");
        if (Objects.equals(translationControl, rotationControl)) {
            rotationWorked = translationWorked;
        } else {
            rotationWorked = rotationControl.act();
        }

        stepTimer.step("logic");
        Vector2D translation = translationControl.getTranslation();

        double velocity = translation.getLength();
        double directionRads = translation.getDirection().radians() +
                rotationControl.getPolarDirectionCorrection().radians();

        velocityX = velocity * Math.cos(directionRads);
        velocityY = velocity * Math.sin(directionRads);

        velocityR = rotationControl.getVelocityR();

        mecanumMotors.setVelocityXYR(
                velocityX,
                velocityY,
                velocityR
        );

        stepTimer.step("motorsDrive");

        mecanumMotors.mecanumDrive();
        stepTimer.step("motorsUpdate");

        mecanumMotors.update();
        stepTimer.step("position");

        if (positionTracker != null) positionTracker.act();
        stepTimer.stop();

    }

    private PositionTracker positionTracker = null;

    public PositionTracker getPositionTracker() {
        return positionTracker;
    }

    public void setPositionTracker(PositionTracker positionTracker) {
        this.positionTracker = positionTracker;
    }

    /**
     * @return whether or not the translation worked
     */
    public boolean translationWorked() {
        return translationWorked;
    }

    /**
     * @return whether or not the rotation worked
     */
    public boolean rotationWorked() {
        return rotationWorked;
    }

    /**
     * @return the robot's x velocity
     */
    public double getVelocityX() {
        return velocityX;
    }

    /**
     * @return the robot's y velocity
     */
    public double getVelocityY() {
        return velocityY;
    }

    /**
     * @return the robot's rotational velocity
     */
    public double getVelocityR() {
        return velocityR;
    }

    public RotationControl getRotationControl() {
        return rotationControl;
    }

    public TranslationControl getTranslationControl() {
        return translationControl;
    }

    public MecanumMotors getMecanumMotors() {
        return mecanumMotors;
    }
}