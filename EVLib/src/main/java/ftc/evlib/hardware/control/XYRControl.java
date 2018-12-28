package ftc.evlib.hardware.control;

import ftc.electronvolts.util.units.Angle;
import ftc.evlib.hardware.control.RotationControl;
import ftc.evlib.hardware.control.TranslationControl;
import ftc.evlib.hardware.control.XYRControls;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 12/14/16
 *
 * Controls both the translation (x and y) and the rotation (r) of a robot with mecanum wheels
 *
 * @see RotationControl
 * @see TranslationControl
 * @see XYRControls
 */
public abstract class XYRControl implements RotationControl, TranslationControl {
    @Override
    public Angle getPolarDirectionCorrection() {
        return Angle.zero();
    }
}
