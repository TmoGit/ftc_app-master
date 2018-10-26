package ftc.evlib.hardware.control;

import ftc.electronvolts.util.Vector2D;
import ftc.evlib.hardware.control.TranslationControls;

/**
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 9/19/16
 *
 * Controls the translation of a mecanum robot
 *
 * @see TranslationControls
 * @see Vector2D
 */
public interface TranslationControl {
    /**
     * update the velocity x and velocity y values
     *
     * @return true if there were no problems
     */
    boolean act();

    /**
     * @return the translational velocity
     */
    Vector2D getTranslation();
}
