package org.firstinspires.ftc.teamcode.utilities.path.heading

import org.firstinspires.ftc.teamcode.utilities.path.heading.HeadingInterpolator
import org.firstinspires.ftc.teamcode.utilities.math.Angle

/**
 * Constant heading interpolator used for arbitrary holonomic translations.
 *
 * @param heading heading to maintain
 */
class ConstantInterpolator(val heading: Double) : HeadingInterpolator() {
    override fun internalGet(s: Double, t: Double): Double = Angle.norm(heading)

    override fun internalDeriv(s: Double, t: Double) = 0.0

    override fun internalSecondDeriv(s: Double, t: Double) = 0.0
}
