package org.firstinspires.ftc.teamcode.utilities.path.heading

import org.firstinspires.ftc.teamcode.utilities.path.ParametricCurve
import org.firstinspires.ftc.teamcode.utilities.path.QuinticPolynomial
import org.firstinspires.ftc.teamcode.utilities.path.heading.TangentInterpolator
import org.firstinspires.ftc.teamcode.utilities.math.Angle

/**
 * Spline heading interpolator for transitioning smoothly between headings without violating continuity (and hence
 * allowing for integration into longer profiles).
 *
 * @param startHeading start heading
 * @param endHeading end heading
 * @param startHeadingDeriv start heading deriv (advanced)
 * @param startHeadingSecondDeriv start heading second deriv (advanced)
 * @param endHeadingDeriv start heading deriv (advanced)
 * @param endHeadingSecondDeriv start heading second deriv (advanced)
 */
// note: the spline parameter is transformed linearly into a pseudo-arclength parameter
class SplineInterpolator @JvmOverloads constructor(
    private val startHeading: Double,
    private val endHeading: Double,
    private val startHeadingDeriv: Double? = null,
    private val startHeadingSecondDeriv: Double? = null,
    private val endHeadingDeriv: Double? = null,
    private val endHeadingSecondDeriv: Double? = null
) : HeadingInterpolator() {
    private val tangentInterpolator = TangentInterpolator()
    private lateinit var headingSpline: QuinticPolynomial

    override fun init(curve: ParametricCurve) {
        super.init(curve)

        tangentInterpolator.init(this.curve)

        val len = curve.length()

        val headingDelta = Angle.normDelta(endHeading - startHeading)

        headingSpline = QuinticPolynomial(
            0.0,
            (startHeadingDeriv ?: curve.tangentAngleDeriv(0.0, 0.0)) * len,
            (startHeadingSecondDeriv ?: curve.tangentAngleSecondDeriv(0.0, 0.0)) * len * len,
            headingDelta,
            (endHeadingDeriv ?: curve.tangentAngleDeriv(len, 1.0)) * len,
            (endHeadingSecondDeriv ?: curve.tangentAngleSecondDeriv(len, 1.0)) * len * len
        )
    }

    override fun internalGet(s: Double, t: Double) = Angle.norm(startHeading + headingSpline[s / curve.length()])

    override fun internalDeriv(s: Double, t: Double): Double {
        val len = curve.length()
        return headingSpline.deriv(s / len) / len
    }

    override fun internalSecondDeriv(s: Double, t: Double): Double {
        val len = curve.length()
        return headingSpline.secondDeriv(s / len) / (len * len)
    }
}
