@file:Suppress("unused")
package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.robomatic.util.PIDController
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.HoldingPower
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.SlidesTolerance
import kotlin.math.abs


/**
 * A class for controlling the slides of the robot.
 * @param motor The slides motor.
 */

class Slides(private val motor: DcMotorEx) {
    var state = State.STOPPED

    private var correction: Double = 0.0
    private var error: Double = 0.0

    private var slidesPIDController = PIDController(0.5, 0.0, 0.0)

    enum class State{
        MOVING, STOPPED
    }

    /**
     * Slides run mode.
     *
     * Default: **RUN_USING_ENCODER**
     */
    var mode: DcMotor.RunMode
        get() = motor.mode
        set(value) {
            motor.mode = value
        }

    /**
     * Slides motor power
     */
    var power: Double
        get() = motor.power
        set(value) {
            motor.power = value
        }

    /**
     * Slides zeroPowerBehavier
     *
     * Default: **BRAKE**
     */
    var zeroPowerBehavior: DcMotor.ZeroPowerBehavior
        get() = motor.zeroPowerBehavior
        set(value) {
            motor.zeroPowerBehavior = value
        }

    /**
     * Slides current Position
     */
    val currentPosition: Double
        get() = motor.currentPosition.toDouble()

    /**
     * Slides current velocity
     */
    val currentVelocity: Double
        get() = motor.velocity

    /**
     * Slides target position
     */
    var targetPosition = 0

    /**
     * Stops the slides.
     */
    fun stop(){
        motor.power = HoldingPower
        state = State.STOPPED
    }

    /**
     * Moves the slides to a target position.
     * @param targetPosition The target position.
     */
    fun goTo(position: Int) {
        state = State.MOVING
        motor.targetPosition = position
        update()
    }

    /**
     * Updates the slides depending on the state.
     */
    fun update(){
        when (state) {
            State.MOVING -> {
                while (abs(error) > SlidesTolerance) {
                    error = targetPosition - currentPosition
                    correction = slidesPIDController.update(error)
                    power = correction
                }
                stop()
            }
            State.STOPPED -> {
                power = HoldingPower
            }
        }
    }

    init {
        slidesPIDController.setOutputBounds(-1.0, 1.0)
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_USING_ENCODER
    }


}