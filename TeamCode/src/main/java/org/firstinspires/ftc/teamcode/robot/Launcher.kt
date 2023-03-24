package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotorEx

class Launcher(private val motor: DcMotorEx): Component {

    var power: Double
        get() = motor.power
        set(value) {
            motor.power = value
        }

    override fun update() {
        this.power = calculatePower(0.0) // #TODO: Implement
        motor.power = this.power
    }

    private fun calculatePower(distance: Double): Double {
        return 0.0 // #TODO: Implement
    }
}