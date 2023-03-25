package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.utilities.Component

class Launcher(val robot: N3Robot): Component {

    private var motor: DcMotorEx = robot.getMotor("LAUNCHER")

    var power: Double
        get() = motor.power
        set(value) {
            motor.power = value
        }

    override fun update() {
        if (robot.opmode.gamepad1.right_trigger.toDouble() > 0.0) {
            power = calculatePower(0.0)
        }
    }

    private fun calculatePower(distance: Double): Double {
        return 0.0 // #TODO: Implement
    }

}