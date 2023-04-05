package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utilities.Component

class Intake(val robot: N3Robot): Component {

    private var SpinMotor: DcMotorEx = robot.getMotor("SPINNER")
    private var LiftMotor: DcMotorEx = robot.getMotor("LIFT")
    private var TiltServo: Servo = robot.getServo("TILT");

    private enum class LiftState{
        REST, LIFTING, TILTING, LOWERING
    }


    override fun update() {
        if (robot.opmode.gamepad1.right_trigger.toDouble() > 0.0) {
            SpinMotor.power = 1.0
        }
    }

    init {
        LiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        LiftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

}