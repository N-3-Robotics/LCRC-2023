package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.utilities.*
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class TeleOP: LinearOpMode() {
    override fun runOpMode() {

        PhotonCore.enable()

        val timer = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)



        val ROBOT = Robot(hardwareMap)

        var m = 1.0

        while (!opModeIsActive()){
            ROBOT.rumble(gamepad1, Side.BOTH, RumbleStrength.HIGH)
            ROBOT.rumble(gamepad2, Side.BOTH, RumbleStrength.HIGH)
        }

        waitForStart()

        while (opModeIsActive()){
            telemetry.addData("Loop Time", timer.milliseconds())
            timer.reset()



            // Drivetrain Control
            ROBOT.gamepadDrive(gamepad1, m)

            telemetry.addData("Robot Pose", ROBOT.currentPose.toString())
            telemetry.update()
        }

        
    }

}