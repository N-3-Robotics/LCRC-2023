package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.N3Robot
import org.firstinspires.ftc.teamcode.utilities.*

@TeleOp(name = "TeleOp")
class TeleOP: LinearOpMode() {
    override fun runOpMode() {
        val timer = ElapsedTime()
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val ROBOT = N3Robot(this)

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
            ROBOT.gamepadDrive(m)

            telemetry.addData("Robot Pose", ROBOT.currentPose.toString())
            ROBOT.update()
        }

        
    }

}