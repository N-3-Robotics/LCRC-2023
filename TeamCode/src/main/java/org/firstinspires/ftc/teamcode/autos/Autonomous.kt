package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.N3Robot

@Autonomous(name = "Autonomous")
class Autonomous: LinearOpMode() {
    override fun runOpMode() {

        val ROBOT = N3Robot(this)

        waitForStart()


        ROBOT.straight(10.0)
        ROBOT.turnRight()
        ROBOT.straight(10.0)
        ROBOT.turnLeft()
        ROBOT.straight(10.0)
    }

}