package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utilities.Robot

// inherit from the Robot class, taking an opmode parameter, and initialize itself using the opmode
class N3Robot(opmode: LinearOpMode): Robot(opmode) {

    val LAUNCHER: Launcher = Launcher(this)

    init {
        components.add(LAUNCHER)
    }

}