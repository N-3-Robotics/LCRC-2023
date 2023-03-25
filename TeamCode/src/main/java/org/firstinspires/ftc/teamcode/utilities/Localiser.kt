package org.firstinspires.ftc.teamcode.utilities

import org.firstinspires.ftc.teamcode.utilities.geometry.Pose2d

interface Localiser {
    var poseEstimate: Pose2d

    val poseVelocity: Pose2d?

    fun update()
}