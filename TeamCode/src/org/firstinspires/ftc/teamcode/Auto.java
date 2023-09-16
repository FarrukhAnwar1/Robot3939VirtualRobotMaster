package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import pkg3939.Robot3939Mecanum;

@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {

    Robot3939Mecanum<Auto> robot = new Robot3939Mecanum<>();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        waitForStart();
    }
}