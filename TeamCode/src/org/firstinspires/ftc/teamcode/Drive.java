package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import pkg3939.Robot3939Mecanum;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends OpMode {

    Robot3939Mecanum<Drive> robot = new Robot3939Mecanum<>();

    @Override
    public void init() { robot.init(this); }

    @Override
    public void loop() { robot.drive(); }
}