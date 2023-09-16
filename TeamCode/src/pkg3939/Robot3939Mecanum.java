package pkg3939;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot3939Mecanum<T extends OpMode> extends Robot3939<T> {

    public Servo backServo;
    public ColorSensor colorSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor frontDistanceSensor;
    public DistanceSensor rightDistanceSensor;
    public DistanceSensor backDistanceSensor;

    @Override
    public void init(T om) {
        super.init(om);
        initServo();
        initColorSensor();
        initDistanceSensors();
    }

    public void initServo() {
        backServo = hwmap.servo.get("back_servo");
        backServo.setPosition(0);
    }

    public void initColorSensor() {
        colorSensor = hwmap.colorSensor.get("color_sensor");
    }

    public void initDistanceSensors() {
        leftDistanceSensor = hwmap.get(DistanceSensor.class, "left_distance");
        frontDistanceSensor = hwmap.get(DistanceSensor.class, "front_distance");
        rightDistanceSensor = hwmap.get(DistanceSensor.class, "right_distance");
        backDistanceSensor = hwmap.get(DistanceSensor.class, "back_distance");
    }

    @Override
    public void drive() {
        super.drive();
        if (gp.dpad_down)
            rotateToAngle(1, 180);
        else if (gp.dpad_right)
            rotateToAngle(1, 270);
        else if (gp.dpad_left)
            rotateToAngle(1, 90);
        else if (gp.dpad_up)
            rotateToAngle(1, 0);
        else if (gp.left_bumper)
            rotate(1, 90);
        else if (gp.right_bumper)
            rotate(1, -90);
        else if (gp.b)
            setBackServoPosition(Math.min(backServo.getPosition() * 180 + 0.5, 180));
        else if (gp.x)
            setBackServoPosition(Math.max(backServo.getPosition() * 180 - 0.5, 0));
        telemetry.addData("Color", colorSensor.blue());
        telemetry.addData("Front Sensor Distance", frontDistanceSensor.getDistance(DistanceUnit.INCH));
    }

    public void setBackServoPosition(double degrees) {
        double pos = degrees / 180;
        backServo.setPosition(pos);
    }
}