package Samples.ftc16072;

import Samples.ftc16072.Util.RobotPosition;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Simple_Auto", group = "ftc16072")
public class SimpleAuto extends QQ_AutoBase {
    @Override
    List<QQ_AutoAction> getSteps() {
        return Arrays.asList(
                new QQ_ActionSetPosition(new RobotPosition(0, 0, DistanceUnit.INCH, 45, AngleUnit.DEGREES)),
                new QQ_ActionDriveTo(0, 24, DistanceUnit.INCH));
    }
}
