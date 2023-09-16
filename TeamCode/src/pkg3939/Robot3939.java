package pkg3939;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot3939<T extends OpMode> {

    public T opMode; // Represents the OpMode utilizing this class

    public HardwareMap hwmap; // Hardware mapping for the robot
    public DcMotor RL, RR, FL, FR; // Motors
    public BNO055IMU imu; // Gyro

    public double WHEEL_DIAMETER = 4; // Diameter of the drive wheel
    public double ENCODER_TICKS = 1120; // Ticks per full revolution of drive motor
    public double ROTATION_OVERSHOOT_PROTECTION = 0; //
    public double TURN_SLOW_SPEED = 0.1; // The speed to which the robot should slow down when getting close to the target angle
    public double START_SLOW_DEGREE = 5; // How many degrees before the target angle should the robot start slowing down

    PIDController pidDrive; // Controller used to drive in straight lines
    private Orientation lastAngles = new Orientation(); // Saves the last angles
    double globalAngle; // Robot angle
    double correction; // Variable used to determine correction needed to drive in straight line

    public Telemetry telemetry; // Robot Telemetry

    public Gamepad gp; // Gamepad controlling the robot

    public ElapsedTime runtime = new ElapsedTime(); // Used to calculate runtime

    public double currentXDisplacement = 0;
    public double currentYDisplacement = 0;

    // Initializes robot components and variables
    public void init(T om) {
        opMode = om;
        hwmap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gp = opMode.gamepad1;
        initPID();
        initMotors();
        initGyro();
    }

    // Initializes motors
    public void initMotors() {
        RL = hwmap.dcMotor.get("back_left_motor");
        RR = hwmap.dcMotor.get("back_right_motor");
        FL = hwmap.dcMotor.get("front_left_motor");
        FR = hwmap.dcMotor.get("front_right_motor");

        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);

        useDriveEncoders(true);
    }

    // Initializes gyro
    public void initGyro() {
        imu = hwmap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

    }

    // Initializes and sets up the PID parameters
    public void initPID() {
        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);
        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 1);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
    }

    // Method for operating the robot during TeleOp
    public void drive() {
        double forward = -gp.left_stick_y;
        double strafe = gp.right_trigger - gp.left_trigger;
        double rotate = gp.right_stick_x;

        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        telemetry.addData("Angle", get360Angle());

        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    // Moves the robot forward/backward along the axis the robot was facing when initialized over the distance provided
    public void moveVertically(double power, int displacementInInches) {
        double initialAngle = get360Angle();
        if (initialAngle > 90 && initialAngle < 270) {
            rotateToAngle(1, 180);
            displacementInInches = -displacementInInches;
        } else
            rotateToAngle(1, 0);
        moveDistance(power, displacementInInches);
        rotateToAngle(1, initialAngle);
    }

    // Moves robot forward/backward during the time provided
    public void moveTime(double power, double time) {
        setMotorPowers(power, power, power, power);
        sleep(time);
        stopDriveMotors();
    }

    // Moves robot forward/backward with the distance provided
    public void moveDistance(double power, double displacementInInches) {
        useDriveEncoders(true);
        resetDriveEncoders();
        runToPositionDriveEncoders();
        resetAngle();
        power = Math.copySign(power, displacementInInches);
        setTargetPositionDriveEncoders(ticksNeeded(displacementInInches));
        setAllMotorPowers(power);
        pidDrive.setOutputRange(0, power);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(getAngle());

            // set power levels.
            FL.setPower(power - correction);
            FR.setPower(power + correction);
            RL.setPower(power - correction);
            RR.setPower(power + correction);
        }
        stopDriveMotors();
        useDriveEncoders(true);
    }

    // Uses PID and gyro to move the robot forward/backward in a straight line during the time provided
    public void movePID(double power, double time) {
        resetAngle();
        pidDrive.setOutputRange(0, power);
        runtime.reset();
        // Use PID with imu input to drive in a straight line.
        while (runtime.seconds() < time) {
            correction = pidDrive.performPID(getAngle());

            // set power levels.
            FL.setPower(power - correction);
            FR.setPower(power + correction);
            RL.setPower(power - correction);
            RR.setPower(power + correction);
        }
        stopDriveMotors();
    }

    // Rotates to the given angle and strafes relative to that
    public void moveAtAngle(double power, double angle, double displacementInInches, boolean returnToOriginalRotation) {
        double initialRotation = get360Angle();
        rotateToAngle(power, angle);
        moveDistance(power, displacementInInches);
        if (returnToOriginalRotation)
            rotateToAngle(power, initialRotation);
    }

    /* Moves the robot left/right along the axis perpendicular to the
    direction the robot was facing when initialized over the distance provided */
    public void strafeHorizontally(double power, int displacementInInches) {
        double initialAngle = get360Angle();
        if (initialAngle > 90 && initialAngle < 270) {
            rotateToAngle(1, 180);
            displacementInInches = -displacementInInches;
        } else
            rotateToAngle(1, 0);
        strafeDistance(power, displacementInInches);
        rotateToAngle(1, initialAngle);
    }

    // Moves the robot left/right during the time provided
    public void strafeTime(double power, double time) {
        setMotorPowers(power, -power, -power, power);
        sleep(time);
        stopDriveMotors();
    }

    // Moves the robot left/right with the distance provided
    public void strafeDistance(double power, double displacementInInches) {
        useDriveEncoders(true);
        resetDriveEncoders();
        power = Math.copySign(power, displacementInInches);
        int ticks = ticksNeeded(displacementInInches);
        resetAngle();
        RL.setTargetPosition(-ticks);
        RR.setTargetPosition(ticks);
        FL.setTargetPosition(ticks);
        FR.setTargetPosition(-ticks);
        runToPositionDriveEncoders();
        setAllMotorPowers(power);
        pidDrive.setOutputRange(0, power);
        while (motorsAreBusy()) {
            correction = pidDrive.performPID(getAngle());

            // set power levels.
            FL.setPower(power - correction);
            FR.setPower(-power + correction);
            RL.setPower(-power - correction);
            RR.setPower(power + correction);
        }
        stopDriveMotors();
        useDriveEncoders(true);
    }

    // Uses PID and gyro to move the robot left/right in a straight line during the time provided
    public void strafePID(double power, double time) {
        resetAngle();
        pidDrive.setOutputRange(0, power);
        runtime.reset();
        // Use PID with imu input to drive in a straight line.
        while (runtime.seconds() < time) {
            correction = pidDrive.performPID(getAngle());

            // set power levels.
            FL.setPower(power - correction);
            FR.setPower(-power + correction);
            RL.setPower(-power - correction);
            RR.setPower(power + correction);
        }
        stopDriveMotors();
    }

    // Rotates to the given angle and strafes relative to that
    public void strafeAtAngle(double power, double angle, double displacementInInches, boolean returnToOriginalRotation) {
        double initialRotation = get360Angle();
        rotateToAngle(power, angle);
        strafeDistance(power, displacementInInches);
        if (returnToOriginalRotation)
            rotateToAngle(power, initialRotation);
    }


    // Moves the robot to the displacements on the x and y axes provided relative to the robot's current position
    public void moveXY(double power, double x, double y, boolean returnToOriginalRotation) {
        double angle =  Math.toDegrees(Math.atan2(y, x));
        if (angle < 0)
            angle = angle + 360;
        angle -= 90;
        if (angle < 0)
            angle += 360;
        double distance = Math.hypot(x, y);
        moveAtAngle(power, angle, distance, returnToOriginalRotation);
        currentXDisplacement += x;
        currentYDisplacement += y;
    }

    /* Moves the robot to the specified coordinates where the origin is the position of the robot when it was initialized
    (Only use if the only things changing the position of the robot are the functions moveXY and moveToPoint) */
    public void moveToPoint(double power, double x, double y, boolean returnToOriginalRotation) {
        double targetX = x - currentXDisplacement;
        double targetY = y - currentYDisplacement;
        moveXY(power, targetX, targetY, returnToOriginalRotation);
    }

    // Rotates the robot based on degrees provided (+ degrees: turn left, - degrees: turn right)
    public void rotate(double power, double degrees) {
        boolean goingLeft = false;
        boolean goingRight = false;

        double startSlowDegree = Math.abs(degrees) - START_SLOW_DEGREE;
        boolean notSlowed = true;

        if (degrees < 0) {// turn right
            goingRight = true;
            setMotorPowers(power, -power, power, -power);
        } else if (degrees > 0) {// turn left.
            goingLeft = true;
            setMotorPowers(-power, power, -power, power);
        } else
            return;

        double oldDegree = get360Angle();
        double degreesMoved = 0;

        double targetDegrees = Math.abs(degrees) - ROTATION_OVERSHOOT_PROTECTION;

        while (degreesMoved < targetDegrees) {
            if ((degreesMoved > startSlowDegree) && notSlowed) {
                if (goingLeft)
                    setMotorPowers(-TURN_SLOW_SPEED, TURN_SLOW_SPEED, -TURN_SLOW_SPEED, TURN_SLOW_SPEED);
                else
                    setMotorPowers(TURN_SLOW_SPEED, -TURN_SLOW_SPEED, TURN_SLOW_SPEED, -TURN_SLOW_SPEED);
                notSlowed = false;
            }

            double newDegree = get360Angle();
            telemetry.addData("Angle", newDegree);
            telemetry.update();
            if (goingRight && newDegree > oldDegree)
                degreesMoved += (oldDegree - newDegree) + 360;
            else if (goingLeft && newDegree < oldDegree)
                degreesMoved += (newDegree - oldDegree) + 360;
            else
                degreesMoved += Math.abs(newDegree - oldDegree);
            oldDegree = newDegree;
        }

        // turn the motors off.
        stopDriveMotors();

        // wait for rotation to stop.
        while (RL.isBusy() || RR.isBusy() || FR.isBusy() || FL.isBusy()) {
        }
    }

    // Rotates the robot to a specified angle
    public void rotateToAngle(double power, double angle) {
        double currentAngle = get360Angle();
        double targetAngle = angle - currentAngle;

        if (targetAngle < -180)
            targetAngle += 360;
        else if (targetAngle > 180)
            targetAngle -= 360;

        rotate(power, (int) Math.round(targetAngle));

    }

    // Sets the provided powers to their corresponding motors
    public void setMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        this.FL.setPower(flPower);
        this.RL.setPower(blPower);
        this.FR.setPower(frPower);
        this.RR.setPower(brPower);
    }

    // Sets the power provided to all motors
    public void setAllMotorPowers(double power) {
        setMotorPowers(power, power, power, power);
    }

    // Stops the drive motors
    public void stopDriveMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    // Returns the angle of the robot on a 0-360 scale (0 is the direction of the robot when initialized)
    public double get360Angle() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    // Gets the cumulative angle and registers the previous angle
    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    // Resets the global angle and registers the last angles
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // Sets whether or not the drive motors should use encoders
    public void useDriveEncoders(boolean useEncoders) {
        if (useEncoders) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Resets the drive motors encoders
    public void resetDriveEncoders() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Sets drive motors encoders to run to position
    public void runToPositionDriveEncoders() {
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Sets target position for drive motor encoders
    public void setTargetPositionDriveEncoders(int position) {
        FL.setTargetPosition(position);
        RL.setTargetPosition(position);
        FR.setTargetPosition(position);
        RR.setTargetPosition(position);
    }

    // Returns the number of ticks needed to drive a certain distance
    public int ticksNeeded(double inches) {
        double circumference = 3.1415 * WHEEL_DIAMETER;
        double rotationsNeeded = inches / circumference;
        return (int) (rotationsNeeded * ENCODER_TICKS); // Ticks needed
    }

    // Returns whether or not any of the motors are busy
    public boolean motorsAreBusy() {
        return (FL.isBusy() || RL.isBusy() || FR.isBusy() || RR.isBusy());
    }

    // Do nothing for the seconds provided
    public void sleep(double time) {
        runtime.reset();
        while (runtime.seconds() < time);
    }
}