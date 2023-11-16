package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        //   Servo test = hardwareMap.get(Servo.class, "test");
        DcMotor lift = hardwareMap.get(DcMotor.class, "intake");

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x * 5;
            final double v1 = r * Math.sin(robotAngle) + rightX;  // Sideways
            final double v2 = r * Math.cos(robotAngle) - rightX;  // Sideways
            final double v3 = r * Math.cos(robotAngle) + rightX;
            final double v4 = r * Math.sin(robotAngle) - rightX;

            motorFrontLeft.setPower(v1);
            motorFrontRight.setPower(1.2*v2);
            motorBackLeft.setPower(v3);
            motorBackRight.setPower(v4);
        }
        if(gamepad1.dpad_up)
        {
            lift.setPower(0.7);
        }
        if(gamepad1.dpad_down)
        {
            lift.setPower(0);
        }
        // test.setPosition(1);
        //  else if (gamepad1.b)
        // test.setPosition(0);
  }
