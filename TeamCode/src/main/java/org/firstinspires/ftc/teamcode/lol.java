package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="lol")
public class lol extends hardwareMap{

    @Override
    public void runOpMode() throws InterruptedException {
        initizalize();
        waitForStart();
        int cones = 0;
        ElapsedTime elapsedTime = new ElapsedTime();
        while (opModeIsActive()) {

            //mecanum code
                mecanumCode();
            //pick up
                pickUp();
            //high pole
                highPole();
            //mid pole
                midPole();
            //low pole
                lowPole();
            //ground junction/terminal

            //camping

            //manual moving (used only for testing)
                testing();
            //telemetry
                telemetry();
        }

    }

    public void pickUp(){
        if (gamepad1.left_bumper || gamepad2.right_bumper) {
            rightArm.setPosition(.9);
            frontArm.setPosition(.45);
            wrist.setPosition(0);
            claw.setPosition(.6);
            tilt.setPosition(1);
            slides(-3, -.1);
            wait(500);
            if (dist.getDistance(DistanceUnit.INCH) < 1.5 && dist.getDistance(DistanceUnit.INCH) > 0){
                claw.setPosition(.3);
                wait(100);
                elevator(0, 1);
                leftArm.setPosition(0.55);
                rightArm.setPosition(0.25);
                wrist.setPosition(1);
                wait(500);
                claw.setPosition(.6);
                wait(100);
                elevator(450, 1);
                tilt.setPosition(.5);
            }
        } else {
            rightArm.setPosition(.4);
            wrist.setPosition(1);
            frontArm.setPosition(.54);
            claw.setPosition(.4);
        }
    }


    public void mecanumCode(){
        if (gamepad1.dpad_down) {
            //reverse mode
            reverse = true;
        } else if (gamepad1.dpad_up) {
            //normal mode
            reverse = false;
        }

        if (gamepad1.right_bumper) {
            strafeDriveTrainPower = 0.5;
            forwardDriveTrainPower = .4;
        } else if (gamepad1.right_trigger > .1) {
            strafeDriveTrainPower = 1;
            forwardDriveTrainPower = 1;
        } else {
            strafeDriveTrainPower = .8;
            forwardDriveTrainPower = .7;
        }

        Left_Stick_X = gamepad1.left_stick_x;
        left_stick_y = gamepad1.left_stick_y;
        Right_Stick_Y = gamepad1.right_stick_y;

        if (reverse == false) {
            direction = 1;
            left_stick_y = gamepad1.left_stick_y;
            Right_Stick_Y = gamepad1.right_stick_y;
        } else if (reverse == true) {
            direction = -1;
            //TODO: fix so it's not lying...
            left_stick_y = gamepad1.right_stick_y;
            Right_Stick_Y = gamepad1.left_stick_y;
        }

        lf.setPower(direction * (strafeDriveTrainPower * -1 * Left_Stick_X + forwardDriveTrainPower * left_stick_y));
        lb.setPower(direction * (strafeDriveTrainPower * 1 * Left_Stick_X + forwardDriveTrainPower * left_stick_y));
        rf.setPower(direction * (strafeDriveTrainPower * 1 * Left_Stick_X + forwardDriveTrainPower * Right_Stick_Y));
        rb.setPower(direction * (strafeDriveTrainPower * -1 * Left_Stick_X + forwardDriveTrainPower * Right_Stick_Y));
    }

    public void slides(int slidePosition, double slidePower) {
        slide.setTargetPosition(slidePosition);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(slidePower);
    }

    public void wait(int waitTime) {
        elapsedTime.reset();
        while(elapsedTime.milliseconds() < waitTime){
            mecanumCode();
            telemetry();
        }
    }
    public void telemetry() {
        telemetry.addData("distance", dist.getDistance(DistanceUnit.INCH));
        telemetry.addData("slides", slide.getCurrentPosition());
        telemetry.addData("frontArm", frontArm.getPosition());
        telemetry.addData("rightArm", rightArm.getPosition());
        telemetry.addData("claw", claw.getPosition());
        telemetry.addData("tilt", tilt.getPosition());
        telemetry.addData("poleDist", bigboy.getDistance(DistanceUnit.INCH));
        telemetry.addData("lslide", lSlide.getCurrentPosition());
        telemetry.addData("rslide", rSlide.getCurrentPosition());
        telemetry.addData("strafeDriveTrainPower", strafeDriveTrainPower);
        telemetry.addData("forwardDriveTrainPower", forwardDriveTrainPower);
        telemetry.update();
    }
    public void highPole() {
        if (gamepad1.triangle || gamepad2.dpad_up) {
            tilt.setPosition(.5);
            elevator(2120, 1);
            isBusyWait(rSlide);
            wait(500);
            elevator(1900, 1);
            tilt.setPosition(1);
            elevator(0, .8);
        }
    }
    public void midPole() {
        if (gamepad1.square || gamepad2.dpad_left) {
            tilt.setPosition(.45);
            elevator(1310, 1);
            isBusyWait(rSlide);
            wait(500);
            elevator(1200, 1);
            tilt.setPosition(1);
            elevator(0, .3);
        }
    }
    public void lowPole() {
        if (gamepad1.circle || gamepad2.dpad_right) {
            tilt.setPosition(.4);
            elevator(1500, 1);
            wait(700);
            elevator(1900, 1);
            tilt.setPosition(1);
            elevator(0, .3);
        }
    }
    public void testing() {
        if (gamepad2.left_bumper) {
            if (gamepad2.right_stick_y > .05 || gamepad2.right_stick_y < -.05) {
                lSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lSlide.setPower(-gamepad2.right_stick_y);
                rSlide.setPower(-gamepad2.right_stick_y);
            } else {
                lSlide.setPower(0);
                rSlide.setPower(0);
            }
        }
    }
    public void elevator(int elevatePosition, double elevatePower){
        lSlide.setTargetPosition(elevatePosition);
        rSlide.setTargetPosition(elevatePosition);
        lSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSlide.setPower(elevatePower);
        rSlide.setPower(elevatePower);
    }
    public void isBusyWait(DcMotor motor) {
        while (motor.isBusy()) {
            mecanumCode();
            telemetry();
        }
    }
}
