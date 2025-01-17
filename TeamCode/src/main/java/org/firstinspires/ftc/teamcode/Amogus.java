package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp
//@Autonomous
public class Amogus extends hardwareMap{
    @Override
    public void runOpMode() throws InterruptedException {
        initizalize();
        lift.setPosition(.15);
        waitForStart();
        int cones = 0;
        ElapsedTime elapsedTime = new ElapsedTime();
        while (opModeIsActive()) {
            try {

                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (Exception e) {

            }
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
                lowPoles();
            //ground junction/terminal
                terminal();
            //camping
                rightCamping();
                leftCamping();
            //manual moving (used only for testing)
                testing();
            //telemetry
                telemetry();
            //extend
                extend();
            //stuck
                stuck();
                driver2();
            //random
                leftArm.setPosition(rightArm.getPosition());
                gamepadEx = new GamepadEx(gamepad1);
                gamepadEx.readButtons();

                if (gamepad1.left_stick_button) {
                    hasCone = false;
                }
                if(lSlide.getCurrentPosition() < 10 || rSlide.getCurrentPosition() < 10) {
                    lSlide.setPower(.2);
                    lSlide.setPower(.2);
                }


        }

    }

    public void driver2() {
        if(gamepad2.right_bumper && !previousGamepad2.right_bumper) {
            elevator(18, .1);
            //move back the slides
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            //move the arm and stuff
            rightArm.setPosition(.9);
            frontArm.setPosition(.33);
            wrist.setPosition(.14);
            claw.setPosition(.59);
            tilt.setPosition(.46);
            slides(13, 1);
        }
        if(gamepad2.left_bumper && !previousGamepad2.left_bumper) {
            hasCone = true;
            claw.setPosition(.82);
            wait(200);

            elevator(18, 1);
            tilt.setPosition(.46);

            //close claw
            elevator(18, 1);
            rightArm.setPosition(.4);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
        }
    }
    public void pickUp(){
        if (gamepad1.left_bumper && hasCone == false|| gamepad2.right_bumper) {
            elevator(18, .1);
            //move back the slides
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            //move the arm and stuff
            rightArm.setPosition(.9);
            frontArm.setPosition(.33);
            wrist.setPosition(.14);
            claw.setPosition(.59);
            tilt.setPosition(.46);
            slides(13, 1);
            if ((dist.getDistance(DistanceUnit.INCH) < 1 && dist.getDistance(DistanceUnit.INCH) > 0) || gamepad2.left_bumper){
                hasCone = true;
                claw.setPosition(.82);
                wait(200);

                elevator(18, 1);
                tilt.setPosition(.46);

                //close claw
                elevator(18, 1);
                //move arm
                rightArm.setPosition(0.25);
                frontArm.setPosition(.54);
                wrist.setPosition(.81);
                wait(900);
                //open claw
                claw.setPosition(.56);
                wait(200);
                //move arm out of the way
                rightArm.setPosition(.4);
                wait(100);
                wrist.setPosition(.81);
                frontArm.setPosition(.54);
                claw.setPosition(.82);
                //tilt the tilt
                tilt.setPosition(.46);
            }
        }

        if (!currentGamepad1.dpad_left && previousGamepad1.dpad_left && hasCone == false){
            rightArm.setPosition(.4);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
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
        } else {
            strafeDriveTrainPower = 1;
            forwardDriveTrainPower = 1;
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
            leftArm.setPosition(rightArm.getPosition());
            gamepadEx = new GamepadEx(gamepad1);
            gamepadEx.readButtons();
        }
    }
    public void telemetry() {
        telemetry.addData("distance", dist.getDistance(DistanceUnit.INCH));
        telemetry.addData("slides", slide.getCurrentPosition());
        telemetry.addData("frontArm", frontArm.getPosition());
        telemetry.addData("rightArm", rightArm.getPosition());
        telemetry.addData("claw", claw.getPosition());
        telemetry.addData("tilt", tilt.getPosition());
      //  telemetry.addData("poleDist", bigboy.getDistance(DistanceUnit.INCH));
        telemetry.addData("lslide", lSlide.getCurrentPosition());
        telemetry.addData("rslide", rSlide.getCurrentPosition());
        telemetry.addData("strafeDriveTrainPower", strafeDriveTrainPower);
        telemetry.addData("forwardDriveTrainPower", forwardDriveTrainPower);
        telemetry.addData("just released", gamepadEx.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER));

        telemetry.addData("\n slide amps", slide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("lslide amps", lSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rslide amps", rSlide.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("turret amps", turret.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("drive", lf.getCurrent(CurrentUnit.AMPS) + lb.getCurrent(CurrentUnit.AMPS) + rb.getCurrent(CurrentUnit.AMPS) + rf.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("total amps", slide.getCurrent(CurrentUnit.AMPS) + lSlide.getCurrent(CurrentUnit.AMPS) + lSlide.getCurrent(CurrentUnit.AMPS) + turret.getCurrent(CurrentUnit.AMPS)+ lf.getCurrent(CurrentUnit.AMPS) + lb.getCurrent(CurrentUnit.AMPS) + rb.getCurrent(CurrentUnit.AMPS) + rf.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
    public void highPole() {
        if (gamepad1.triangle || gamepad2.dpad_up) {
            hasCone = false;
            
            //elevate the elevator
            wait(100);
            elevator(845, 1);
            while (lSlide.getCurrentPosition() < (lSlide.getTargetPosition() - 5)) {
                wait(1);
            }
            wait(100);
            tilt.setPosition(.46);
            elevator(13, 1);

        }
    }
    public void midPole() {
        if (gamepad1.square || gamepad2.dpad_left) {
            hasCone = false;
            //elevate the elevator
            wait(100);
            elevator(550, 1);
            while (lSlide.getCurrentPosition() < (lSlide.getTargetPosition() - 5)) {
                wait(1);
            }
            wait(100);
            tilt.setPosition(.46);
            elevator(18, .8);
        }
    }
    public void lowPole() {
        if ((gamepad1.left_trigger > .1 || gamepad2.dpad_right)) {
            claw.setPosition(.82);
            wrist.setPosition(.17);
            rightArm.setPosition(.32);
            frontArm.setPosition(1);
        }
        if (currentGamepad1.left_trigger < .3 && previousGamepad1.left_trigger > .3){
            hasCone = false;
            claw.setPosition(.56);
            wait(500);
            rightArm.setPosition(.4);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.56);
        }
    }
    public void terminal() {
        if ((gamepad1.right_trigger > .1 || gamepad2.dpad_right)) {
            rightArm.setPosition(.9);
            frontArm.setPosition(.35);
            wrist.setPosition(.14);
            claw.setPosition(.82);
        }
        if (currentGamepad1.right_trigger < .3 && previousGamepad1.right_trigger > .3){
            hasCone = false;
            claw.setPosition(.56);
            wait(200);
            rf.setPower(1);
            rb.setPower(1);
            lf.setPower(1);
            lb.setPower(1);
            sleep(200);
            rf.setPower(0);
            rb.setPower(0);
            lf.setPower(0);
            lb.setPower(0);
            rightArm.setPosition(.4);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
        }
    }
    public void lowPoles() {
        if (gamepad1.right_stick_button || gamepad2.dpad_left) {
            hasCone = false;
            tilt.setPosition(.2);
            //elevate the elevator
            wait(100);
            elevator(225, 1);
            wait(700);
            elevator(0, .8);
            wait(1000);
            tilt.setPosition(.46);
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
        while ((motor.getCurrentPosition() > motor.getTargetPosition() - 5 || motor.getCurrentPosition() < motor.getTargetPosition() + 5)) {
            mecanumCode();
            telemetry();
            leftArm.setPosition(rightArm.getPosition());
            gamepadEx = new GamepadEx(gamepad1);
            gamepadEx.readButtons();
        }
    }
    public void  rightCamping() {
        if (gamepad1.dpad_right) {
            //tilt the tilt
            tilt.setPosition(.46);
            //put down the arm and open the claw
            rightArm.setPosition(.9);
            frontArm.setPosition(.3);
            wrist.setPosition(.14);
            claw.setPosition(.56);
            tilt.setPosition(.46);
            //move the slides
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (slide.getCurrentPosition() < -400) {
                slide.setPower(-.4);
            } else {
                slide.setPower(-.5);
            }
            wait(200);
            while ((dist.getDistance(DistanceUnit.INCH) > 5 && dist.getDistance(DistanceUnit.INCH) > 0) && !gamepad1.touchpad && !gamepad2.left_bumper) {
                wait(1);
            }
            slide.setPower(-.4);
            while ((dist.getDistance(DistanceUnit.INCH) > .8 && dist.getDistance(DistanceUnit.INCH) > 0) && !gamepad1.touchpad && ! gamepad2.right_bumper) {
                wait(1);
            }
            //this if statement is so that I can make the retract if just in case the claw dosen't sense any cone
            if (gamepad1.touchpad) {
                //pick up the cone
                claw.setPosition(.82);
                slide.setPower(-.1);
                wait(200);
                //move the slides back
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                //bring the arm up
                rightArm.setPosition(.4);
                wrist.setPosition(.81);
                frontArm.setPosition(.54);
                claw.setPosition(.82);
            } else {
            //pick up the cone
            claw.setPosition(.82);
            slide.setPower(-.1);
            wait(200);
            //move the slides back
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            //bring the arm up
            rightArm.setPosition(.4);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
            //second part
                wait(500);
                hasCone = false;
                tilt.setPosition(.46);
                //close claw
                claw.setPosition(.82);
                wait(100);
                elevator(18, 1);
                //move arm
                rightArm.setPosition(0.25);
                wrist.setPosition(.81);
                wait(200);
                //open claw
                claw.setPosition(.56);
                wait(200);
                //move arm out of the way
                rightArm.setPosition(.4);
                wait(100);
                wrist.setPosition(.81);
                frontArm.setPosition(.54);
                claw.setPosition(.82);
                wait(100);
                //move the turret
                turret.setTargetPosition(-626);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
                wait(200);
                rightArm.setPosition(.9);
                frontArm.setPosition(.3);
                wrist.setPosition(.14);
                claw.setPosition(.56);
                tilt.setPosition(.46);
                slide.setTargetPosition(-430);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                //raise the elevator
                elevator(845, 1);
                //move the elevator and turret
                while (lSlide.getCurrentPosition() < (lSlide.getTargetPosition() - 5)) {
                    wait(1);
                }
                wait(100);
                elevator(18, 1);
                wait(300);
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
            }
        }
    }
    public void extend () {
        if (gamepad1.cross) {
            //tilt the tilt
            tilt.setPosition(.46);
            //put down the arm and open the claw
            rightArm.setPosition(.9);
            frontArm.setPosition(.3);
            wrist.setPosition(.14);
            claw.setPosition(.56);
            tilt.setPosition(.46);
            //move the slides
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (slide.getCurrentPosition() < -400) {
                slide.setPower(-.2);
            } else {
                slide.setPower(-.5);
            }
            wait(200);
            while ((dist.getDistance(DistanceUnit.INCH) > 5 && dist.getDistance(DistanceUnit.INCH) > 0) && !gamepad1.touchpad) {
                wait(1);
            }
            slide.setPower(-.2);
            while ((dist.getDistance(DistanceUnit.INCH) > 1 && dist.getDistance(DistanceUnit.INCH) > 0) && !gamepad1.touchpad) {
                wait(1);
            }
            //pick up the cone
            claw.setPosition(.82);
            slide.setPower(-.1);
            wait(200);
            rightArm.setPosition(.4);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
            //move the slides back
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            while (slide.getCurrentPosition() > (slide.getTargetPosition() + 5)) {
                wait(1);
            }
            //move arm
            rightArm.setPosition(0.25);
            frontArm.setPosition(.54);
            wrist.setPosition(.81);
            wait(900);
            //open claw
            claw.setPosition(.56);
            wait(200);
            //move arm out of the way
            rightArm.setPosition(.4);
            wait(100);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
            //tilt the tilt
            tilt.setPosition(.46);
        }
    }
    public void leftCamping (){
        if (gamepad1.dpad_left) {
            //tilt the tilt
            tilt.setPosition(.46);
            //put down the arm and open the claw
            rightArm.setPosition(.9);
            frontArm.setPosition(.3);
            wrist.setPosition(.14);
            claw.setPosition(.56);
            tilt.setPosition(.46);
            //move the slides
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (slide.getCurrentPosition() < -400) {
                slide.setPower(-.4);
            } else {
                slide.setPower(-.5);
            }
            wait(200);
            while ((dist.getDistance(DistanceUnit.INCH) > 5 && dist.getDistance(DistanceUnit.INCH) > 0) && !gamepad1.touchpad && !gamepad2.left_bumper) {
                wait(1);
            }
            slide.setPower(-.4);
            while ((dist.getDistance(DistanceUnit.INCH) > .8 && dist.getDistance(DistanceUnit.INCH) > 0) && !gamepad1.touchpad && !gamepad2.left_bumper) {
                wait(1);
            }
            //this if statement is so that I can make the retract if just in case the claw dosen't sense any cone
            if (gamepad1.touchpad) {
                //pick up the cone
                claw.setPosition(.82);
                slide.setPower(-.1);
                wait(200);
                //move the slides back
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                //bring the arm up
                rightArm.setPosition(.4);
                wrist.setPosition(.81);
                frontArm.setPosition(.54);
                claw.setPosition(.82);
            } else {
                //pick up the cone
                claw.setPosition(.82);
                slide.setPower(-.1);
                wait(200);
                //move the slides back
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                //bring the arm up
                rightArm.setPosition(.4);
                wrist.setPosition(.81);
                frontArm.setPosition(.54);
                claw.setPosition(.82);
                //second part
                wait(500);
                hasCone = false;
                tilt.setPosition(.46);
                //close claw
                claw.setPosition(.82);
                wait(100);
                elevator(18, 1);
                //move arm
                rightArm.setPosition(0.25);
                wrist.setPosition(.81);
                wait(200);
                //open claw
                claw.setPosition(.56);
                wait(200);
                //move arm out of the way
                rightArm.setPosition(.4);
                wait(100);
                wrist.setPosition(.81);
                frontArm.setPosition(.54);
                claw.setPosition(.82);
                wait(100);
                //move the turret
                turret.setTargetPosition(626);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
                wait(200);
                rightArm.setPosition(.9);
                frontArm.setPosition(.3);
                wrist.setPosition(.14);
                claw.setPosition(.56);
                tilt.setPosition(.46);
                slide.setTargetPosition(-430);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);
                //raise the elevator
                elevator(845, 1);
                //move the elevator and turret
                while (lSlide.getCurrentPosition() < (lSlide.getTargetPosition() - 5)) {
                    wait(1);
                }
                wait(100);
                elevator(18, 1);
                wait(300);
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1);
            }
        }
    }
    public void stuck() {
        if(currentGamepad1.circle && !previousGamepad1.circle && stuckMode == false) {
            stuckMode = true;
            //move all the slides out
            slide.setTargetPosition(-300);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            elevator(630, 1);
            //move the arm and stuff
            rightArm.setPosition(.4);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
            tilt.setPosition(.9);
        }
        if(gamepad1.touchpad && stuckMode == true) {
            stuckMode = false;
            //move the arm and stuff
            rightArm.setPosition(.4);
            wrist.setPosition(.81);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
            tilt.setPosition(.46);
            //move all the slides
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            elevator(18, 1);
        }

        if(gamepad1.touchpad) {
            elevator(18, 1);
            slide.setTargetPosition(0);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
            claw.setPosition(.82);
            wait(200);
            rightArm.setPosition(.4);
            wrist.setPosition(1);
            frontArm.setPosition(.54);
            claw.setPosition(.82);
        }
    }
}
