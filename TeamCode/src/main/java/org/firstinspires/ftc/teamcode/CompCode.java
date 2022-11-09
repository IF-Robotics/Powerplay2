package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "League1_Final")
public class CompCode extends LinearOpMode {


    private DcMotor back_Right;
    private DcMotor front_Right;
    private DcMotor front_Left;
    private DcMotor back_Leftx;
    private DcMotor elevate_Left;
    private DcMotor elevate_Right;
    private DcMotor elevateMid;
    private DcMotor arm;
    private Servo flip;
    private Servo wrist;
    private Servo claw;
    private LED leftGreen;
    private LED leftRed;
    private LED rightGreen;
    private LED rightRed;
    private DistanceSensor distance;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {
        double Drive_train_Power;
        float Left_Stick_X;
        float left_stick_y;
        float Right_Stick_Y;
        int elevate_brake_L;
        int elevate_brake_R;
        int elevateMid_brake;

        back_Right = hardwareMap.get(DcMotor.class, "back_Right");
        front_Right = hardwareMap.get(DcMotor.class, "front_Right");
        front_Left = hardwareMap.get(DcMotor.class, "front_Left");
        //front left has y axis dead wheel on it
        back_Leftx = hardwareMap.get(DcMotor.class, "back_Left");
        elevate_Left = hardwareMap.get(DcMotor.class, "elevate_Left");
        elevate_Right = hardwareMap.get(DcMotor.class, "elevate_Right");
        elevateMid = hardwareMap.get(DcMotor.class, "elevateMiddle");
        arm = hardwareMap.get(DcMotor.class, "arm");
        flip = hardwareMap.get(Servo.class, "flip");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        leftGreen = hardwareMap.get(LED.class, "leftGreen");
        leftRed = hardwareMap.get(LED.class, "leftRed");
        rightGreen = hardwareMap.get(LED.class, "rightGreen");
        rightRed = hardwareMap.get(LED.class, "rightRed");
        distance = hardwareMap.get(DistanceSensor.class, "distance");

//init
        // Drivetrain
        Drive_train_Power = 1;
        Left_Stick_X = 0;
        left_stick_y = 0;
        Right_Stick_Y = 0;
        boolean reverse = false;
        back_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_Right.setDirection(DcMotorSimple.Direction.REVERSE);

        front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_Leftx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevateMid.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_Leftx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_Leftx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Elevate
        elevate_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevate_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevateMid.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevate_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevate_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevateMid.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevate_Right.setDirection(DcMotorSimple.Direction.REVERSE);
        elevate_Left.setDirection(DcMotorSimple.Direction.FORWARD);
        elevateMid.setDirection(DcMotorSimple.Direction.REVERSE);
        elevate_Right.setTargetPosition(220);
        elevate_Left.setTargetPosition(220);
        elevateMid.setTargetPosition(220);
        elevate_Right.setPower(0.7);
        elevate_Left.setPower(0.7);
        elevateMid.setPower(0.7);
        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Accessories
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
        flip.setPosition(0.3);
        wrist.setPosition(1);
        claw.setPosition(0.59);
        elevate_brake_L = 220;
        elevate_brake_R = 220;
        elevateMid_brake = 220;
        boolean clawStatus = false;
        boolean wristStatus = false;
        int armMode = 0;
        int stackHeight = 500;
        int stackOneClick = 0;
        int modeOneSwitch = 0;
        int clawOneClick = 0;
        int wristOneClick = 0;

        //start
        waitForStart();
        if (opModeIsActive()) {
            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    // drive train
                    if (gamepad1.dpad_down) {
                        reverse = true;
                    }
                    if (gamepad1.dpad_up) {
                        reverse = false;
                    }
                    telemetry.addData("reverse", reverse);

                    Left_Stick_X = gamepad1.left_stick_x;
                    int direction = 1;
                    if (reverse == false) {
                        direction = 1;
                        left_stick_y = gamepad1.left_stick_y;
                        Right_Stick_Y = gamepad1.right_stick_y;
                    } else if (reverse == true) {
                        direction = -1;
                        left_stick_y = gamepad1.right_stick_y;
                        Right_Stick_Y = gamepad1.left_stick_y;
                    }

                    front_Left.setPower(direction * Drive_train_Power * (-1 * Left_Stick_X + left_stick_y));
                    back_Leftx.setPower(direction * Drive_train_Power * (1 * Left_Stick_X + left_stick_y));
                    front_Right.setPower(direction * Drive_train_Power * (1 * Left_Stick_X + Right_Stick_Y));
                    back_Right.setPower(direction * Drive_train_Power * (-1 * Left_Stick_X + Right_Stick_Y));

                    if (gamepad1.right_bumper) {
                        Drive_train_Power = 0.3;
                    } else if (gamepad1.left_bumper) {
                        Drive_train_Power = 1;
                    } else {
                        Drive_train_Power = 0.7;
                    }

                    //set up stackOneClick
                    if (gamepad2.a && armMode == 2){
                        stackOneClick += 1;
                    } else if (stackOneClick > 2) {
                        stackOneClick = 2;
                    } else {
                        stackOneClick = 0;
                    }

                    //set up modeOneSwitch
                    if (gamepad2.dpad_right){
                        modeOneSwitch += 1;
                    } else if (modeOneSwitch > 2){
                        modeOneSwitch = 2;
                    } else {
                        modeOneSwitch = 0;
                    }

                    //set up clawOneClick
                    if (gamepad2.left_bumper){
                        clawOneClick += 1;
                    } else if (clawOneClick > 2){
                        clawOneClick = 2;
                    } else {
                        clawOneClick = 0;
                    }

                    //set up wristOneClick
                    if (gamepad2.right_bumper){
                        wristOneClick += 1;
                    } else if (wristOneClick > 2){
                        wristOneClick = 2;
                    } else {
                        wristOneClick = 0;
                    }

                    //arm modes switch
                    if (gamepad2.dpad_right && armMode == 0 && modeOneSwitch == 1) {
                        armMode = 1;
                    } else if (gamepad2.dpad_right && armMode == 1 && modeOneSwitch == 1){
                        armMode = 2;
                    } else if (gamepad2.dpad_right && armMode == 2 && modeOneSwitch == 1){
                        armMode = 0;
                    }

                    //armMode Lights
                    if (armMode == 0) {
                        leftGreen.enable(false);
                        leftRed.enable(true);
                        rightGreen.enable(false);
                        rightRed.enable(true);

                    } else if (armMode == 1) {
                        leftGreen.enable(true);
                        leftRed.enable(false);
                        rightGreen.enable(true);
                        rightRed.enable(false);

                    } else if (armMode == 2) {
                        leftGreen.enable(true);
                        leftRed.enable(false);
                        rightGreen.enable(false);
                        rightRed.enable(true);
                    }

                    // Claw
                    if (gamepad2.left_bumper && clawStatus == false && clawOneClick == 1) {
                        claw.setPosition(0.59);
                        clawStatus = true;
                    } else if (gamepad2.left_bumper && clawStatus == true && clawOneClick == 1) {
                        claw.setPosition(0.99);
                        clawStatus = false;
                    }

                    // Wrist Move
                    if (gamepad2.right_bumper && wristStatus == false && wristOneClick == 1) {
                        wrist.setPosition(1);
                        claw.setPosition(0.59);
                        elevate_Right.setTargetPosition(220);
                        elevate_Left.setTargetPosition(220);
                        elevateMid.setTargetPosition(220);
                        elevate_Right.setPower(0.7);
                        elevate_Left.setPower(0.7);
                        elevateMid.setPower(0.7);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_brake_R = 220;
                        elevate_brake_L = 220;
                        elevateMid_brake = 220;
                        wristStatus = true;
                    } else if (gamepad2.right_bumper && wristStatus == true && wristOneClick == 1) {
                        wrist.setPosition(0.645);
                        elevate_Right.setTargetPosition(90);
                        elevate_Left.setTargetPosition(90);
                        elevateMid.setTargetPosition(90);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setPower(0.2);
                        elevate_Right.setPower(0.2);
                        elevateMid.setPower(0.2);
                        elevate_brake_R = 90;
                        elevate_brake_L = 90;
                        elevateMid_brake = 90;
                        wristStatus = false;
                    } else if (gamepad2.right_trigger > 0.1){
                        wrist.setPosition(0.645);
                        claw.setPosition(0.99);
                        elevate_Right.setTargetPosition(90);
                        elevate_Left.setTargetPosition(90);
                        elevateMid.setTargetPosition(90);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setPower(0.2);
                        elevate_Right.setPower(0.2);
                        elevateMid.setPower(0.2);
                        elevate_brake_R = 90;
                        elevate_brake_L = 90;
                        elevateMid_brake = 90;
                        wristStatus = false;
                    }

                    // Presets

                    //forward presets
                    if (gamepad2.dpad_up) {
                        elevate_Right.setTargetPosition(1630);
                        elevate_Left.setTargetPosition(1630);
                        elevateMid.setTargetPosition(1630);
                        elevate_Right.setPower(0.7);
                        elevate_Left.setPower(0.7);
                        elevateMid.setPower(0.7);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_brake_R = 1630;
                        elevate_brake_L = 1630;
                        elevateMid_brake = 1630;
                        arm.setTargetPosition(500);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.6);
                        wrist.setPosition(0.4);
                        flip.setPosition(0.3);
                    }

                    if (gamepad2.dpad_left) {
                        elevate_Right.setTargetPosition(1788);
                        elevate_Left.setTargetPosition(1788);
                        elevateMid.setTargetPosition(1788);
                        elevate_Right.setPower(0.7);
                        elevate_Left.setPower(0.7);
                        elevateMid.setPower(0.7);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wrist.setPosition(0.62);
                        arm.setTargetPosition(0);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.6);
                        flip.setPosition(0.3);
                        elevate_brake_R = 1788;
                        elevate_brake_L = 1788;
                        elevateMid_brake = 1788;
                    }

                    if (gamepad2.dpad_down) {
                        elevate_Right.setTargetPosition(1110);
                        elevate_Left.setTargetPosition(1110);
                        elevateMid.setTargetPosition(1110);
                        elevate_Right.setPower(0.7);
                        elevate_Left.setPower(0.7);
                        elevateMid.setPower(0.7);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wrist.setPosition(0.62);
                        arm.setTargetPosition(0);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.6);
                        flip.setPosition(0.3);
                        elevate_brake_R = 1110;
                        elevate_brake_L = 1110;
                        elevateMid_brake = 1110;
                    }

                    //backward presets
                    if (gamepad2.triangle) {
                        elevate_Right.setTargetPosition(1620);
                        elevate_Left.setTargetPosition(1620);
                        elevateMid.setTargetPosition(1620);
                        elevate_Right.setPower(0.7);
                        elevate_Left.setPower(0.7);
                        elevateMid.setPower(0.7);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setTargetPosition(1370);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wrist.setPosition(0.6);
                        flip.setPosition(0.95);
                        arm.setPower(0.6);
                        elevate_brake_R = 1620;
                        elevate_brake_L = 1620;
                        elevateMid_brake = 1620;

                    } else if (gamepad2.square) {
                        elevate_Right.setTargetPosition(920);
                        elevate_Left.setTargetPosition(920);
                        elevateMid.setTargetPosition(920);
                        elevate_Right.setPower(0.7);
                        elevate_Left.setPower(0.7);
                        elevateMid.setPower(0.7);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setTargetPosition(1370);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wrist.setPosition(0.6);
                        flip.setPosition(0.95);
                        arm.setPower(0.6);
                        elevate_brake_R = 920;
                        elevate_brake_L = 920;
                        elevateMid_brake = 920;

                    } else if (gamepad2.circle) {
                        elevate_Right.setTargetPosition(372);
                        elevate_Left.setTargetPosition(372);
                        elevateMid.setTargetPosition(372);
                        elevate_Right.setPower(0.7);
                        elevate_Left.setPower(0.7);
                        elevateMid.setPower(0.7);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setTargetPosition(1370);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        wrist.setPosition(0.6);
                        flip.setPosition(0.95);
                        arm.setPower(0.6);
                        elevate_brake_R = 372;
                        elevate_brake_L = 372;
                        elevateMid_brake = 372;

                    } else if (gamepad2.cross && armMode == 0) {
                        elevate_Right.setTargetPosition(90);
                        elevate_Left.setTargetPosition(90);
                        elevateMid.setTargetPosition(90);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setPower(.7);
                        elevate_Right.setPower(.7);
                        elevateMid.setPower(.7);
                        flip.setPosition(0.3);
                        wrist.setPosition(0.645);
                        claw.setPosition(0.99);
                        arm.setTargetPosition(0);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.8);
                        elevate_brake_R = 90;
                        elevate_brake_L = 90;
                        elevateMid_brake = 90;

                    } else if (gamepad2.cross && armMode == 1) {
                        elevate_Right.setTargetPosition(200);
                        elevate_Left.setTargetPosition(200);
                        elevateMid.setTargetPosition(200);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setPower(.7);
                        elevate_Right.setPower(.7);
                        elevateMid.setPower(.7);
                        flip.setPosition(0.3);
                        wrist.setPosition(0.95);
                        claw.setPosition(0.59);
                        arm.setTargetPosition(0);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.8);
                        elevate_brake_R = 200;
                        elevate_brake_L = 200;
                        elevateMid_brake = 200;


                    } else if (gamepad2.cross && armMode == 2 && stackOneClick == 1) {
                        elevate_Right.setTargetPosition(stackHeight);
                        elevate_Left.setTargetPosition(stackHeight);
                        elevateMid.setTargetPosition(stackHeight);
                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevate_Left.setPower(.7);
                        elevate_Right.setPower(.7);
                        elevateMid.setPower(.7);
                        flip.setPosition(0.3);
                        wrist.setPosition(0.63);
                        claw.setPosition(0.99);
                        arm.setTargetPosition(0);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(0.8);
                        elevate_brake_R = stackHeight;
                        elevate_brake_L = stackHeight;
                        elevateMid_brake = stackHeight;
                        stackHeight -= 96;
                        if (stackHeight < 50) {
                            stackHeight = 477;
                        }


                    } else {
                        if (!elevate_Right.isBusy()) {
                            if (elevate_brake_L < 200 || elevate_brake_R < 200) {
                                elevate_brake_R = elevate_Left.getCurrentPosition();
                                elevate_brake_L = elevate_Right.getCurrentPosition();
                                elevateMid_brake = elevateMid.getCurrentPosition();
                                elevate_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                elevate_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                elevateMid.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                if (gamepad2.left_stick_y < -0.1) {
                                    elevate_Left.setPower(-1 * gamepad2.left_stick_y);
                                    elevate_Right.setPower(-1 * gamepad2.left_stick_y);
                                    elevateMid.setPower(-1 * gamepad2.left_stick_y);
                                } else {
                                    elevate_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    elevate_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    elevateMid.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    elevate_Left.setPower(0.03);
                                    elevate_Right.setPower(0.03);
                                    elevateMid.setPower(0.03);
                                }
                            } else {
                                // virtual limit
                                if (elevate_brake_L > 1925 || elevate_brake_R > 1925) {
                                    if (gamepad2.left_stick_y > 0.1) {
                                        elevate_brake_L = elevate_Right.getCurrentPosition();
                                        elevate_brake_R = elevate_Right.getCurrentPosition();
                                        elevateMid_brake = elevateMid.getCurrentPosition();
                                        elevate_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                        elevate_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                        elevateMid.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                        elevate_Right.setPower(-1 * gamepad2.left_stick_y);
                                        elevate_Left.setPower(-1 * gamepad2.left_stick_y);
                                        elevateMid.setPower(-1 * gamepad2.left_stick_y);
                                    } else {
                                        // holding
                                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        elevateMid.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        elevate_Left.setTargetPosition(elevate_brake_L);
                                        elevate_Right.setTargetPosition(elevate_brake_R);
                                        elevateMid.setTargetPosition(elevateMid_brake);
                                        elevate_Left.setPower(1);
                                        elevate_Right.setPower(1);
                                        elevateMid.setPower(1);
                                    }
                                } else {
                                    // movement
                                    if (gamepad2.left_stick_y < -0.1 || gamepad2.left_stick_y > 0.1){
                                        if (gamepad2.left_stick_y < .1) {
                                            elevate_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                            elevate_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                            elevate_brake_R = elevate_Left.getCurrentPosition();
                                            elevate_brake_L = elevate_Right.getCurrentPosition();
                                            elevate_Left.setPower(-0.7 * gamepad2.left_stick_y);
                                            elevate_Right.setPower(-0.7 * gamepad2.left_stick_y);
                                        } else {
                                            elevate_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                            elevate_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                            elevate_brake_R = elevate_Left.getCurrentPosition();
                                            elevate_brake_L = elevate_Right.getCurrentPosition();
                                            elevate_Left.setPower(-0.1 * gamepad2.left_stick_y);
                                            elevate_Right.setPower(-0.1 * gamepad2.left_stick_y);
                                        }
                                    } else {
                                        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                        elevate_Left.setTargetPosition(elevate_brake_L);
                                        elevate_Right.setTargetPosition(elevate_brake_R);
                                        elevate_Left.setPower(1);
                                        elevate_Right.setPower(1);
                                    }
                                }
                            }
                        }
                    }
                    //rumble controllers
                    if (claw.getPosition() > 0.95 && distance.getDistance(DistanceUnit.CM) < 7) {
                        gamepad1.rumble(100);
                        gamepad2.rumble(100);
                        gamepad1.setLedColor(1, 1, 1, 300);
                        gamepad2.setLedColor(1, 1, 1, 300);
                    }
                    // telemetry
                    telemetry.addData("Left Joystick", gamepad2.left_stick_y);
                    telemetry.addData("armMode", armMode);
                    telemetry.addData("Arm", arm.getCurrentPosition());
                    telemetry.addData("Elevator Left", elevate_Left.getCurrentPosition());
                    telemetry.addData("Elevator Right", elevate_Right.getCurrentPosition());
                    telemetry.addData("Elevator Middle", elevateMid.getCurrentPosition());
                    telemetry.addData("odopody", front_Right.getCurrentPosition());
                    telemetry.addData("back_Left", back_Leftx.getCurrentPosition());
                    telemetry.addData("distance sensor", distance.getDistance(DistanceUnit.CM));
                    telemetry.update();
                }
            }
        }
    }
}

