package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class TeleopFunctions extends Hardwaremap{
    public double driveTrainPower;
    public float Left_Stick_X;
    public float left_stick_y;
    public float Right_Stick_Y;
    public int elevate_brake_L;
    public int elevate_brake_R;
    public boolean reverse = false;

    public boolean clawStatus = false;
    public boolean wristStatus = false;
    public int armMode = 0;
    public int stackHeight = 500;
    public int stackOneClick = 0;
    public int modeOneSwitch = 0;
    public int clawOneClick = 0;
    public int wristOneClick = 0;

    public void preset(int elevatePosition, double elevatePower, double flipPosition, double wristPosition, double clawPosition, int armPosition, double armPower) {
        elevate_Right.setTargetPosition(elevatePosition);
        elevate_Left.setTargetPosition(elevatePosition);
        elevate_Left.setPower(elevatePower);
        elevate_Right.setPower(elevatePower);
        elevate_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevate_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flip.setPosition(flipPosition);
        wrist.setPosition(wristPosition);
        claw.setPosition(clawPosition);
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
    }
}
