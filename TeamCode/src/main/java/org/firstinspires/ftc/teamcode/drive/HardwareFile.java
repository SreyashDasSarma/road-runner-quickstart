package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class HardwareFile {
    private LinearOpMode linearOpMode;
    public DcMotor intakeMotor, shooter, transportMotor, armWobble;
    public Servo grabberWobble, slapper, tilter;
    HardwareMap map;
    //public SampleMecanumDrive driveTrain;
    public static Pose2d robotPose = new Pose2d();

    public HardwareFile(HardwareMap imported) {
        //robotPose = new Pose2d(x - 70.4725, y - 70.4725, robotOrientation);
        construct(imported);
    }
    private void construct(HardwareMap imported){
        map = imported;
        intakeMotor = map.get(DcMotor.class, "intakeR");
        transportMotor = map.get(DcMotor.class, "intakeL");
        /*shooter = map.get(DcMotor.class, "fw");
        armWobble = map.get(DcMotor.class, "wobbleArm2");
        grabberWobble = map.get(Servo.class, "wobbleGrabber2");
        slapper = map.get(Servo.class, "mag");
        tilter = map.get(Servo.class, "tilt");
        armWobble.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);*/

    }
    /*public void wobbleArmUp() {
        armWobble.setTargetPosition(100);
    }
    public void wobbleArmDown() {
        armWobble.setTargetPosition(100);

    }
    public void wobbleArmVertical(){
        armWobble.setTargetPosition(100);
    }
    public void grab(){
        grabberWobble.setPosition(0.83);
    }
    public void release(){
        grabberWobble.setPosition(0.29);
    }*/
    public void intake(double intakeSpeed){
        intakeMotor.setPower(-intakeSpeed);
    }

    public void shooter(double shooterpower){
        shooter.setPower(shooterpower);
    }
    public void magup(){
        tilter.setPosition(1);
    }
    public void magdown(){
        tilter.setPosition(0.5);
    }

}
