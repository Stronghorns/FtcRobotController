package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "ClifforduseRUN")
public class ClifforduseRUN extends LinearOpMode {

  private Servo hook_servo;
  private DcMotor left_front;
  private DcMotor right_front;
  private DcMotor left_back;
  private DcMotor right_back;

  double elapTimeSet;
  ElapsedTime time2;

  /**
   * Describe this function...
   */
  private void closeClaw() {
    hook_servo.setPosition(0);
    while (time2.seconds() < time2.seconds() + 1) {
      telemetry.addData("Grabbing", "Closing Claw");
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void move(double speed, double length) {
    elapTimeSet = time2.seconds() + length;
    while (time2.seconds() < elapTimeSet) {
      left_front.setPower(speed);
      right_front.setPower(speed);
      left_back.setPower(speed);
      right_back.setPower(speed);
      telemetry.addData("Moving forward for", length + " seconds (" + JavaUtil.formatNumber(elapTimeSet - time2.seconds(), 2) + "/" + length + ") at speed " + speed);
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void openClaw() {
    hook_servo.setPosition(0.25);
    while (time2.seconds() < time2.seconds() + 1) {
      telemetry.addData("Releasing", "Opening Claw");
      telemetry.update();
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    hook_servo = hardwareMap.get(Servo.class, "hook_servo");
    left_front = hardwareMap.get(DcMotor.class, "left_front");
    right_front = hardwareMap.get(DcMotor.class, "right_front");
    left_back = hardwareMap.get(DcMotor.class, "left_back");
    right_back = hardwareMap.get(DcMotor.class, "right_back");

    right_back.setDirection(DcMotorSimple.Direction.REVERSE);
    left_back.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    time2 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    if (opModeIsActive()) {
      move(0.5, 0.7);
    }
  }

  /**
   * Describe this function...
   */
  private void wait(double length) {
    elapTimeSet = time2.seconds() + length;
    while (time2.seconds() < elapTimeSet) {
      telemetry.addData("Waiting", Double.parseDouble(JavaUtil.formatNumber(elapTimeSet - time2.seconds(), 2)));
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void turn(double speed, double length) {
    elapTimeSet = time2.seconds() + length;
    while (time2.seconds() < elapTimeSet) {
      left_front.setPower(speed);
      right_front.setPower(-speed);
      left_back.setPower(speed);
      right_back.setPower(-speed);
      telemetry.addData("Turning for", length + " seconds (" + JavaUtil.formatNumber(elapTimeSet - time2.seconds(), 2) + "/" + length + ") at speed " + speed);
      telemetry.update();
    }
  }

  /**
   * Describe this function...
   */
  private void linear(double speed, double length) {
    elapTimeSet = time2.seconds() + length;
    while (time2.seconds() < elapTimeSet) {
      left_back.setPower(speed);
      telemetry.addData("Moving the Linear Slide for", length + " seconds (" + JavaUtil.formatNumber(elapTimeSet - time2.seconds(), 2) + "/" + length + ") at speed " + speed);
      telemetry.update();
    }
  }
}