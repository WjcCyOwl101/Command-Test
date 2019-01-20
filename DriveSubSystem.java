/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;

/**
 * Add your docs here.
 */
public class DriveSubSystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Motor controller objects
  public WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.leftMasterPort);
  public WPI_TalonSRX leftSlave = new WPI_TalonSRX(RobotMap.leftSlavePort);
  public WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.rightMasterPort);
  public WPI_TalonSRX rightSlave = new WPI_TalonSRX(RobotMap.rightSlavePort);

  // New differential drive object
  // assigne motor controller to differential drive
  public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  // Create constructor function
  public DriveSubSystem(){
    // Point Slaves to Masters
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }

  // Add manualDrive() method
  public void manualDrive(double move, double turn){
    
    if (Math.abs(move) > .10) {
      move = 0;
    }
    
    if (Math.abs(turn) > .10) {
      turn = 0;
    }
    
    drive.arcadeDrive(move, turn);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveManuallyCommand());
  }

  /**
   * @return the leftMaster
   */
  public WPI_TalonSRX getLeftMaster() {
    return leftMaster;
  }

  /**
   * @param leftMaster the leftMaster to set
   */
  public void setLeftMaster(WPI_TalonSRX leftMaster) {
    this.leftMaster = leftMaster;
  }

  /**
   * @return the leftSlave
   */
  public WPI_TalonSRX getLeftSlave() {
    return leftSlave;
  }

  /**
   * @param leftSlave the leftSlave to set
   */
  public void setLeftSlave(WPI_TalonSRX leftSlave) {
    this.leftSlave = leftSlave;
  }

  /**
   * @return the rightMaster
   */
  public WPI_TalonSRX getRightMaster() {
    return rightMaster;
  }

  /**
   * @param rightMaster the rightMaster to set
   */
  public void setRightMaster(WPI_TalonSRX rightMaster) {
    this.rightMaster = rightMaster;
  }

  /**
   * @return the rightSlave
   */
  public WPI_TalonSRX getRightSlave() {
    return rightSlave;
  }

  /**
   * @param rightSlave the rightSlave to set
   */
  public void setRightSlave(WPI_TalonSRX rightSlave) {
    this.rightSlave = rightSlave;
  }

  /**
   * @return the drive
   */
  public DifferentialDrive getDrive() {
    return drive;
  }

  /**
   * @param drive the drive to set
   */
  public void setDrive(DifferentialDrive drive) {
    this.drive = drive;
  }
}
