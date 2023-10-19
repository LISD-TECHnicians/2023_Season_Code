// Copyright (c) FIRST and other WPILib contributors.
// Victor Solis III was here
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class Robot extends TimedRobot {

  private final Timer timer = new Timer();
  private final Timer autoTimer = new Timer();
  private final WPI_TalonFX frontRightMotor = new WPI_TalonFX(4, "rio"); // Drive motor instanciation 
  private final WPI_TalonFX rearRightMotor = new WPI_TalonFX(3, "rio"); 
  private final WPI_TalonFX frontLeftMotor = new WPI_TalonFX(5, "rio"); 
  private final WPI_TalonFX rearLeftMotor = new WPI_TalonFX(2, "rio"); 

  //private final TalonFXSensorCollection frontRightSensors = frontRightMotor.getSensorCollection();
  //private final TalonFXSensorCollection rearLeftSensors = rearLeftMotor.getSensorCollection();

  private RelativeEncoder Neo_A_Encoder;
  private RelativeEncoder Neo_C_Encoder; 

  private CANSparkMax brushlessNeo_A = new CANSparkMax(6,MotorType.kBrushless);
  private CANSparkMax brushlessNeo_B = new CANSparkMax(7,MotorType.kBrushless);
  private CANSparkMax brushlessNeo_C = new CANSparkMax(8,MotorType.kBrushless);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
  
  private final Joystick rightStick = new Joystick(0);
  private final Joystick leftStick = new Joystick(1);

  private final DigitalInput upperHoistLimitSwitch = new DigitalInput(1);
  private final DigitalInput lowerHoistLimitSwitch = new DigitalInput(0);

  private final Solenoid gripperSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); // Gyro used for balancing

  final NeutralMode brakeNeutral = NeutralMode.Brake; // Neutral modes for drive system
  final NeutralMode coastNeutral = NeutralMode.Coast;
  
  final float defaultCarriageSpeed = .80f;
  final float defaultArmSpeed = .50f;
  final float defaultArmSpeedRecution = .50f;
  final float upperCarriageLimit = 147.282f;
  final float midCarriageLimit = 90.0f;
  final float lowerCarriageLimit = 4.00f;
  final float outerArmLimit = 44.0f; 
  final float innerArmLimit = 6.0f;
  final double RunOnce = 0;
  final double normAuto = 1;
  final double balAuto = 0.5;
  public double runOneBal = 0;
  double timerStart = 0;
  double timerCurr = autoTimer.get();

  final int encoderUnitsPerRevolution = 2048;
  final double gearboxRatio = 8.45f;
  final double encoderWithGearBox = (encoderUnitsPerRevolution / gearboxRatio);
  final double wheelCercumference = 18.875d; // in inches
  final double encoderUnitsPerInch = (encoderWithGearBox / wheelCercumference);
  
  

  double timeStamp = 0.0f;
  double timeStampFinal = 0.0f;

  Boolean lockDrivePosition;
  Boolean gripperTrigger;
  Boolean reduceRotation;
  Boolean quickGrabButton;
  Boolean quickArmUpButton;
  Boolean middleCube;
  Boolean tableGrabButton;
  Boolean highCube;
  Boolean tablePieceButton;
  double forwardMovement;
  double rotation;
  double liftHoistUpDown;
  double rotateArm;
  double navXPitch;
  boolean autoBalanceButton;
  double LeftSpeed;
  double RightSpeed;
  boolean limeLightTarget;
  boolean LLApril;

 
  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    
    frontRightMotor.configFactoryDefault(); 
    rearRightMotor.configFactoryDefault();
    frontLeftMotor.configFactoryDefault();
    rearLeftMotor.configFactoryDefault();

    brushlessNeo_A.restoreFactoryDefaults();
    brushlessNeo_B.restoreFactoryDefaults();
    brushlessNeo_C.restoreFactoryDefaults();  

    brushlessNeo_A.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true); //Enables a motor limitation for forward 
    brushlessNeo_A.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    brushlessNeo_B.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true); //Enables a motor limitation for forward 
    brushlessNeo_B.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    brushlessNeo_C.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true); //Enables a motor limitation for forward 
    brushlessNeo_C.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true); //Enables a motor limitation for reverse

    brushlessNeo_A.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperCarriageLimit);
    brushlessNeo_A.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, lowerCarriageLimit);
    brushlessNeo_B.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, upperCarriageLimit);
    brushlessNeo_B.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, lowerCarriageLimit);
    brushlessNeo_C.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, outerArmLimit); // one value is one rotation
    brushlessNeo_C.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, innerArmLimit); // limited distance reverse 


    Neo_A_Encoder = brushlessNeo_A.getEncoder(); // Grabs current encoder pos values. 
    Neo_C_Encoder = brushlessNeo_C.getEncoder(); 

    rearRightMotor.follow(frontRightMotor); //rear wheels will follow the front leading wheels
    rearLeftMotor.follow(frontLeftMotor);

    frontRightMotor.setInverted(TalonFXInvertType.CounterClockwise); //May need to run opposite direction
    rearRightMotor.setInverted(InvertType.FollowMaster); //Inversion will match master
    frontLeftMotor.setInverted(TalonFXInvertType.CounterClockwise); // THIS IS THE DIFF DRIVE ERROR (no its not)
    rearLeftMotor.setInverted(InvertType.FollowMaster);

    brushlessNeo_A.setInverted(false);
    brushlessNeo_B.setInverted(true);
    brushlessNeo_C.setInverted(false);
  
  }

  @Override
  public void teleopPeriodic() {
  
    forwardMovement = rightStick.getRawAxis(1);
    rotation = rightStick.getRawAxis(2);
    liftHoistUpDown = leftStick.getRawAxis(1);
    rotateArm = leftStick.getRawAxis(2);
    quickGrabButton = leftStick.getRawButton(4); 
    quickArmUpButton = leftStick.getRawButton(6); 
    middleCube = leftStick.getRawButton(7);
    tableGrabButton = leftStick.getRawButton(3); 
    highCube = leftStick.getRawButton(8);
    lockDrivePosition = rightStick.getRawButton(11);
    reduceRotation = rightStick.getRawButton(2);
    gripperTrigger = leftStick.getRawButtonPressed(1);
    tablePieceButton = leftStick.getRawButtonPressed(5);
    autoBalanceButton = rightStick.getRawButton(12);
    limeLightTarget = leftStick.getRawButton(11);
    LLApril = leftStick.getRawButtonPressed(12);

    frontRightMotor.setNeutralMode(coastNeutral); // Constantly sets neutral mode to coast in teleop
    frontLeftMotor.setNeutralMode(coastNeutral);
    rearRightMotor.setNeutralMode(coastNeutral);
    rearLeftMotor.setNeutralMode(coastNeutral);

    SmartDashboard.putNumber("Joystick Drive", forwardMovement);
    
    // Deadband threshhold for right joystick inputs 
    rotation -= rotation*.40; // The constructor from wpi has the arguments backwards
    // keeps drive code from fighting for motor cmd 
    if (autoBalanceButton){
      autoBalance(0.008);
    }
    else if(limeLightTarget){
      LimeLight();
    } 
    else {
    m_robotDrive.arcadeDrive( rotation, forwardMovement);  
    }

    if (Math.abs(forwardMovement) < 0.08) {forwardMovement = 0;} // deadbands for drive system movememnt
    if (Math.abs(rotation) < 0.08) {rotation = 0;}

    if (reduceRotation) 
    {rotation -= (rotation*.10);
     forwardMovement -= (forwardMovement*.50);
    }
    // stops arm from hitting top when elevator goes up
    if((liftHoistUpDown != 0) == (rotateArm >= 0.03)){
    while(Neo_C_Encoder.getPosition() < 5.5){
      brushlessNeo_C.set(defaultArmSpeed);
    }
    
    
  }
    
    if (!upperHoistLimitSwitch.get())
    { if (liftHoistUpDown >= 0) 
      {liftHoistUpDown = 0;}
      brushlessNeo_A.set(liftHoistUpDown);
      brushlessNeo_B.set(liftHoistUpDown);}

    if (!lowerHoistLimitSwitch.get())
    {if (liftHoistUpDown <= 0)
      {
      brushlessNeo_A.set(0);
      brushlessNeo_B.set(0);}
      brushlessNeo_A.set(liftHoistUpDown);
      brushlessNeo_B.set(liftHoistUpDown);}

    else
    {brushlessNeo_A.set(liftHoistUpDown);
      brushlessNeo_B.set(liftHoistUpDown);}
    
    brushlessNeo_C.set(defaultArmSpeedRecution * rotateArm);
 
     if (gripperTrigger) 
    {gripperSolenoidPCM.toggle();} 
  
    if (lockDrivePosition)
    { 
      frontRightMotor.setNeutralMode(brakeNeutral);
      frontLeftMotor.setNeutralMode(brakeNeutral);
      rearRightMotor.setNeutralMode(brakeNeutral);
      rearLeftMotor.setNeutralMode(brakeNeutral);}

    else
    { 
      frontRightMotor.setNeutralMode(coastNeutral);
      frontLeftMotor.setNeutralMode(coastNeutral);
      rearRightMotor.setNeutralMode(coastNeutral);
      rearLeftMotor.setNeutralMode(coastNeutral);}

    if (quickGrabButton) {downGrabUp();}
    if (quickArmUpButton) {upArm();}
    if (middleCube) {scoreCubeMiddle();}
    if (tableGrabButton) {grabTablePiece();}
    if (tablePieceButton) {upToTablePiece();}
    if (highCube) {scoreCubeHigh();}
  }

    public void LimeLight(){
      double KpAim = 0.1;
      double KpDrive = 0.1;
      double minAimCommand = 0.1;
      double minDriveCommand = 0.1;
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      double HeadingError = -tx;
      double steering = 0;
      double distanceError = -ty;
      if (tx > 1.0){
        steering = KpAim * HeadingError - minAimCommand;
      }
      if (tx < -1.0){
        steering = KpAim * HeadingError + minAimCommand;
      }
      double drive = KpDrive * distanceError;
      SmartDashboard.putNumber("Left Speed", LeftSpeed);
      SmartDashboard.putNumber("Right Speed", RightSpeed);
      LeftSpeed = steering + drive;
      RightSpeed = -steering + drive;
      setSpeed(LeftSpeed, RightSpeed);
    }
  // balance tied to a button
    public void autoBalance(double speed){
      navXPitch = gyro.getRoll();
      double forwardCMD = (navXPitch * speed);
      //System.out.println("NavX Drive " + navXPitch);
    if((navXPitch > -7) && (navXPitch < 7)){
      setSpeed(0, 0);
      frontRightMotor.setNeutralMode(brakeNeutral);
      frontLeftMotor.setNeutralMode(brakeNeutral);
      rearRightMotor.setNeutralMode(brakeNeutral);
      rearLeftMotor.setNeutralMode(brakeNeutral);
      }else{
        frontRightMotor.setNeutralMode(coastNeutral);
        frontLeftMotor.setNeutralMode(coastNeutral);
        rearRightMotor.setNeutralMode(coastNeutral);
        rearLeftMotor.setNeutralMode(coastNeutral);
      }
      
        LeftSpeed = forwardCMD;
        RightSpeed = -forwardCMD;

        setSpeed(LeftSpeed, RightSpeed);
      }
    public void setSpeed(double LeftSpeed, double RightSpeed){
      frontLeftMotor.set(ControlMode.PercentOutput, LeftSpeed);
      frontRightMotor.set(ControlMode.PercentOutput, RightSpeed);
    }
    public void downGrabUp()
    {
      if (Neo_A_Encoder.getPosition() >= lowerCarriageLimit)
      {brushlessNeo_A.set(-defaultCarriageSpeed);
        brushlessNeo_B.set(-defaultCarriageSpeed);}

      if (Neo_C_Encoder.getPosition() <= outerArmLimit)
      {brushlessNeo_C.set(defaultArmSpeed);}
    }

    public void upArm()
    {
      gripperSolenoidPCM.set(false);

      if (Neo_A_Encoder.getPosition() >= midCarriageLimit)
      {
        brushlessNeo_A.set(-defaultCarriageSpeed);
        brushlessNeo_B.set(-defaultCarriageSpeed);
      }
      if  (Neo_C_Encoder.getPosition() >= innerArmLimit)
      {
        brushlessNeo_C.set(-defaultArmSpeed);
      }
      if  (Neo_A_Encoder.getPosition() >= lowerCarriageLimit)
      {
        brushlessNeo_A.set(-defaultCarriageSpeed);
        brushlessNeo_B.set(-defaultCarriageSpeed);
      }
    }

    public void scoreCubeMiddle()
    {
      if (Neo_A_Encoder.getPosition() >= midCarriageLimit) {carriageDownMiddle();}
    
      else {carriageUpMiddle();}
      
      if (Neo_C_Encoder.getPosition() >= 40.00)
      {
        armPositionOut();
      }
      else {armPositionIn();}
    }

    

    public void grabTablePiece()
    {
      gripperSolenoidPCM.set(false);
      if (Neo_C_Encoder.getPosition() <= outerArmLimit / 2)
      {
        brushlessNeo_C.set(defaultArmSpeed * .5);
      }
      
     
      if (Neo_A_Encoder.getPosition() <= 125.0)
      {
        brushlessNeo_A.set(defaultCarriageSpeed);
        brushlessNeo_B.set(defaultCarriageSpeed);
      }
    }

    public void upToTablePiece()
    {
      //Reverse order of logic for grapTablePiece Function
      //Victor Was here
      //Drive carriage up to value of 125 if below value of 125
      if (Neo_A_Encoder.getPosition() <= 125.0)
      {
        brushlessNeo_A.set(defaultCarriageSpeed);
        brushlessNeo_B.set(defaultCarriageSpeed);
      }
      
      //Drive arm out to outerlimit value aprx. 44 
      if (Neo_C_Encoder.getPosition() <= outerArmLimit)
      {
        brushlessNeo_C.set(defaultArmSpeed * .5);
      }

    }
    
    public void scoreCubeHighAUTO(double moveTime, double moveSpeed)
    {
      while (Neo_C_Encoder.getPosition() <= 20.5)
      {
        brushlessNeo_C.set(.25 * moveSpeed);
      }
      brushlessNeo_C.set(0);
      while (Neo_A_Encoder.getPosition() <= 110.00)
      {
        brushlessNeo_A.set(.5 * moveSpeed);
        brushlessNeo_B.set(.5 * moveSpeed);
      }
      brushlessNeo_A.set(0);
      brushlessNeo_B.set(0);
      while (Neo_C_Encoder.getPosition() <= 30.5)
      {
        brushlessNeo_C.set(.25 * moveSpeed);
      }
      brushlessNeo_C.set(0);
      
      gripperSolenoidPCM.set(true);
      Timer.delay(1);
      frontRightMotor.set(.25);
      rearRightMotor.set(.25);
      frontLeftMotor.set(-.25);
      rearLeftMotor.set(-.25);
      Timer.delay(moveTime);
      frontRightMotor.set(0);
      rearRightMotor.set(0);
      frontLeftMotor.set(0);
      rearLeftMotor.set(0);

      while (Neo_A_Encoder.getPosition() >= 8.00)
      {
        brushlessNeo_A.set(-.5 * moveSpeed);
        brushlessNeo_B.set(-.5 * moveSpeed);
      }
      brushlessNeo_A.set(0);
      brushlessNeo_B.set(0);
      while (Neo_C_Encoder.getPosition() >= 6.00)
      {
        brushlessNeo_C.set(-.25 * moveSpeed);
      }
      brushlessNeo_C.set(0);

    }

    public void scoreCubeHigh()
    {
      if (Neo_C_Encoder.getPosition() <= outerArmLimit / 2)
      {
        brushlessNeo_C.set(defaultArmSpeed*.5);
      }
      brushlessNeo_C.set(0);
      if (Neo_A_Encoder.getPosition() <= 130.00)
      {
        brushlessNeo_A.set(.5);
        brushlessNeo_B.set(.5);
      }
      brushlessNeo_A.set(0);
      brushlessNeo_B.set(0);
      if (Neo_C_Encoder.getPosition() <= 30.5)
      {
        brushlessNeo_C.set(.25);
      }
      brushlessNeo_C.set(0);
      
    }

    
    public void carriageDownMiddle()
    {
      if (Neo_A_Encoder.getPosition() >= midCarriageLimit)
      {
        brushlessNeo_A.set(-defaultArmSpeed);
        brushlessNeo_B.set(-defaultArmSpeed);
      }
    }
    
    public void carriageUpMiddle()
    {
      if (Neo_A_Encoder.getPosition() <= midCarriageLimit) {
        brushlessNeo_A.set(defaultArmSpeed);
        brushlessNeo_B.set(defaultArmSpeed);
      }
    }

    public void armPositionOut()
    {
      if (Neo_C_Encoder.getPosition() >= outerArmLimit / 2)
      {
      brushlessNeo_C.set(defaultArmSpeed);
      }
    }

    public void armPositionIn()
    {
      brushlessNeo_C.set(-defaultArmSpeed); 
    }
    
    public void driveRobotStraightold(double InchesTraveled, double runOnce) 
    {
    
      if (runOnce == 0){
      frontRightMotor.setSelectedSensorPosition(0);
      frontLeftMotor.setSelectedSensorPosition(0);
      runOnce += 1;
      }
      System.out.println("runonce " + runOnce);
      while ((((-frontRightMotor.getSelectedSensorPosition()) + frontLeftMotor.getSelectedSensorPosition())/2) >= inchesToEncoder(InchesTraveled))
      {
        boolean speedSet = (((-frontRightMotor.getMotorOutputPercent()) + frontLeftMotor.getMotorOutputPercent())/2 == 0);
        
        if ((((-frontRightMotor.getMotorOutputPercent()) + frontLeftMotor.getMotorOutputPercent())/2) < 0.1) {
          if (-0.1 < (((-frontRightMotor.getMotorOutputPercent()) + frontLeftMotor.getMotorOutputPercent())/2)) {
            frontRightMotor.set(.25);
            rearRightMotor.set(.25);
            frontLeftMotor.set(-.25);
            rearLeftMotor.set(-.25);
         }
        }
        System.out.println("speedset : " + speedSet);
      } 
      while ((((-frontRightMotor.getSelectedSensorPosition()) + frontLeftMotor.getSelectedSensorPosition())/2) < inchesToEncoder(InchesTraveled))
      {
         System.out.println("stop motors");
         stopMotor();
      }  
    }
    
    public void stopMotor(){
      // frontRightMotor.stopMotor();
      frontRightMotor.set(0);
      rearRightMotor.set(0);
      frontLeftMotor.set(0);
      rearLeftMotor.set(0);
    }
    public double inchesToEncoder(double inches) {
      double encoderUnits = encoderUnitsPerInch * inches;
      return encoderUnits;
    }
   // old delay 3.5 
   public void driveRobotStraight(double delay, double speed) // Method to drive robot backwards 8 ft in AUTO
   {
     
     frontRightMotor.set(speed);
     rearRightMotor.set(speed);
     frontLeftMotor.set(-speed);
     rearLeftMotor.set(-speed);
     Timer.delay(delay);
     frontRightMotor.set(0);
     rearRightMotor.set(0);
     frontLeftMotor.set(0);
     rearLeftMotor.set(0);
 }

    @Override 
    public void autonomousInit() //Initialization of autonomous mode 
    {
    
      
    rearRightMotor.follow(frontRightMotor); //rear wheels will follow the front leading wheels
    rearLeftMotor.follow(frontLeftMotor);

    frontRightMotor.setInverted(TalonFXInvertType.CounterClockwise); //May need to run opposite direction
    rearRightMotor.setInverted(InvertType.FollowMaster); //Inversion will match master
    frontLeftMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rearLeftMotor.setInverted(InvertType.FollowMaster);

    brushlessNeo_A.setInverted(false);
    brushlessNeo_B.setInverted(true);
    brushlessNeo_C.setInverted(false);
    runOneBal = 0;
    autoTimer.reset();
    autoTimer.start();
    
    
    }
    
    @Override 
    public void autonomousPeriodic() 
    {
      // norm 1, comp bal 2, bal 3
      double autoSelect = 3;
      double curTime = autoTimer.get();

      //start of bal

      if(autoSelect == 3){
      timerStart = autoTimer.get();
      if (runOneBal ==0){
      scoreCubeHighAUTO(0.5,1);
      }
      runOneBal = 1;
      if (runOneBal == 1){
      driveRobotStraight(1.6, 0.35);
      runOneBal = 2;
      }
      curTime = autoTimer.get();
      while(curTime < 14.5){
        //System.out.print("timer " + autoTimer.get());
      autoBalance(0.013);
      frontRightMotor.setNeutralMode(brakeNeutral);
      frontLeftMotor.setNeutralMode(brakeNeutral);
      rearRightMotor.setNeutralMode(brakeNeutral);
      rearLeftMotor.setNeutralMode(brakeNeutral);
      curTime = autoTimer.get();
    }
    stopMotor();
    Timer.delay(4);
  }
      //end of bal
      // start of comp bal
      
      if(autoSelect == 2){
      timerStart = autoTimer.get();
      if (runOneBal ==0){
      scoreCubeHighAUTO(0.25,2);
      }
      runOneBal = 1;
      if (runOneBal == 1){
      driveRobotStraight(3.1, 0.35);
      runOneBal = 2;
      }
      if (runOneBal == 2){
        driveRobotStraight(1, -0.4);
        runOneBal = 3;
      }
      
      curTime = autoTimer.get();
      while(curTime < 14.9){
        //System.out.print("timer " + autoTimer.get());
      autoBalance(0.013);
      curTime = autoTimer.get();
    }
    stopMotor();
  }
    
    // end of auto bal comp
    // start of norm auto 
    if(autoSelect == 1){
    if(runOneBal == 0){
      scoreCubeHighAUTO(.5,1);
     runOneBal = 1; 
  }
  
    if(runOneBal == 1){
    driveRobotStraight(3.5,0.25);
    runOneBal = 2;
  } 
}
    }
}