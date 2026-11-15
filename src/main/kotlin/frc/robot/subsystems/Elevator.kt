package frc.robot.subsystems

import Engine.utils.Units.Angular.RPM
import Engine.utils.Units.Angular.asDegrees
import Engine.utils.Units.Angular.asRPM
import Engine.utils.Units.Angular.asRotations
import Engine.utils.Units.Angular.degrees
import Engine.utils.Units.Angular.rotations
import Engine.utils.Units.Electrical.volts
import Engine.utils.Units.Linear.DistanceUnit
import Engine.utils.Units.Linear.meters
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLimitSwitch
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkSoftLimit
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.EncoderConfig
import com.revrobotics.spark.config.FeedForwardConfig
import com.revrobotics.spark.config.MAXMotionConfig
import com.revrobotics.spark.config.SoftLimitConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

object Elevator : SubsystemBase() {

    // establish motors, sensors, etc
    val motor = SparkMax(Constants.elevatorMotorID, SparkLowLevel.MotorType.kBrushless)
    val controller = motor.closedLoopController
    val lowerSwitch = DigitalInput(Constants.elevatorLowerSwitch)
    val upperSwitch = DigitalInput(Constants.elevatorUpperSwitch)

    // establish values
    val elevatorHeight = 0.0.meters
    var setpoint = 0.0.meters
    var lowerBound = 0.0.degrees
    var upperBound = 0.0.degrees

    init {
        // configure motor
        val config = SparkMaxConfig()
            .smartCurrentLimit(Constants.elevatorCurrentLimit)
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .inverted(false) // todo

        // for PID, FF, encoder and motion profiling
        // todo https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html
        // todo https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control
        val controlConfig = ClosedLoopConfig()
            .pid(0.0, 0.0, 0.0)
        val ffConfig = FeedForwardConfig()
            .kS(0.0)
            .kV(0.0)
            .kA(0.0)
            .kG(0.0)
        val motionConfig = MAXMotionConfig()
            .maxAcceleration(50.0) // rpm per second
            .cruiseVelocity(30.0.RPM.asRPM)
            .allowedProfileError(5.0.rotations.asRotations)
        val encoderConfig = EncoderConfig()
            .positionConversionFactor(360.0.degrees.asDegrees)
            .inverted(false) // todo
        val limitsConfig = SoftLimitConfig()
            .forwardSoftLimit(upperBound.asDegrees)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(lowerBound.asDegrees)
            .reverseSoftLimitEnabled(true)

        // applying all configs
        controlConfig
            .apply(ffConfig)
            .apply(motionConfig)
        config
            .apply(controlConfig)
            .apply(encoderConfig)
            .apply(limitsConfig)
        motor.configure(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        )
    }

    override fun periodic() {
        // holds the elevator at a position
        controller.setSetpoint(setpoint.asMeters, SparkBase.ControlType.kMAXMotionPositionControl)
    }

    /**
     * Moves the elevator to an inputted height.
     * @param position the height, in meters, to move the elevator to.
     */
    fun setPositionCommand(position: DistanceUnit) : Command {
        return run {
            setpoint = position
        }.until { controller.isAtSetpoint() }
    }

    /**
     * Finds and saves the lower and upper bounds of the elevator.
     */
    fun findBoundsCommand() : Command {
        return run {
            motor.setVoltage(-1.0.volts.asVolts) // todo
        }
            .until { lowerSwitch.get() }
            .andThen({
                motor.stopMotor()
                lowerBound = motor.encoder.position.degrees
            })
            .andThen({ motor.setVoltage(1.0.volts.asVolts) }) // todo
            .until { upperSwitch.get() }
            .andThen({
                motor.stopMotor()
                upperBound = motor.encoder.position.degrees
            })
            .andThen({
                // generate a new config and apply it
                val newLimitsConfig = SoftLimitConfig()
                    .forwardSoftLimit(upperBound.asDegrees)
                    .forwardSoftLimitEnabled(true)
                    .reverseSoftLimit(lowerBound.asDegrees)
                    .reverseSoftLimitEnabled(true)
                motor.configure(
                    SparkMaxConfig().apply(newLimitsConfig),
                    ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters
                )
            })
    }
}