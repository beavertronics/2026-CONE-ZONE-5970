package frc.robot.subsystems

import Engine.utils.Units.Angular.asDegrees
import Engine.utils.Units.Angular.degrees
import Engine.utils.Units.Linear.DistanceUnit
import Engine.utils.Units.Linear.meters
import Engine.utils.Units.Linear.metersPerSecond
import Engine.utils.Units.Linear.metersPerSecondSquared
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

object Elevator : SubsystemBase() {

    /**
     * The elevator needs:
     *  - a position controller
     *  - sensors for hard stops
     */

    // establish motors, sensors, etc
    val motor = SparkMax(Constants.elevatorMotorID, SparkLowLevel.MotorType.kBrushless)
    val lowerSwitch = DigitalInput(Constants.elevatorLowerSwitch)
    val upperSwitch = DigitalInput(Constants.elevatorUpperSwitch)

    // establish motion profile for smooth control
    val profileConstraints = TrapezoidProfile.Constraints(
        0.0.metersPerSecond.asMetersPerSecond,
        0.0.metersPerSecondSquared.asMetersPerSecondSquared
    )
    val profile = TrapezoidProfile(profileConstraints)

    // controllers
    // todo https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html
    val pid = PIDController(
        0.0,
        0.0,
        0.0
    )
    val ff = ElevatorFeedforward(
        0.0,
        0.0,
        0.0,
        0.0
    )
    var setpoint = 0.0.meters
    var zero = 0.0.degrees
    var adjusted = 0.0.degrees
        get() = motor.encoder.position.degrees - zero

    init {
        // configure motor
        motor.configure(
            SparkMaxConfig()
                .smartCurrentLimit(Constants.elevatorCurrentLimit)
                .inverted(false), // todo figure out if inversion is needed
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        )

        // configure controllers
        pid.reset()
        pid.setpoint = 0.0.meters.asMeters
        pid.setTolerance(0.25.meters.asMeters)
    }

    override fun periodic() {
        // holds the elevator at a position
        motor.setVoltage(
            12.0 * (pid.calculate(adjusted.asDegrees, setpoint.asMeters) + ff.calculate(adjusted.asDegrees))
        )
    }

    /**
     * Moves the elevator to an inputted height.
     * @param position the height, in meters, to move the elevator to.
     */
    fun setPositionCommand(position: DistanceUnit) : Command {
        return run {
            setpoint = position
        }.until { lowerSwitch.get() || upperSwitch.get() || pid.atSetpoint() }
    }
}