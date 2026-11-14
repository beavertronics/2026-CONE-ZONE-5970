package frc.robot.subsystems

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.LimitSwitchConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DigitalInput
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

    init {
        // configure motor
        motor.configure(
            SparkMaxConfig()
                .smartCurrentLimit(Constants.elevatorCurrentLimit)
                .inverted(false), // todo figure out if inversion is needed
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        )
    }

    override fun periodic() {
        super.periodic()
    }
}