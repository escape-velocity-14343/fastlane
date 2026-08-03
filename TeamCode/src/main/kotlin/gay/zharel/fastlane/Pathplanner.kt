package gay.zharel.fastlane

import com.seattlesolvers.solverslib.command.button.Trigger
import dev.nextftc.control.geometry.Pose2d
import dev.nextftc.control.geometry.Rotation2d
import dev.nextftc.control.geometry.Vector2d
import dev.nextftc.units.Degrees
import dev.nextftc.units.Inches
import dev.nextftc.units.MetersPerSecond
import dev.nextftc.units.Radians
import dev.nextftc.units.measuretypes.Angle
import dev.nextftc.units.measuretypes.LinearVelocity
import dev.nextftc.units.measuretypes.Voltage
import dev.nextftc.units.unittypes.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.json.JSONObject
import java.io.File
import java.util.*
import java.util.function.DoubleSupplier
import java.util.function.Supplier

// this file is for parsing and interpreting pathplanner stuff
object PathplannerFileManager {
    val ROOT = AppUtil.ROOT_FOLDER.resolve("src/main/deploy/pathplanner")

    val paths: List<File> by lazy {
        ROOT.resolve("paths").listFiles()?.filter { it.isFile } ?: emptyList()
    }

    val autos: List<File> by lazy {
        ROOT.resolve("autos").listFiles()?.filter { it.isFile } ?: emptyList()
    }

    fun getPathFile(name: String): File {
        return if (name.endsWith(".path")) ROOT.resolve("paths/$name"); else ROOT.resolve("paths/$name.path")
    }

    fun getAutoFile(name: String): File {
        return if (name.endsWith(".auto")) ROOT.resolve("autos/$name"); else ROOT.resolve("autos/$name.auto")
    }
}

object PathPlannerParser {

    /**
     * Parses a PathPlanner .path file into a PathplannerPath object.
     */
    fun parse(file: File): PathplannerPath {
        val jsonString = file.readText()
        val root = JSONObject(jsonString)

        // 1. Parse Start and End Rotations
        // Fallback to 0.0 if the state is missing (some older pathplanner versions)
        val startRotationDegrees = root.optJSONObject("idealStartingState")?.optDouble("rotation", 0.0) ?: 0.0
        val endRotationDegrees = root.optJSONObject("goalEndState")?.optDouble("rotation", 0.0) ?: 0.0

        // 2. Parse Waypoints (Anchors)
        // In PathPlanner, 't' represents the index of the anchor in the sequence.
        val waypointsArray = root.getJSONArray("waypoints")
        val anchors = List(waypointsArray.length()) { i ->
            val anchorJson = waypointsArray.getJSONObject(i).getJSONObject("anchor")
            PathplannerWaypoint(
                t = i.toDouble(),
                // Note: Depending on your exact unit library, you may need to wrap these
                // raw doubles, e.g., Vector2d(Meters(x), Meters(y))
                pos = Vector2d(Inches.of(anchorJson.getDouble("x")), Inches.of(anchorJson.getDouble("y")))
            )
        }

        // 3. Parse Rotation Targets
        val rotationTargetsArray = root.optJSONArray("rotationTargets")
        val rotationTargets = if (rotationTargetsArray != null) {
            List(rotationTargetsArray.length()) { i ->
                val rtJson = rotationTargetsArray.getJSONObject(i)
                PathplannerRotationTarget(
                    pos = rtJson.getDouble("waypointRelativePos"),
                    // Adjust to your Angle constructor (e.g., Degrees(val))
                    angle = Degrees.of(rtJson.getDouble("rotationDegrees"))
                )
            }
        } else emptyList()

        // 4. Parse Constraint Zones
        val constraintZonesArray = root.optJSONArray("constraintZones")
        val constraintZones = if (constraintZonesArray != null) {
            List(constraintZonesArray.length()) { i ->
                val czJson = constraintZonesArray.getJSONObject(i)
                val constraintsJson = czJson.getJSONObject("constraints")
                PathplannerConstraint(
                    startPos = czJson.getDouble("minWaypointRelativePos"),
                    endPos = czJson.getDouble("maxWaypointRelativePos"),
                    // Adjust to your LinearVelocity constructor
                    topSpeed = MetersPerSecond.of(constraintsJson.getDouble("maxVelocity"))
                )
            }
        } else emptyList()

        // 5. Parse Event Markers
        val eventMarkersArray = root.optJSONArray("eventMarkers")
        val eventMarkers = if (eventMarkersArray != null) {
            List(eventMarkersArray.length()) { i ->
                val emJson = eventMarkersArray.getJSONObject(i)
                val endPos = if (emJson.isNull("endWaypointRelativePos")) {
                    null
                } else {
                    emJson.getDouble("endWaypointRelativePos")
                }

                PathplannerEvent(
                    name = emJson.getString("name"),
                    startPos = emJson.getDouble("waypointRelativePos"),
                    endPos = endPos
                )
            }
        } else emptyList()

        // 6. Construct and Return the Path
        return PathplannerPath(
            startRotation = Degrees.of(startRotationDegrees),
            endRotation = Degrees.of(endRotationDegrees),
            anchors = anchors,
            rotationTargets = rotationTargets,
            constraintZones = constraintZones,
            eventMarkers = eventMarkers,
            pathName = file.nameWithoutExtension
        )
    }
}

// handles named events or whatever
object NamedEventManager {
    lateinit var currentT: DoubleSupplier
    lateinit var lastT: DoubleSupplier
    lateinit var pathName: Supplier<String>
    val map: MutableMap<String, Trigger> = mutableMapOf()

    fun get(name: String) = map.getOrDefault(name, Trigger())
}

data class PathplannerWaypoint(
    val t: Double,
    val pos: Vector2d<DistanceUnit>
) {
    val fastlane: Waypoint get() = Waypoint(pos)
}

data class PathplannerEvent(
    val name: String,
    val startPos: Double,
    val endPos: Double?
)

data class PathplannerConstraint(
    val startPos: Double,
    val endPos: Double,
    val topSpeed: LinearVelocity
) {
    val fastlane: ConstraintZone get() = ConstraintZone(startPos, endPos,
        Throttle.of(topSpeed.into(MetersPerSecond)))
}

data class PathplannerRotationTarget(
    val pos: Double,
    val angle: Angle
) {
    val fastlane: RotationTarget get() = RotationTarget(
        pos,
        Rotation2d.fromAngle(angle)
    )
}

data class PathplannerPath(
    val startRotation: Angle,
    val endRotation: Angle,
    val anchors: List<PathplannerWaypoint>,
    val rotationTargets: List<PathplannerRotationTarget>,
    val constraintZones: List<PathplannerConstraint>,
    val eventMarkers: List<PathplannerEvent>,
    val pathName: String
) {

    val fastlanePath: FastlanePath get() {

        // add start and end
        val rotTargets: MutableList<PathplannerRotationTarget> =
            mutableListOf(PathplannerRotationTarget(0.0, startRotation))

        rotTargets.addAll(rotationTargets)
        rotTargets.add(PathplannerRotationTarget(anchors.size - 1.0, endRotation))

        eventMarkers.forEach {
            if (it.endPos != null) {
                NamedEventManager.map[it.name] =
                    NamedEventManager.map.getOrDefault(it.name, Trigger())
                        .or(Trigger { it.endPos > NamedEventManager.currentT.asDouble
                                && NamedEventManager.currentT.asDouble > it.startPos
                                && NamedEventManager.pathName.get() == pathName
                        })
            } else {
                NamedEventManager.map[it.name] =
                    NamedEventManager.map.getOrDefault(it.name, Trigger())
                        .or(Trigger { it.startPos > NamedEventManager.lastT.asDouble
                                && NamedEventManager.currentT.asDouble > it.startPos
                                && NamedEventManager.pathName.get() == pathName})
            }
        }

        return FastlanePath(
            anchors.map { it.fastlane },
            rotTargets.map { it.fastlane },
            constraintZones.map { it.fastlane },
            pathName,
        )
    }
}