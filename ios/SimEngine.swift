import Foundation
import CoreGraphics

// MARK: - Model types

struct Obstacle {
    let rect: CGRect
}

enum RoverDecision: String {
    case forward, turnLeft, turnRight, stop

    var label: String {
        switch self {
        case .forward: return "forward"
        case .turnLeft: return "turn left"
        case .turnRight: return "turn right"
        case .stop: return "stop"
        }
    }
}

struct DecisionResult {
    let decision: RoverDecision
    let confidence: Double
    let fellBack: Bool
}

struct SensorReading {
    var left: CGFloat = 150
    var front: CGFloat = 150
    var right: CGFloat = 150
}

struct LogEntry: Identifiable {
    let id = UUID()
    let text: String
    let isCollision: Bool
}

/// Drives the rover: sensing, the decision step, wind, and collisions.
/// Mirrors the web version's control loop one-to-one so behavior matches.
final class SimEngine: ObservableObject {

    // Arena
    let arenaSize = CGSize(width: 340, height: 460)
    let margin: CGFloat = 8
    let obstacles: [Obstacle]

    // Rover
    @Published var roverPos: CGPoint
    @Published var roverHeading: CGFloat = 0.35
    let roverRadius: CGFloat = 9
    let roverSpeed: CGFloat = 58
    let turnRate: CGFloat = 2.6

    let maxRange: CGFloat = 140
    let danger: CGFloat = 40
    let fallbackConfidence: Double = 0.33
    let decisionInterval: TimeInterval = 0.15

    // Wind
    @Published var windAngle: Double
    @Published var windSpeed: Double = 0.6
    @Published var windMaxSpeed: Double = 1.4
    @Published var windGust: Double = 0.5
    @Published var windAware: Bool = true

    // Live readouts
    @Published var sensors = SensorReading()
    @Published var lastResult = DecisionResult(decision: .forward, confidence: 0.7, fellBack: false)

    // Stats
    @Published var ticks: Int = 0
    @Published var collisions: Int = 0
    @Published var fallbacks: Int = 0
    @Published var confidenceSum: Double = 0
    @Published var confidenceCount: Int = 0
    @Published var log: [LogEntry] = []

    @Published var running: Bool = true

    private var timeSinceLastDecision: TimeInterval = 0

    init() {
        obstacles = [
            Obstacle(rect: CGRect(x: 55, y: 65, width: 100, height: 20)),
            Obstacle(rect: CGRect(x: 140, y: 155, width: 20, height: 150)),
            Obstacle(rect: CGRect(x: 190, y: 50, width: 120, height: 20)),
            Obstacle(rect: CGRect(x: 260, y: 235, width: 20, height: 170)),
            Obstacle(rect: CGRect(x: 95, y: 355, width: 140, height: 20)),
            Obstacle(rect: CGRect(x: 210, y: 300, width: 75, height: 20))
        ]
        roverPos = CGPoint(x: 36, y: 36)
        windAngle = Double.random(in: 0..<(2 * .pi))
        sensors = Self.sense(from: roverPos, heading: roverHeading, obstacles: obstacles,
                              arenaSize: arenaSize, margin: margin, maxRange: maxRange)
    }

    // MARK: Sensing

    private static func blocked(_ p: CGPoint, obstacles: [Obstacle], arenaSize: CGSize, margin: CGFloat) -> Bool {
        if p.x < margin || p.x > arenaSize.width - margin || p.y < margin || p.y > arenaSize.height - margin {
            return true
        }
        for o in obstacles where o.rect.contains(p) { return true }
        return false
    }

    private static func cast(from origin: CGPoint, angle: CGFloat, obstacles: [Obstacle],
                              arenaSize: CGSize, margin: CGFloat, maxRange: CGFloat) -> CGFloat {
        let step: CGFloat = 4
        var dist: CGFloat = 0
        while dist < maxRange {
            dist += step
            let p = CGPoint(x: origin.x + cos(angle) * dist, y: origin.y + sin(angle) * dist)
            if blocked(p, obstacles: obstacles, arenaSize: arenaSize, margin: margin) { return max(0, dist - step) }
        }
        return maxRange
    }

    private static func sense(from pos: CGPoint, heading: CGFloat, obstacles: [Obstacle],
                               arenaSize: CGSize, margin: CGFloat, maxRange: CGFloat) -> SensorReading {
        var s = SensorReading()
        s.left = cast(from: pos, angle: heading - 0.78, obstacles: obstacles, arenaSize: arenaSize, margin: margin, maxRange: maxRange)
        s.front = cast(from: pos, angle: heading, obstacles: obstacles, arenaSize: arenaSize, margin: margin, maxRange: maxRange)
        s.right = cast(from: pos, angle: heading + 0.78, obstacles: obstacles, arenaSize: arenaSize, margin: margin, maxRange: maxRange)
        return s
    }

    private func readSensorsNow() -> SensorReading {
        Self.sense(from: roverPos, heading: roverHeading, obstacles: obstacles,
                   arenaSize: arenaSize, margin: margin, maxRange: maxRange)
    }

    // MARK: The decision engine
    //
    // Pure function of the current state: one typed decision plus a
    // confidence score out.
    private func decideAction() -> DecisionResult {
        var decision: RoverDecision
        var confidence: Double

        if sensors.front < danger {
            decision = sensors.left > sensors.right ? .turnLeft : .turnRight
            confidence = clamp(Double(abs(sensors.left - sensors.right) / maxRange) + 0.45, 0, 1)
        } else if windAware {
            let lateral = sin(CGFloat(windAngle) - roverHeading) * CGFloat(windSpeed)
            if abs(lateral) > 0.55 {
                decision = lateral > 0 ? .turnLeft : .turnRight
                confidence = clamp(Double(abs(lateral)) / windMaxSpeed, 0.25, 0.9)
            } else {
                decision = .forward
                confidence = clamp(Double(sensors.front / maxRange), 0.55, 0.97)
            }
        } else {
            decision = .forward
            confidence = clamp(Double(sensors.front / maxRange), 0.5, 0.95)
        }

        confidence = clamp(confidence + Double.random(in: -0.04...0.04), 0.05, 0.99)

        var fellBack = false
        if confidence < fallbackConfidence {
            decision = .stop
            fellBack = true
        }

        return DecisionResult(decision: decision, confidence: confidence, fellBack: fellBack)
    }

    private func clamp(_ v: Double, _ a: Double, _ b: Double) -> Double { max(a, min(b, v)) }

    // MARK: Per-frame step

    func update(dt: TimeInterval) {
        guard running else { return }
        let dt = min(dt, 0.05) // clamp in case of a stall/backgrounding

        updateWind(dt: dt)
        applyWindPush(dt: dt)
        resolveCollisions()

        timeSinceLastDecision += dt
        if timeSinceLastDecision >= decisionInterval {
            timeSinceLastDecision = 0
            decisionTick()
        }
    }

    private func updateWind(dt: TimeInterval) {
        let driftRate = 0.6 * windGust
        let gustRate = 0.5 * windGust
        windAngle += Double.random(in: -0.5...0.5) * driftRate * dt
        windSpeed = clamp(windSpeed + Double.random(in: -0.5...0.5) * gustRate * dt, 0, windMaxSpeed)
    }

    private func applyWindPush(dt: TimeInterval) {
        let pushScale: CGFloat = 24
        roverPos.x += cos(CGFloat(windAngle)) * CGFloat(windSpeed) * pushScale * CGFloat(dt)
        roverPos.y += sin(CGFloat(windAngle)) * CGFloat(windSpeed) * pushScale * CGFloat(dt)
    }

    private func resolveCollisions() {
        var collided = false
        if roverPos.x < margin + roverRadius { roverPos.x = margin + roverRadius; collided = true }
        if roverPos.x > arenaSize.width - margin - roverRadius { roverPos.x = arenaSize.width - margin - roverRadius; collided = true }
        if roverPos.y < margin + roverRadius { roverPos.y = margin + roverRadius; collided = true }
        if roverPos.y > arenaSize.height - margin - roverRadius { roverPos.y = arenaSize.height - margin - roverRadius; collided = true }

        for o in obstacles {
            let cx = min(max(roverPos.x, o.rect.minX), o.rect.maxX)
            let cy = min(max(roverPos.y, o.rect.minY), o.rect.maxY)
            let dx = roverPos.x - cx
            let dy = roverPos.y - cy
            let d = sqrt(dx * dx + dy * dy)
            if d < roverRadius {
                collided = true
                let push = (roverRadius - d) + 0.5
                if d > 0.001 {
                    roverPos.x += (dx / d) * push
                    roverPos.y += (dy / d) * push
                } else {
                    roverPos.x += push
                }
                roverHeading += .pi * 0.5 * CGFloat(Bool.random() ? 1 : -1) * 0.3
            }
        }
        if collided {
            collisions += 1
            addLog("collision — bounced clear", isCollision: true)
        }
    }

    private func decisionTick() {
        sensors = readSensorsNow()
        let result = decideAction()
        lastResult = result

        ticks += 1
        confidenceSum += result.confidence
        confidenceCount += 1

        if result.fellBack {
            fallbacks += 1
            addLog("low confidence → safety stop", isCollision: false)
        } else if result.decision != .forward {
            addLog("\(result.decision.label)  (conf \(String(format: "%.2f", result.confidence)))", isCollision: false)
        }

        applyDecision(result.decision, dt: decisionInterval)
    }

    private func applyDecision(_ decision: RoverDecision, dt: TimeInterval) {
        switch decision {
        case .forward:
            roverPos.x += cos(roverHeading) * roverSpeed * CGFloat(dt)
            roverPos.y += sin(roverHeading) * roverSpeed * CGFloat(dt)
        case .turnLeft:
            roverHeading -= turnRate * CGFloat(dt)
            roverPos.x += cos(roverHeading) * roverSpeed * 0.35 * CGFloat(dt)
            roverPos.y += sin(roverHeading) * roverSpeed * 0.35 * CGFloat(dt)
        case .turnRight:
            roverHeading += turnRate * CGFloat(dt)
            roverPos.x += cos(roverHeading) * roverSpeed * 0.35 * CGFloat(dt)
            roverPos.y += sin(roverHeading) * roverSpeed * 0.35 * CGFloat(dt)
        case .stop:
            break // no self-propelled movement this tick
        }
    }

    private func addLog(_ text: String, isCollision: Bool) {
        let seconds = Double(ticks) * decisionInterval
        log.insert(LogEntry(text: String(format: "t+%.1fs  %@", seconds, text), isCollision: isCollision), at: 0)
        if log.count > 40 { log.removeLast() }
    }

    func reset() {
        roverPos = CGPoint(x: 36, y: 36)
        roverHeading = 0.35
        windAngle = Double.random(in: 0..<(2 * .pi))
        windSpeed = windMaxSpeed * 0.4
        ticks = 0; collisions = 0; fallbacks = 0; confidenceSum = 0; confidenceCount = 0
        log.removeAll()
        sensors = readSensorsNow()
        lastResult = DecisionResult(decision: .forward, confidence: 0.7, fellBack: false)
        timeSinceLastDecision = 0
    }

    var averageConfidence: Double? { confidenceCount > 0 ? confidenceSum / Double(confidenceCount) : nil }
}
