import SwiftUI

private let bg = Color(red: 0.075, green: 0.086, blue: 0.071)
private let panel = Color(red: 0.106, green: 0.129, blue: 0.110)
private let panel2 = Color(red: 0.122, green: 0.149, blue: 0.126)
private let lineColor = Color(red: 0.169, green: 0.2, blue: 0.173)
private let textColor = Color(red: 0.906, green: 0.918, blue: 0.894)
private let mutedColor = Color(red: 0.561, green: 0.627, blue: 0.541)
private let mossColor = Color(red: 0.498, green: 0.749, blue: 0.541)
private let amberColor = Color(red: 0.878, green: 0.643, blue: 0.345)
private let rustColor = Color(red: 0.769, green: 0.349, blue: 0.247)
private let dustColor = Color(red: 0.486, green: 0.612, blue: 0.702)

struct ContentView: View {
    @StateObject private var engine = SimEngine()
    @State private var lastTimestamp: Date = Date()

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                header
                arenaCard
                decisionCard
                sensorCard
                windCard
                statsCard
                noteText
            }
            .padding(16)
        }
        .background(bg.ignoresSafeArea())
        .foregroundColor(textColor)
    }

    // MARK: Header

    private var header: some View {
        VStack(alignment: .leading, spacing: 6) {
            Text("SENSE → DECIDE → ACT")
                .font(.system(.caption, design: .monospaced))
                .foregroundColor(dustColor)
            Text("Rover obstacle avoidance with wind")
                .font(.title2.weight(.semibold))
            Text("Every 150ms it reads three distance sensors, gets a typed decision plus a confidence score, and moves.")
                .font(.footnote)
                .foregroundColor(mutedColor)
        }
    }

    // MARK: Arena

    private var arenaCard: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                statusPill
                Spacer()
                Text("loop 150ms")
                    .font(.system(.caption2, design: .monospaced))
                    .foregroundColor(mutedColor)
            }

            TimelineView(.animation) { timeline in
                Canvas { context, size in
                    draw(context: context, size: size)
                }
                .onChange(of: timeline.date) { _, newDate in
                    let dt = newDate.timeIntervalSince(lastTimestamp)
                    lastTimestamp = newDate
                    engine.update(dt: dt)
                }
            }
            .frame(width: engine.arenaSize.width, height: engine.arenaSize.height)
            .background(Color(red: 0.059, green: 0.075, blue: 0.063))
            .cornerRadius(8)

            HStack(spacing: 10) {
                Button(engine.running ? "Pause" : "Resume") {
                    engine.running.toggle()
                }
                .buttonStyle(PrimaryButtonStyle())

                Button("Reset") { engine.reset() }
                    .buttonStyle(SecondaryButtonStyle())
            }
        }
        .padding(14)
        .background(panel)
        .cornerRadius(10)
        .overlay(RoundedRectangle(cornerRadius: 10).stroke(lineColor, lineWidth: 1))
    }

    private var statusPill: some View {
        HStack(spacing: 6) {
            Circle()
                .fill(engine.running ? mossColor : mutedColor)
                .frame(width: 6, height: 6)
            Text(engine.running ? "running" : "paused")
                .font(.system(.caption2, design: .monospaced))
                .foregroundColor(mutedColor)
        }
        .padding(.horizontal, 9)
        .padding(.vertical, 4)
        .overlay(Capsule().stroke(lineColor, lineWidth: 1))
    }

    private func draw(context: GraphicsContext, size: CGSize) {
        // Grid
        var gridPath = Path()
        var gx: CGFloat = 0
        while gx < size.width { gridPath.move(to: CGPoint(x: gx, y: 0)); gridPath.addLine(to: CGPoint(x: gx, y: size.height)); gx += 38 }
        var gy: CGFloat = 0
        while gy < size.height { gridPath.move(to: CGPoint(x: 0, y: gy)); gridPath.addLine(to: CGPoint(x: size.width, y: gy)); gy += 38 }
        context.stroke(gridPath, with: .color(Color.white.opacity(0.04)), lineWidth: 1)

        // Bounds
        let bounds = CGRect(x: engine.margin, y: engine.margin,
                             width: size.width - engine.margin * 2, height: size.height - engine.margin * 2)
        context.stroke(Path(bounds), with: .color(lineColor), lineWidth: 2)

        // Obstacles
        for o in engine.obstacles {
            context.fill(Path(o.rect), with: .color(panel2))
            context.stroke(Path(o.rect), with: .color(lineColor), lineWidth: 1)
        }

        // Wind field arrows
        var windPath = Path()
        var wy: CGFloat = 40
        while wy < size.height {
            var wx: CGFloat = 40
            while wx < size.width {
                let len = 8 + CGFloat(engine.windSpeed) * 9
                let ex = wx + cos(CGFloat(engine.windAngle)) * len
                let ey = wy + sin(CGFloat(engine.windAngle)) * len
                windPath.move(to: CGPoint(x: wx, y: wy))
                windPath.addLine(to: CGPoint(x: ex, y: ey))
                wx += 95
            }
            wy += 95
        }
        context.stroke(windPath, with: .color(dustColor.opacity(0.35)), lineWidth: 1.2)

        // Sensor rays
        let angles: [CGFloat] = [engine.roverHeading - 0.78, engine.roverHeading, engine.roverHeading + 0.78]
        let dists: [CGFloat] = [engine.sensors.left, engine.sensors.front, engine.sensors.right]
        for i in 0..<3 {
            let d = dists[i]
            var ray = Path()
            ray.move(to: engine.roverPos)
            ray.addLine(to: CGPoint(x: engine.roverPos.x + cos(angles[i]) * d, y: engine.roverPos.y + sin(angles[i]) * d))
            let color = d < engine.danger ? rustColor.opacity(0.55) : mossColor.opacity(0.28)
            context.stroke(ray, with: .color(color), lineWidth: 1)
        }

        // Rover (triangle pointing along heading)
        let r = engine.roverRadius
        let h = engine.roverHeading
        func rotated(_ dx: CGFloat, _ dy: CGFloat) -> CGPoint {
            CGPoint(x: engine.roverPos.x + dx * cos(h) - dy * sin(h),
                    y: engine.roverPos.y + dx * sin(h) + dy * cos(h))
        }
        var roverPath = Path()
        roverPath.move(to: rotated(r + 4, 0))
        roverPath.addLine(to: rotated(-r, r * 0.85))
        roverPath.addLine(to: rotated(-r, -r * 0.85))
        roverPath.closeSubpath()

        let roverColor: Color
        switch engine.lastResult.decision {
        case .stop: roverColor = rustColor
        case .forward: roverColor = mossColor
        case .turnLeft, .turnRight: roverColor = amberColor
        }
        context.fill(roverPath, with: .color(roverColor))
    }

    // MARK: Decision card

    private var decisionCard: some View {
        cardShell {
            blockTitle("CURRENT DECISION")
            Text(engine.lastResult.decision.label)
                .font(.system(.title3, design: .monospaced).weight(.semibold))
                .foregroundColor(decisionColor)

            GeometryReader { geo in
                ZStack(alignment: .leading) {
                    Capsule().fill(lineColor)
                    Capsule().fill(dustColor)
                        .frame(width: geo.size.width * CGFloat(engine.lastResult.confidence))
                }
            }
            .frame(height: 5)

            Text("confidence \(String(format: "%.2f", engine.lastResult.confidence))" + (engine.lastResult.fellBack ? "  ·  fallback stop" : ""))
                .font(.system(.caption2, design: .monospaced))
                .foregroundColor(mutedColor)
        }
    }

    private var decisionColor: Color {
        switch engine.lastResult.decision {
        case .forward: return mossColor
        case .turnLeft, .turnRight: return amberColor
        case .stop: return rustColor
        }
    }

    // MARK: Sensor card

    private var sensorCard: some View {
        cardShell {
            blockTitle("SENSORS (px to obstacle)")
            HStack(spacing: 8) {
                sensorCell("left", engine.sensors.left)
                sensorCell("front", engine.sensors.front)
                sensorCell("right", engine.sensors.right)
            }
        }
    }

    private func sensorCell(_ label: String, _ value: CGFloat) -> some View {
        VStack(spacing: 4) {
            Text(label).font(.caption2).foregroundColor(mutedColor)
            Text("\(Int(value))").font(.system(.body, design: .monospaced))
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 8)
        .background(panel2)
        .cornerRadius(6)
        .overlay(RoundedRectangle(cornerRadius: 6).stroke(lineColor, lineWidth: 1))
    }

    // MARK: Wind card

    private var windCard: some View {
        cardShell {
            blockTitle("WIND")

            Toggle(isOn: $engine.windAware) {
                VStack(alignment: .leading, spacing: 2) {
                    Text("Wind-aware decisions").font(.subheadline)
                    Text("feed wind into the decision model").font(.caption2).foregroundColor(mutedColor)
                }
            }
            .tint(dustColor)

            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("Max wind speed").font(.caption).foregroundColor(mutedColor)
                    Spacer()
                    Text(String(format: "%.1f", engine.windMaxSpeed)).font(.system(.caption, design: .monospaced))
                }
                Slider(value: $engine.windMaxSpeed, in: 0...3, step: 0.1).tint(dustColor)
            }

            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("Gustiness").font(.caption).foregroundColor(mutedColor)
                    Spacer()
                    Text(String(format: "%.2f", engine.windGust)).font(.system(.caption, design: .monospaced))
                }
                Slider(value: $engine.windGust, in: 0...1, step: 0.05).tint(dustColor)
            }

            HStack(spacing: 8) {
                sensorCell("direction", CGFloat((engine.windAngle * 180 / .pi).truncatingRemainder(dividingBy: 360)))
                sensorCellDouble("speed", engine.windSpeed)
            }
        }
    }

    private func sensorCellDouble(_ label: String, _ value: Double) -> some View {
        VStack(spacing: 4) {
            Text(label).font(.caption2).foregroundColor(mutedColor)
            Text(String(format: "%.2f", value)).font(.system(.body, design: .monospaced))
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 8)
        .background(panel2)
        .cornerRadius(6)
        .overlay(RoundedRectangle(cornerRadius: 6).stroke(lineColor, lineWidth: 1))
    }

    // MARK: Stats card

    private var statsCard: some View {
        cardShell {
            blockTitle("SESSION STATS")
            LazyVGrid(columns: [GridItem(.flexible()), GridItem(.flexible())], spacing: 8) {
                statCell("ticks", "\(engine.ticks)")
                statCell("collisions", "\(engine.collisions)")
                statCell("avg confidence", engine.averageConfidence.map { String(format: "%.2f", $0) } ?? "—")
                statCell("fallback stops", "\(engine.fallbacks)")
            }

            Divider().background(lineColor)

            ScrollView {
                VStack(alignment: .leading, spacing: 5) {
                    ForEach(Array(engine.log.prefix(10).enumerated()), id: \.element.id) { index, entry in
                        Text(entry.text)
                            .font(.system(.caption2, design: .monospaced))
                            .foregroundColor(index == 0 ? textColor : mutedColor)
                    }
                }
            }
            .frame(maxHeight: 90)
        }
    }

    private func statCell(_ label: String, _ value: String) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(label).font(.caption2).foregroundColor(mutedColor)
            Text(value).font(.system(.body, design: .monospaced))
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    // MARK: Note

    private var noteText: some View {
        Text("The decision step is a pure function: current state in, one typed decision plus a confidence score out. It lives in decideAction() in SimEngine.swift.")
            .font(.caption2)
            .foregroundColor(mutedColor)
            .padding(.top, 4)
    }

    // MARK: Shared card shell

    @ViewBuilder
    private func cardShell<Content: View>(@ViewBuilder content: () -> Content) -> some View {
        VStack(alignment: .leading, spacing: 10) {
            content()
        }
        .padding(14)
        .background(panel)
        .cornerRadius(10)
        .overlay(RoundedRectangle(cornerRadius: 10).stroke(lineColor, lineWidth: 1))
    }

    private func blockTitle(_ text: String) -> some View {
        Text(text)
            .font(.system(size: 11, weight: .semibold))
            .foregroundColor(mutedColor)
    }
}

// MARK: - Button styles

struct PrimaryButtonStyle: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.subheadline.weight(.medium))
            .padding(.horizontal, 16)
            .padding(.vertical, 9)
            .background(mossColor.opacity(configuration.isPressed ? 0.75 : 1))
            .foregroundColor(bg)
            .cornerRadius(6)
    }
}

struct SecondaryButtonStyle: ButtonStyle {
    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.subheadline.weight(.medium))
            .padding(.horizontal, 16)
            .padding(.vertical, 9)
            .background(panel2.opacity(configuration.isPressed ? 0.7 : 1))
            .foregroundColor(textColor)
            .cornerRadius(6)
            .overlay(RoundedRectangle(cornerRadius: 6).stroke(lineColor, lineWidth: 1))
    }
}

#Preview {
    ContentView()
}
