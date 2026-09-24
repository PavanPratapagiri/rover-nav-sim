# Rover Sim — iOS (SwiftUI)

Port of the browser version: same rover, same obstacles, same wind model, same
decision function (`decideAction()`), now driven by SwiftUI's `Canvas`
instead of an HTML `<canvas>`.

## Set up in Xcode

1. Open Xcode → **File → New → Project → iOS → App**.
2. Product Name: `RoverSim`. Interface: **SwiftUI**. Language: **Swift**.
3. Once the project is created, delete the auto-generated `ContentView.swift`
   and the `RoverSimApp.swift` Xcode made for you.
4. Drag the three files from this folder — `RoverSimApp.swift`,
   `SimEngine.swift`, `ContentView.swift` — into the project navigator
   (check "Copy items if needed").
5. Build and run (⌘R) on a simulator or device.

No third-party packages needed — everything is Foundation, CoreGraphics, and
SwiftUI.

## Where things live

- **`SimEngine.swift`** — all the logic: sensor ray-casting, wind drift/gusts,
  collision resolution, and `decideAction()`, the decision function.
- **`ContentView.swift`** — the UI: the `Canvas` that draws the arena, and the
  control panel (decision readout, sensors, wind sliders, stats, log).
- **`RoverSimApp.swift`** — the app entry point, generated boilerplate.

## Known differences from the web version

- Canvas coordinates are tuned to a phone-sized arena (340×460pt vs 760×460px)
  so it fits on screen without scrolling sideways; obstacle layout was
  rescaled to match.
- No `localStorage`-equivalent persistence — settings reset on relaunch. If
  you want sliders to persist, back them with `@AppStorage` instead of plain
  `@Published` properties.
