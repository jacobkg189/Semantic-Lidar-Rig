import ARKit
import Combine
import SwiftUI

@main
struct SemanticScannerApp: App {
    var body: some Scene {
        WindowGroup { ContentView() }
    }
}

/// Deliberately a status readout, not a viewfinder. Phase 0 only needs to answer
/// "is data flowing?" — AR rendering lands in Phase 3, where seeing lidar points
/// land on real walls is the actual calibration check.
struct ContentView: View {
    @StateObject private var model = StreamModel()

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            Text("Semantic Scanner")
                .font(.title2.weight(.semibold))
            Text("Phase 0 — transport")
                .font(.caption)
                .foregroundStyle(.secondary)

            Divider()

            row("Link", model.linkLabel, color: model.linkColor)
            row("Tracking", model.streamer.trackingLabel,
                color: model.streamer.trackingLabel == "normal" ? .green : .orange)
            row("Poses sent", "\(model.streamer.poseCount)")
            if model.streamer.frameCount > 0 {
                row("Frames sent", "\(model.streamer.frameCount)")
            }

            Divider()

            Toggle("Stream camera frames (10 Hz)", isOn: $model.sendFrames)
                .font(.callout)

            Text(model.hint)
                .font(.footnote)
                .foregroundStyle(.secondary)
                .fixedSize(horizontal: false, vertical: true)

            Spacer()
        }
        .padding(24)
        .onAppear { model.start() }
        .onDisappear { model.stop() }
    }

    private func row(_ label: String, _ value: String, color: Color = .primary) -> some View {
        HStack {
            Text(label).foregroundStyle(.secondary)
            Spacer()
            Text(value).foregroundStyle(color).monospacedDigit()
        }
        .font(.callout)
    }
}

final class StreamModel: ObservableObject {
    let server = PhoneServer()
    let streamer: ARStreamer

    @Published var linkLabel = "starting"
    @Published var linkColor: Color = .secondary
    @Published var sendFrames = false {
        didSet { streamer.cameraFrameHz = sendFrames ? 10 : 0 }
    }

    var hint: String {
        switch server.state {
        case .connected:
            return "Streaming. Keep this screen in the foreground — iOS suspends the session if the app is backgrounded or the phone locks."
        case .listening:
            return "Waiting for the Mac. Run:\n  iproxy 5555 5555\n  python3 mac/phase0_check.py"
        case .failed(let msg):
            return "Listener failed: \(msg)"
        case .idle:
            return "Starting up."
        }
    }

    private var cancellable: Any?

    init() {
        streamer = ARStreamer(server: server)
        // Republish the streamer's counters so SwiftUI re-renders on each pose.
        cancellable = streamer.objectWillChange.sink { [weak self] _ in
            self?.objectWillChange.send()
        }
        server.onStateChange = { [weak self] state in
            guard let self else { return }
            switch state {
            case .idle:               self.linkLabel = "idle";                 self.linkColor = .secondary
            case .listening(let p):   self.linkLabel = "listening :\(p)";      self.linkColor = .orange
            case .connected:          self.linkLabel = "connected";            self.linkColor = .green
            case .failed(let m):      self.linkLabel = "failed";               self.linkColor = .red; _ = m
            }
            self.objectWillChange.send()
        }
    }

    func start() {
        server.start()
        streamer.start()
    }

    func stop() {
        streamer.stop()
        server.stop()
    }
}
