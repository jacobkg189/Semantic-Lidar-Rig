import Foundation
import Network

/// TCP listener the Mac dials through usbmuxd.
///
/// The phone is the *server* here, which is backwards from how you'd normally
/// set this up. It's forced by usbmuxd: `iproxy 5555 5555` maps a Mac-local
/// port onto a device port, so traffic only flows Mac → phone at connect time.
final class PhoneServer {
    enum State: Equatable {
        case idle
        case listening(port: UInt16)
        case connected
        case failed(String)
    }

    private let port: NWEndpoint.Port
    private var listener: NWListener?
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "phone-server")

    /// Sends are fire-and-forget, but a backed-up socket must not grow an
    /// unbounded queue. Past this many unacknowledged sends we drop camera
    /// frames — poses are small and always get through.
    private var inFlight = 0
    private let maxInFlight = 8

    var onStateChange: ((State) -> Void)?
    private(set) var state: State = .idle {
        didSet { if state != oldValue { DispatchQueue.main.async { self.onStateChange?(self.state) } } }
    }

    private(set) var bytesSent: Int = 0
    private(set) var messagesSent: Int = 0
    private(set) var framesDropped: Int = 0

    init(port: UInt16 = 5555) {
        self.port = NWEndpoint.Port(rawValue: port)!
    }

    func start() {
        do {
            let params = NWParameters.tcp
            params.allowLocalEndpointReuse = true
            // Latency over throughput: poses are 53 bytes and Nagle would
            // coalesce them into unhelpful bursts.
            if let tcp = params.defaultProtocolStack.internetProtocol as? NWProtocolTCP.Options {
                tcp.noDelay = true
            }
            let listener = try NWListener(using: params, on: port)
            listener.newConnectionHandler = { [weak self] conn in self?.accept(conn) }
            listener.stateUpdateHandler = { [weak self] st in
                guard let self else { return }
                switch st {
                case .ready:  self.state = .listening(port: self.port.rawValue)
                case .failed(let e): self.state = .failed(e.localizedDescription)
                default: break
                }
            }
            listener.start(queue: queue)
            self.listener = listener
        } catch {
            state = .failed(error.localizedDescription)
        }
    }

    func stop() {
        connection?.cancel()
        listener?.cancel()
        connection = nil
        listener = nil
        state = .idle
    }

    private func accept(_ conn: NWConnection) {
        // One client at a time. A second connection replaces the first rather
        // than being refused, so a crashed Mac-side run can just reconnect.
        connection?.cancel()
        connection = conn
        inFlight = 0

        // Identity-check every callback against the current connection. The
        // connection we just cancelled reports `.cancelled` asynchronously, and
        // without this guard that stale callback lands *after* the replacement
        // reports `.ready` and knocks state back to .listening — at which point
        // `send()`'s guard silently drops everything, HELLO included. Presents
        // as a connection that opens and then sends zero bytes.
        conn.stateUpdateHandler = { [weak self, weak conn] st in
            guard let self, let conn, self.connection === conn else { return }
            switch st {
            case .ready:
                self.state = .connected
                self.sendHello()
            case .failed, .cancelled:
                self.connection = nil
                self.state = .listening(port: self.port.rawValue)
            default: break
            }
        }
        conn.start(queue: queue)
    }

    private func sendHello() {
        let caps = DeviceCapabilities.current()
        send(WireFormat.hello(
            deviceName: DeviceCapabilities.modelIdentifier(),
            osVersion: UIDeviceOSVersion(),
            capabilities: caps
        ), droppable: false)
    }

    /// - Parameter droppable: camera frames may be dropped under backpressure;
    ///   poses may not.
    func send(_ data: Data, droppable: Bool = false) {
        guard let conn = connection, case .connected = state else { return }

        if droppable && inFlight >= maxInFlight {
            framesDropped += 1
            return
        }

        inFlight += 1
        conn.send(content: data, completion: .contentProcessed { [weak self] error in
            guard let self else { return }
            self.inFlight -= 1
            if error == nil {
                self.bytesSent += data.count
                self.messagesSent += 1
            }
        })
    }
}

private func UIDeviceOSVersion() -> String {
    let v = ProcessInfo.processInfo.operatingSystemVersion
    return "\(v.majorVersion).\(v.minorVersion).\(v.patchVersion)"
}
