//
//  HTTPFileServer.swift
//  Tracking Viewer
//
//  Simple HTTP server for serving recordings via curl-accessible URLs
//

import Foundation
import Network
import UIKit
import Combine

/// HTTP File Server for serving recordings over the network
/// Supports individual file download and zip export
@MainActor
class HTTPFileServer: ObservableObject {
    static let shared = HTTPFileServer()
    
    // MARK: - Published Properties
    @Published var isRunning: Bool = false
    @Published var serverURL: String?
    @Published var port: UInt16 = 8080
    @Published var activeDownloads: Int = 0
    @Published var exportProgress: Double = 0
    @Published var isCreatingZip: Bool = false
    @Published var statusMessage: String = ""
    
    // MARK: - Private Properties
    private var listener: NWListener?
    private var connections: [NWConnection] = []
    private let serverQueue = DispatchQueue(label: "com.trackingviewer.httpserver", qos: .userInitiated)
    
    // Temporary zip file management
    private var tempZipURL: URL?
    
    private init() {}
    
    // MARK: - Server Control
    
    func startServer() {
        guard !isRunning else { return }
        
        do {
            let parameters = NWParameters.tcp
            parameters.allowLocalEndpointReuse = true
            
            listener = try NWListener(using: parameters, on: NWEndpoint.Port(rawValue: port)!)
            
            listener?.stateUpdateHandler = { [weak self] state in
                Task { @MainActor in
                    switch state {
                    case .ready:
                        if let port = self?.listener?.port?.rawValue {
                            self?.port = port
                            self?.updateServerURL()
                            self?.isRunning = true
                            print("🌐 [HTTPServer] Started on port \(port)")
                        }
                    case .failed(let error):
                        print("❌ [HTTPServer] Failed: \(error)")
                        self?.isRunning = false
                        self?.serverURL = nil
                    case .cancelled:
                        self?.isRunning = false
                        self?.serverURL = nil
                    default:
                        break
                    }
                }
            }
            
            listener?.newConnectionHandler = { [weak self] connection in
                self?.handleConnection(connection)
            }
            
            listener?.start(queue: serverQueue)
            
        } catch {
            print("❌ [HTTPServer] Failed to start: \(error)")
        }
    }
    
    func stopServer() {
        listener?.cancel()
        listener = nil
        
        for connection in connections {
            connection.cancel()
        }
        connections.removeAll()
        
        // Clean up temp files
        cleanupTempFiles()
        
        isRunning = false
        serverURL = nil
        print("🛑 [HTTPServer] Stopped")
    }
    
    private func updateServerURL() {
        if let ip = getWiFiAddress() {
            serverURL = "http://\(ip):\(port)"
        } else {
            serverURL = "http://localhost:\(port)"
        }
    }
    
    // MARK: - URL Generation
    
    /// Generate a URL for downloading a single recording
    func getDownloadURL(for recording: Recording) -> String? {
        guard isRunning, let base = serverURL else { return nil }
        return "\(base)/download/\(recording.id)"
    }
    
    /// Generate a URL for downloading all recordings as a zip
    func getDownloadAllURL() -> String? {
        guard isRunning, let base = serverURL else { return nil }
        return "\(base)/download-all"
    }
    
    /// Generate a URL for downloading selected recordings as a zip
    func getDownloadSelectedURL(recordingIDs: [String]) -> String? {
        guard isRunning, let base = serverURL else { return nil }
        let ids = recordingIDs.joined(separator: ",")
        return "\(base)/download-selected?ids=\(ids)"
    }
    
    // MARK: - Connection Handling
    
    nonisolated private func handleConnection(_ connection: NWConnection) {
        Task { @MainActor in
            self.connections.append(connection)
        }
        
        connection.stateUpdateHandler = { [weak self] state in
            switch state {
            case .ready:
                self?.receiveRequest(on: connection)
            case .failed, .cancelled:
                Task { @MainActor in
                    self?.connections.removeAll { $0 === connection }
                    if self?.activeDownloads ?? 0 > 0 {
                        self?.activeDownloads -= 1
                    }
                }
            default:
                break
            }
        }
        
        connection.start(queue: serverQueue)
    }
    
    nonisolated private func receiveRequest(on connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] content, _, isComplete, error in
            guard let self = self else { return }
            
            if let data = content, let request = String(data: data, encoding: .utf8) {
                self.handleRequest(request, on: connection)
            } else if let error = error {
                print("❌ [HTTPServer] Receive error: \(error)")
                connection.cancel()
            }
        }
    }
    
    nonisolated private func handleRequest(_ request: String, on connection: NWConnection) {
        // Parse HTTP request
        let lines = request.components(separatedBy: "\r\n")
        guard let firstLine = lines.first else {
            sendError(404, message: "Bad Request", on: connection)
            return
        }
        
        let parts = firstLine.components(separatedBy: " ")
        guard parts.count >= 2 else {
            sendError(400, message: "Bad Request", on: connection)
            return
        }
        
        let method = parts[0]
        let path = parts[1]
        
        print("📥 [HTTPServer] \(method) \(path)")
        
        Task { @MainActor in
            self.activeDownloads += 1
        }
        
        // Route requests
        if path == "/" {
            Task { @MainActor in
                self.sendIndexPage(on: connection)
            }
        } else if path == "/download-all" {
            Task {
                await self.handleDownloadAll(on: connection)
            }
        } else if path.hasPrefix("/download-selected") {
            // Parse query parameters
            let components = URLComponents(string: path)
            let ids = components?.queryItems?.first(where: { $0.name == "ids" })?.value?.components(separatedBy: ",") ?? []
            Task {
                await self.handleDownloadSelected(recordingIDs: ids, on: connection)
            }
        } else if path.hasPrefix("/download/") {
            let recordingID = String(path.dropFirst("/download/".count))
            Task {
                await self.handleDownloadRecording(id: recordingID, on: connection)
            }
        } else if path.hasPrefix("/file/") {
            // Direct file access: /file/{recordingID}/{filename}
            let pathComponents = path.dropFirst("/file/".count).components(separatedBy: "/")
            if pathComponents.count >= 2 {
                let recordingID = pathComponents[0]
                let filename = pathComponents.dropFirst().joined(separator: "/")
                Task {
                    await self.handleFileRequest(recordingID: recordingID, filename: filename, on: connection)
                }
            } else {
                sendError(404, message: "Not Found", on: connection)
            }
        } else {
            sendError(404, message: "Not Found", on: connection)
        }
    }
    
    // MARK: - Request Handlers
    
    private func sendIndexPage(on connection: NWConnection) {
        Task { @MainActor in
            let recordings = RecordingsManager.shared.recordings
            
            var html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Tracking Viewer - Recordings</title>
                <meta name="viewport" content="width=device-width, initial-scale=1">
                <style>
                    body { font-family: -apple-system, BlinkMacSystemFont, sans-serif; margin: 20px; background: #1a1a1a; color: #fff; }
                    h1 { color: #0a84ff; }
                    .recording { background: #2a2a2a; padding: 15px; margin: 10px 0; border-radius: 8px; }
                    .recording h3 { margin: 0 0 10px 0; }
                    .recording .meta { color: #888; font-size: 14px; }
                    a { color: #0a84ff; text-decoration: none; }
                    a:hover { text-decoration: underline; }
                    .btn { display: inline-block; padding: 10px 20px; background: #0a84ff; color: white; border-radius: 6px; margin: 5px 5px 5px 0; }
                    .btn-all { background: #30d158; }
                    code { background: #333; padding: 2px 6px; border-radius: 4px; font-size: 13px; }
                    .curl { background: #333; padding: 10px; border-radius: 6px; margin: 10px 0; overflow-x: auto; }
                </style>
            </head>
            <body>
                <h1>📹 Tracking Viewer Recordings</h1>
                <p>Found <strong>\(recordings.count)</strong> recordings</p>
            """
            
            if !recordings.isEmpty {
                let allURL = self.getDownloadAllURL() ?? ""
                html += """
                <div style="margin: 20px 0;">
                    <a href="/download-all" class="btn btn-all">⬇️ Download All as ZIP</a>
                </div>
                <div class="curl">
                    <code>curl -o recordings.zip "\(allURL)"</code>
                </div>
                <hr style="border-color: #333; margin: 20px 0;">
                """
            }
            
            for recording in recordings {
                let downloadURL = self.getDownloadURL(for: recording) ?? ""
                html += """
                <div class="recording">
                    <h3>\(recording.displayName)</h3>
                    <p class="meta">
                        Duration: \(recording.durationString) |
                        Frames: \(recording.frameCountString) |
                        Size: \(recording.fileSizeString)
                    </p>
                    <a href="/download/\(recording.id)" class="btn">⬇️ Download ZIP</a>
                    <div class="curl">
                        <code>curl -o \(recording.id).zip "\(downloadURL)"</code>
                    </div>
                    <p style="font-size: 12px; color: #666; margin-top: 10px;">
                        Files:
                        <a href="/file/\(recording.id)/video.mp4">video.mp4</a> |
                        <a href="/file/\(recording.id)/metadata.json">metadata.json</a> |
                        <a href="/file/\(recording.id)/tracking.jsonl">tracking.jsonl</a>
                    </p>
                </div>
                """
            }
            
            html += """
            </body>
            </html>
            """
            
            let response = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: \(html.utf8.count)\r\nConnection: close\r\n\r\n\(html)"
            
            self.sendResponse(response.data(using: .utf8)!, on: connection)
        }
    }
    
    private func handleDownloadRecording(id: String, on connection: NWConnection) async {
        await MainActor.run {
            isCreatingZip = true
            exportProgress = 0
        }
        
        defer {
            Task { @MainActor in
                isCreatingZip = false
                exportProgress = 0
                statusMessage = ""
            }
        }
        
        // Find the recording
        let recording = await MainActor.run {
            RecordingsManager.shared.recordings.first { $0.id == id }
        }
        
        guard let recording = recording else {
            sendError(404, message: "Recording not found", on: connection)
            return
        }
        
        // Create zip of the recording folder
        guard let zipData = await createZip(for: [recording]) else {
            sendError(500, message: "Failed to create ZIP", on: connection)
            return
        }
        
        // Send the zip file
        sendFile(data: zipData, filename: "\(id).zip", mimeType: "application/zip", on: connection)
    }
    
    private func handleDownloadAll(on connection: NWConnection) async {
        await MainActor.run {
            isCreatingZip = true
            exportProgress = 0
        }
        
        defer {
            Task { @MainActor in
                isCreatingZip = false
                exportProgress = 0
                statusMessage = ""
            }
        }
        
        let recordings = await MainActor.run {
            RecordingsManager.shared.recordings
        }
        
        guard !recordings.isEmpty else {
            sendError(404, message: "No recordings found", on: connection)
            return
        }
        
        guard let zipData = await createZip(for: Array(recordings)) else {
            sendError(500, message: "Failed to create ZIP", on: connection)
            return
        }
        
        let timestamp = ISO8601DateFormatter().string(from: Date()).replacingOccurrences(of: ":", with: "-")
        sendFile(data: zipData, filename: "all_recordings_\(timestamp).zip", mimeType: "application/zip", on: connection)
    }
    
    private func handleDownloadSelected(recordingIDs: [String], on connection: NWConnection) async {
        await MainActor.run {
            isCreatingZip = true
            exportProgress = 0
        }
        
        defer {
            Task { @MainActor in
                isCreatingZip = false
                exportProgress = 0
                statusMessage = ""
            }
        }
        
        let allRecordings = await MainActor.run {
            RecordingsManager.shared.recordings
        }
        
        let selected = allRecordings.filter { recordingIDs.contains($0.id) }
        
        guard !selected.isEmpty else {
            sendError(404, message: "No matching recordings found", on: connection)
            return
        }
        
        guard let zipData = await createZip(for: selected) else {
            sendError(500, message: "Failed to create ZIP", on: connection)
            return
        }
        
        let timestamp = ISO8601DateFormatter().string(from: Date()).replacingOccurrences(of: ":", with: "-")
        sendFile(data: zipData, filename: "selected_recordings_\(timestamp).zip", mimeType: "application/zip", on: connection)
    }
    
    private func handleFileRequest(recordingID: String, filename: String, on connection: NWConnection) async {
        let recording = await MainActor.run {
            RecordingsManager.shared.recordings.first { $0.id == recordingID }
        }
        
        guard let recording = recording else {
            sendError(404, message: "Recording not found", on: connection)
            return
        }
        
        let fileURL = recording.folderURL.appendingPathComponent(filename)
        
        // Check if file exists
        guard FileManager.default.fileExists(atPath: fileURL.path) else {
            sendError(404, message: "File not found", on: connection)
            return
        }
        
        // Download from iCloud if needed
        let downloaded = await RecordingsManager.shared.downloadFromiCloud(fileURL)
        guard downloaded else {
            sendError(503, message: "File not yet available (downloading from iCloud)", on: connection)
            return
        }
        
        // Read file data
        guard let data = try? Data(contentsOf: fileURL) else {
            sendError(500, message: "Failed to read file", on: connection)
            return
        }
        
        // Determine MIME type
        let mimeType: String
        switch fileURL.pathExtension.lowercased() {
        case "mp4": mimeType = "video/mp4"
        case "json": mimeType = "application/json"
        case "jsonl": mimeType = "application/x-jsonlines"
        default: mimeType = "application/octet-stream"
        }
        
        sendFile(data: data, filename: filename, mimeType: mimeType, on: connection)
    }
    
    // MARK: - ZIP Creation (File-based for large files)
    
    private func createZip(for recordings: [Recording]) async -> Data? {
        let fileManager = FileManager.default
        let tempDir = fileManager.temporaryDirectory
        let zipURL = tempDir.appendingPathComponent("\(UUID().uuidString).zip")
        
        // Clean up any existing file
        try? fileManager.removeItem(at: zipURL)
        
        do {
            // Create file handle for writing
            fileManager.createFile(atPath: zipURL.path, contents: nil)
            let fileHandle = try FileHandle(forWritingTo: zipURL)
            
            defer {
                try? fileHandle.close()
            }
            
            var centralDirectory = Data()
            var fileCount: UInt16 = 0
            var currentOffset: UInt64 = 0
            
            let totalRecordings = Double(recordings.count)
            
            for (recordingIndex, recording) in recordings.enumerated() {
                await MainActor.run {
                    exportProgress = Double(recordingIndex) / totalRecordings * 0.95
                }
                
                let progressMsg = "Processing \(recordingIndex + 1)/\(recordings.count): \(recording.id)"
                print("📦 [HTTPServer] \(progressMsg)")
                await MainActor.run {
                    statusMessage = progressMsg
                }
                
                // Get files in this recording folder
                let folderContents: [URL]
                do {
                    folderContents = try fileManager.contentsOfDirectory(at: recording.folderURL, includingPropertiesForKeys: [.isRegularFileKey, .fileSizeKey], options: [.skipsHiddenFiles])
                } catch {
                    print("⚠️ [HTTPServer] Failed to list \(recording.id): \(error)")
                    continue
                }
                
                for fileURL in folderContents {
                    // Skip directories
                    guard let resourceValues = try? fileURL.resourceValues(forKeys: [.isRegularFileKey, .fileSizeKey]),
                          resourceValues.isRegularFile == true else {
                        continue
                    }
                    
                    let fileSize = resourceValues.fileSize ?? 0
                    
                    // Download from iCloud if needed
                    let downloaded = await RecordingsManager.shared.downloadFromiCloud(fileURL)
                    guard downloaded else {
                        print("⚠️ [HTTPServer] Skipping \(fileURL.lastPathComponent) - not downloaded")
                        continue
                    }
                    
                    // Create relative path: recordingID/filename
                    let relativePath = "\(recording.id)/\(fileURL.lastPathComponent)"
                    
                    // Write local file header (with CRC=0, use data descriptor)
                    let localHeader = createZipLocalHeaderStreaming(filename: relativePath, fileSize: UInt32(fileSize))
                    try fileHandle.write(contentsOf: localHeader)
                    
                    // Stream file data in chunks
                    let sourceHandle = try FileHandle(forReadingFrom: fileURL)
                    defer { try? sourceHandle.close() }
                    
                    var crc: UInt32 = 0xFFFFFFFF
                    var bytesWritten: UInt32 = 0
                    let chunkSize = 1024 * 1024 // 1MB chunks
                    
                    while true {
                        // Read chunk inside autoreleasepool to free memory
                        var chunk: Data?
                        autoreleasepool {
                            chunk = sourceHandle.readData(ofLength: chunkSize)
                        }
                        
                        guard let data = chunk, !data.isEmpty else {
                            break // EOF reached
                        }
                        
                        // Update CRC incrementally
                        for byte in data {
                            crc ^= UInt32(byte)
                            for _ in 0..<8 {
                                if crc & 1 != 0 {
                                    crc = (crc >> 1) ^ 0xEDB88320
                                } else {
                                    crc >>= 1
                                }
                            }
                        }
                        
                        try fileHandle.write(contentsOf: data)
                        bytesWritten += UInt32(data.count)
                    }
                    
                    crc = ~crc
                    
                    // Write data descriptor
                    let dataDescriptor = createDataDescriptor(crc: crc, size: bytesWritten)
                    try fileHandle.write(contentsOf: dataDescriptor)
                    
                    // Create central directory entry
                    let centralHeader = createZipCentralHeaderStreaming(filename: relativePath, crc: crc, fileSize: bytesWritten, localHeaderOffset: UInt32(currentOffset))
                    centralDirectory.append(centralHeader)
                    
                    currentOffset += UInt64(localHeader.count) + UInt64(bytesWritten) + UInt64(dataDescriptor.count)
                    fileCount += 1
                }
            }
            
            await MainActor.run {
                exportProgress = 0.98
            }
            
            // Write central directory
            try fileHandle.write(contentsOf: centralDirectory)
            
            // Write end of central directory
            let endRecord = createZipEndRecord(fileCount: fileCount, centralDirectorySize: UInt32(centralDirectory.count), centralDirectoryOffset: UInt32(currentOffset))
            try fileHandle.write(contentsOf: endRecord)
            
            try fileHandle.close()
            
            await MainActor.run {
                exportProgress = 1.0
            }
            
            // Read the complete ZIP file
            let zipData = try Data(contentsOf: zipURL)
            
            // Clean up temp file
            try? fileManager.removeItem(at: zipURL)
            
            print("✅ [HTTPServer] Created ZIP with \(fileCount) files, \(ByteCountFormatter.string(fromByteCount: Int64(zipData.count), countStyle: .file))")
            
            return zipData
            
        } catch {
            print("❌ [HTTPServer] Failed to create ZIP: \(error)")
            try? fileManager.removeItem(at: zipURL)
            return nil
        }
    }
    
    /// Local file header for streaming (uses data descriptor for CRC)
    private func createZipLocalHeaderStreaming(filename: String, fileSize: UInt32) -> Data {
        var header = Data()
        let filenameData = filename.data(using: .utf8)!
        
        // Local file header signature
        header.append(contentsOf: [0x50, 0x4b, 0x03, 0x04])
        // Version needed (2.0)
        header.append(contentsOf: [0x14, 0x00])
        // General purpose bit flag (bit 3 = use data descriptor)
        header.append(contentsOf: [0x08, 0x00])
        // Compression method (0 = stored)
        header.append(contentsOf: [0x00, 0x00])
        // Last mod time/date
        header.append(contentsOf: [0x00, 0x00, 0x00, 0x00])
        // CRC-32 (0, will be in data descriptor)
        header.append(contentsOf: [0x00, 0x00, 0x00, 0x00])
        // Compressed size (0, will be in data descriptor)
        header.append(contentsOf: [0x00, 0x00, 0x00, 0x00])
        // Uncompressed size (0, will be in data descriptor)
        header.append(contentsOf: [0x00, 0x00, 0x00, 0x00])
        // Filename length
        header.append(contentsOf: withUnsafeBytes(of: UInt16(filenameData.count).littleEndian) { Array($0) })
        // Extra field length
        header.append(contentsOf: [0x00, 0x00])
        // Filename
        header.append(filenameData)
        
        return header
    }
    
    /// Data descriptor (follows file data when using streaming)
    private func createDataDescriptor(crc: UInt32, size: UInt32) -> Data {
        var descriptor = Data()
        // Optional signature
        descriptor.append(contentsOf: [0x50, 0x4b, 0x07, 0x08])
        // CRC-32
        descriptor.append(contentsOf: withUnsafeBytes(of: crc.littleEndian) { Array($0) })
        // Compressed size
        descriptor.append(contentsOf: withUnsafeBytes(of: size.littleEndian) { Array($0) })
        // Uncompressed size
        descriptor.append(contentsOf: withUnsafeBytes(of: size.littleEndian) { Array($0) })
        return descriptor
    }
    
    /// Central directory header for streaming mode
    private func createZipCentralHeaderStreaming(filename: String, crc: UInt32, fileSize: UInt32, localHeaderOffset: UInt32) -> Data {
        var header = Data()
        let filenameData = filename.data(using: .utf8)!
        
        // Central file header signature
        header.append(contentsOf: [0x50, 0x4b, 0x01, 0x02])
        // Version made by
        header.append(contentsOf: [0x14, 0x03])
        // Version needed
        header.append(contentsOf: [0x14, 0x00])
        // General purpose bit flag (bit 3 = data descriptor used)
        header.append(contentsOf: [0x08, 0x00])
        // Compression method
        header.append(contentsOf: [0x00, 0x00])
        // Last mod time/date
        header.append(contentsOf: [0x00, 0x00, 0x00, 0x00])
        // CRC-32
        header.append(contentsOf: withUnsafeBytes(of: crc.littleEndian) { Array($0) })
        // Compressed size
        header.append(contentsOf: withUnsafeBytes(of: fileSize.littleEndian) { Array($0) })
        // Uncompressed size
        header.append(contentsOf: withUnsafeBytes(of: fileSize.littleEndian) { Array($0) })
        // Filename length
        header.append(contentsOf: withUnsafeBytes(of: UInt16(filenameData.count).littleEndian) { Array($0) })
        // Extra field length
        header.append(contentsOf: [0x00, 0x00])
        // Comment length
        header.append(contentsOf: [0x00, 0x00])
        // Disk number start
        header.append(contentsOf: [0x00, 0x00])
        // Internal file attributes
        header.append(contentsOf: [0x00, 0x00])
        // External file attributes
        header.append(contentsOf: [0x00, 0x00, 0x00, 0x00])
        // Relative offset of local header
        header.append(contentsOf: withUnsafeBytes(of: localHeaderOffset.littleEndian) { Array($0) })
        // Filename
        header.append(filenameData)
        
        return header
    }
    
    private func createZipEndRecord(fileCount: UInt16, centralDirectorySize: UInt32, centralDirectoryOffset: UInt32) -> Data {
        var record = Data()
        
        // End of central directory signature
        record.append(contentsOf: [0x50, 0x4b, 0x05, 0x06])
        // Number of this disk
        record.append(contentsOf: [0x00, 0x00])
        // Disk where central directory starts
        record.append(contentsOf: [0x00, 0x00])
        // Number of central directory records on this disk
        record.append(contentsOf: withUnsafeBytes(of: fileCount.littleEndian) { Array($0) })
        // Total number of central directory records
        record.append(contentsOf: withUnsafeBytes(of: fileCount.littleEndian) { Array($0) })
        // Size of central directory
        record.append(contentsOf: withUnsafeBytes(of: centralDirectorySize.littleEndian) { Array($0) })
        // Offset of start of central directory
        record.append(contentsOf: withUnsafeBytes(of: centralDirectoryOffset.littleEndian) { Array($0) })
        // Comment length
        record.append(contentsOf: [0x00, 0x00])
        
        return record
    }
    
    /// Simple CRC32 implementation
    private func crc32(_ data: Data) -> UInt32 {
        var crc: UInt32 = 0xFFFFFFFF
        
        for byte in data {
            crc ^= UInt32(byte)
            for _ in 0..<8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0xEDB88320
                } else {
                    crc >>= 1
                }
            }
        }
        
        return ~crc
    }
    
    // MARK: - Response Helpers
    
    private func sendFile(data: Data, filename: String, mimeType: String, on connection: NWConnection) {
        var headers = "HTTP/1.1 200 OK\r\n"
        headers += "Content-Type: \(mimeType)\r\n"
        headers += "Content-Length: \(data.count)\r\n"
        headers += "Content-Disposition: attachment; filename=\"\(filename)\"\r\n"
        headers += "Connection: close\r\n"
        headers += "\r\n"
        
        var response = headers.data(using: .utf8)!
        response.append(data)
        
        sendResponse(response, on: connection)
    }
    
    nonisolated private func sendError(_ code: Int, message: String, on connection: NWConnection) {
        let body = """
        <!DOCTYPE html>
        <html>
        <head><title>\(code) \(message)</title></head>
        <body>
            <h1>\(code) \(message)</h1>
        </body>
        </html>
        """
        
        let response = "HTTP/1.1 \(code) \(message)\r\nContent-Type: text/html\r\nContent-Length: \(body.utf8.count)\r\nConnection: close\r\n\r\n\(body)"
        
        sendResponse(response.data(using: .utf8)!, on: connection)
    }
    
    nonisolated private func sendResponse(_ data: Data, on connection: NWConnection) {
        connection.send(content: data, completion: .contentProcessed { [weak self] error in
            if let error = error {
                print("❌ [HTTPServer] Send error: \(error)")
            }
            connection.cancel()
            
            Task { @MainActor in
                self?.connections.removeAll { $0 === connection }
                if self?.activeDownloads ?? 0 > 0 {
                    self?.activeDownloads -= 1
                }
            }
        })
    }
    
    private func cleanupTempFiles() {
        if let url = tempZipURL {
            try? FileManager.default.removeItem(at: url)
            tempZipURL = nil
        }
    }
    
    // MARK: - Network Utilities
    
    private func getWiFiAddress() -> String? {
        var address: String?
        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        
        guard getifaddrs(&ifaddr) == 0 else { return nil }
        guard let firstAddr = ifaddr else { return nil }
        
        for ifptr in sequence(first: firstAddr, next: { $0.pointee.ifa_next }) {
            let interface = ifptr.pointee
            let addrFamily = interface.ifa_addr.pointee.sa_family
            
            if addrFamily == UInt8(AF_INET) {
                let name = String(cString: interface.ifa_name)
                if name == "en0" {
                    var hostname = [CChar](repeating: 0, count: Int(NI_MAXHOST))
                    getnameinfo(interface.ifa_addr, socklen_t(interface.ifa_addr.pointee.sa_len),
                               &hostname, socklen_t(hostname.count),
                               nil, socklen_t(0), NI_NUMERICHOST)
                    address = String(cString: hostname)
                }
            }
        }
        
        freeifaddrs(ifaddr)
        return address
    }
}
