//
//  PlaylistViews.swift
//  Tracking Viewer
//
//  Created on 12/02/25.
//

import SwiftUI

// MARK: - Playlists List View

/// Main view showing all playlists
struct PlaylistsListView: View {
    @StateObject private var playlistManager = PlaylistManager.shared
    @StateObject private var recordingsManager = RecordingsManager.shared
    @State private var showCreateSheet = false
    @State private var playlistToEdit: Playlist?
    @State private var showDeleteConfirmation = false
    @State private var playlistToDelete: Playlist?
    
    var body: some View {
        List {
            if playlistManager.playlists.isEmpty {
                emptyStateView
            } else {
                ForEach(playlistManager.playlists) { playlist in
                    NavigationLink(destination: PlaylistDetailView(playlist: playlist)) {
                        PlaylistRow(playlist: playlist)
                    }
                    .swipeActions(edge: .trailing, allowsFullSwipe: false) {
                        Button(role: .destructive) {
                            playlistToDelete = playlist
                            showDeleteConfirmation = true
                        } label: {
                            Label("Delete", systemImage: "trash")
                        }
                        
                        Button {
                            playlistToEdit = playlist
                        } label: {
                            Label("Edit", systemImage: "pencil")
                        }
                        .tint(.orange)
                    }
                    .contextMenu {
                        Button {
                            playlistToEdit = playlist
                        } label: {
                            Label("Edit Playlist", systemImage: "pencil")
                        }
                        
                        Button(role: .destructive) {
                            playlistToDelete = playlist
                            showDeleteConfirmation = true
                        } label: {
                            Label("Delete Playlist", systemImage: "trash")
                        }
                    }
                }
            }
        }
        .navigationTitle("Playlists")
        .toolbar {
            ToolbarItem(placement: .topBarTrailing) {
                Button {
                    showCreateSheet = true
                } label: {
                    Image(systemName: "plus")
                }
            }
        }
        .sheet(isPresented: $showCreateSheet) {
            PlaylistEditSheet(mode: .create)
        }
        .sheet(item: $playlistToEdit) { playlist in
            PlaylistEditSheet(mode: .edit(playlist))
        }
        .alert("Delete Playlist?", isPresented: $showDeleteConfirmation) {
            Button("Cancel", role: .cancel) {}
            Button("Delete", role: .destructive) {
                if let playlist = playlistToDelete {
                    playlistManager.deletePlaylist(playlist)
                }
            }
        } message: {
            Text("This will delete the playlist but not the recordings inside it.")
        }
    }
    
    private var emptyStateView: some View {
        VStack(spacing: 16) {
            Image(systemName: "folder.badge.plus")
                .font(.system(size: 50))
                .foregroundColor(.gray)
            
            Text("No Playlists")
                .font(.title3)
                .fontWeight(.semibold)
            
            Text("Create playlists to organize your recordings")
                .font(.subheadline)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
            
            Button {
                showCreateSheet = true
            } label: {
                Label("Create Playlist", systemImage: "plus")
                    .padding(.horizontal, 20)
                    .padding(.vertical, 10)
            }
            .buttonStyle(.bordered)
        }
        .frame(maxWidth: .infinity)
        .listRowBackground(Color.clear)
    }
}

// MARK: - Playlist Row

struct PlaylistRow: View {
    let playlist: Playlist
    @StateObject private var recordingsManager = RecordingsManager.shared
    
    private var recordingCount: Int {
        playlist.recordingIDs.count
    }
    
    private var thumbnail: Recording? {
        // Get first valid recording as thumbnail
        playlist.recordingIDs.compactMap { id in
            recordingsManager.recordings.first { $0.id == id }
        }.first
    }
    
    var body: some View {
        HStack(spacing: 12) {
            // Icon with color
            ZStack {
                RoundedRectangle(cornerRadius: 8)
                    .fill(playlist.color.color.opacity(0.2))
                    .frame(width: 50, height: 50)
                
                Image(systemName: playlist.icon.systemName)
                    .font(.title2)
                    .foregroundColor(playlist.color.color)
            }
            
            VStack(alignment: .leading, spacing: 4) {
                Text(playlist.name)
                    .font(.headline)
                
                HStack {
                    Text("\(recordingCount) recording\(recordingCount == 1 ? "" : "s")")
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                    
                    if !playlist.description.isEmpty {
                        Text("·")
                            .foregroundColor(.secondary)
                        Text(playlist.description)
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                            .lineLimit(1)
                    }
                }
            }
            
            Spacer()
        }
        .padding(.vertical, 4)
    }
}

// MARK: - Playlist Detail View

struct PlaylistDetailView: View {
    let playlist: Playlist
    
    @StateObject private var playlistManager = PlaylistManager.shared
    @StateObject private var recordingsManager = RecordingsManager.shared
    @StateObject private var httpServer = HTTPFileServer.shared
    @State private var showAddRecordingsSheet = false
    @State private var showShareSheet = false
    @State private var exportURLs: [URL] = []
    @State private var showCopiedToast = false
    @State private var isEditMode: EditMode = .inactive
    
    private var recordings: [Recording] {
        playlistManager.recordings(for: playlist)
    }
    
    // Get the latest version of the playlist
    private var currentPlaylist: Playlist {
        playlistManager.playlists.first { $0.id == playlist.id } ?? playlist
    }
    
    var body: some View {
        List {
            // Playlist header
            Section {
                PlaylistHeaderView(playlist: currentPlaylist)
            }
            
            // HTTP Download section (if server is running)
            if httpServer.isRunning && !currentPlaylist.recordingIDs.isEmpty {
                Section("Download") {
                    httpDownloadSection
                }
            }
            
            // Recordings section
            Section {
                if recordings.isEmpty {
                    emptyRecordingsView
                } else {
                    ForEach(recordings) { recording in
                        NavigationLink(destination: RecordingDetailView(recording: recording)) {
                            PlaylistRecordingRow(recording: recording)
                        }
                    }
                    .onDelete(perform: deleteRecordings)
                    .onMove(perform: moveRecordings)
                }
            } header: {
                HStack {
                    Text("Recordings (\(recordings.count))")
                    Spacer()
                    Button {
                        showAddRecordingsSheet = true
                    } label: {
                        Image(systemName: "plus")
                    }
                }
            }
        }
        .navigationTitle(currentPlaylist.name)
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .topBarTrailing) {
                Menu {
                    Button {
                        showAddRecordingsSheet = true
                    } label: {
                        Label("Add Recordings", systemImage: "plus")
                    }
                    
                    if !recordings.isEmpty {
                        Button {
                            Task {
                                exportURLs = recordings.map { $0.folderURL }
                                showShareSheet = true
                            }
                        } label: {
                            Label("Export All", systemImage: "square.and.arrow.up")
                        }
                    }
                    
                    Divider()
                    
                    EditButton()
                } label: {
                    Image(systemName: "ellipsis.circle")
                }
            }
        }
        .environment(\.editMode, $isEditMode)
        .sheet(isPresented: $showAddRecordingsSheet) {
            AddToPlaylistSheet(playlist: currentPlaylist)
        }
        .sheet(isPresented: $showShareSheet) {
            ShareSheet(items: exportURLs)
        }
        .overlay(alignment: .bottom) {
            if showCopiedToast {
                ToastView(message: "Copied to clipboard")
                    .transition(.move(edge: .bottom).combined(with: .opacity))
            }
        }
        .animation(.easeInOut, value: showCopiedToast)
    }
    
    private var emptyRecordingsView: some View {
        VStack(spacing: 12) {
            Image(systemName: "video.badge.plus")
                .font(.system(size: 40))
                .foregroundColor(.gray)
            
            Text("No recordings in this playlist")
                .font(.subheadline)
                .foregroundColor(.secondary)
            
            Button {
                showAddRecordingsSheet = true
            } label: {
                Text("Add Recordings")
            }
            .buttonStyle(.bordered)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 20)
        .listRowBackground(Color.clear)
    }
    
    private var httpDownloadSection: some View {
        VStack(alignment: .leading, spacing: 8) {
            if let url = httpServer.getPlaylistDownloadURL(for: currentPlaylist) {
                HStack {
                    Text(url)
                        .font(.system(.caption, design: .monospaced))
                        .foregroundColor(.secondary)
                        .lineLimit(1)
                        .truncationMode(.middle)
                    
                    Spacer()
                    
                    Button {
                        UIPasteboard.general.string = url
                        showToast()
                    } label: {
                        Image(systemName: "doc.on.doc")
                    }
                }
                
                Button {
                    let curl = "curl -o \"\(currentPlaylist.name.replacingOccurrences(of: " ", with: "_")).zip\" \"\(url)\""
                    UIPasteboard.general.string = curl
                    showToast()
                } label: {
                    Label("Copy curl Command", systemImage: "terminal")
                        .font(.subheadline)
                        .frame(maxWidth: .infinity)
                }
                .buttonStyle(.bordered)
                .tint(.orange)
            }
        }
    }
    
    private func deleteRecordings(at offsets: IndexSet) {
        let recordingsToRemove = offsets.map { recordings[$0] }
        for recording in recordingsToRemove {
            playlistManager.removeRecording(recording, from: currentPlaylist)
        }
    }
    
    private func moveRecordings(from source: IndexSet, to destination: Int) {
        playlistManager.moveRecording(in: currentPlaylist, from: source, to: destination)
    }
    
    private func showToast() {
        showCopiedToast = true
        DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
            showCopiedToast = false
        }
    }
}

// MARK: - Playlist Header View

struct PlaylistHeaderView: View {
    let playlist: Playlist
    
    private var dateFormatter: DateFormatter {
        let f = DateFormatter()
        f.dateStyle = .medium
        f.timeStyle = .short
        return f
    }
    
    var body: some View {
        VStack(spacing: 16) {
            // Icon
            ZStack {
                Circle()
                    .fill(playlist.color.color.opacity(0.2))
                    .frame(width: 80, height: 80)
                
                Image(systemName: playlist.icon.systemName)
                    .font(.system(size: 36))
                    .foregroundColor(playlist.color.color)
            }
            
            // Name and description
            VStack(spacing: 4) {
                Text(playlist.name)
                    .font(.title2)
                    .fontWeight(.bold)
                
                if !playlist.description.isEmpty {
                    Text(playlist.description)
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                        .multilineTextAlignment(.center)
                }
            }
            
            // Stats
            HStack(spacing: 20) {
                VStack {
                    Text("\(playlist.recordingCount)")
                        .font(.headline)
                    Text("Recordings")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                
                Divider()
                    .frame(height: 30)
                
                VStack {
                    Text(dateFormatter.string(from: playlist.createdAt))
                        .font(.caption)
                    Text("Created")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
            }
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 8)
        .listRowBackground(Color.clear)
    }
}

// MARK: - Playlist Recording Row

struct PlaylistRecordingRow: View {
    let recording: Recording
    @StateObject private var recordingsManager = RecordingsManager.shared
    @State private var thumbnail: UIImage?
    
    var body: some View {
        HStack(spacing: 12) {
            // Thumbnail
            Group {
                if let thumbnail = thumbnail {
                    Image(uiImage: thumbnail)
                        .resizable()
                        .aspectRatio(contentMode: .fill)
                } else {
                    Rectangle()
                        .fill(Color.gray.opacity(0.3))
                        .overlay {
                            Image(systemName: "video")
                                .foregroundColor(.gray)
                        }
                }
            }
            .frame(width: 60, height: 40)
            .cornerRadius(6)
            .clipped()
            
            VStack(alignment: .leading, spacing: 2) {
                Text(recording.displayName)
                    .font(.subheadline)
                    .fontWeight(.medium)
                
                Text("\(recording.durationString) · \(recording.frameCountString)")
                    .font(.caption)
                    .foregroundColor(.secondary)
            }
        }
        .task {
            thumbnail = await recordingsManager.getThumbnail(for: recording)
        }
    }
}

// MARK: - Playlist Edit Sheet

struct PlaylistEditSheet: View {
    enum Mode {
        case create
        case edit(Playlist)
    }
    
    let mode: Mode
    
    @StateObject private var playlistManager = PlaylistManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var name: String = ""
    @State private var description: String = ""
    @State private var selectedColor: PlaylistColor = .blue
    @State private var selectedIcon: PlaylistIcon = .folder
    
    private var isEditing: Bool {
        if case .edit = mode { return true }
        return false
    }
    
    private var isValid: Bool {
        !name.trimmingCharacters(in: .whitespaces).isEmpty
    }
    
    var body: some View {
        NavigationStack {
            Form {
                Section("Name") {
                    TextField("Playlist Name", text: $name)
                }
                
                Section("Description (Optional)") {
                    TextField("Description", text: $description, axis: .vertical)
                        .lineLimit(2...4)
                }
                
                Section("Color") {
                    LazyVGrid(columns: Array(repeating: GridItem(.flexible()), count: 6), spacing: 12) {
                        ForEach(PlaylistColor.allCases, id: \.self) { color in
                            Circle()
                                .fill(color.color)
                                .frame(width: 36, height: 36)
                                .overlay {
                                    if selectedColor == color {
                                        Image(systemName: "checkmark")
                                            .font(.caption.bold())
                                            .foregroundColor(.white)
                                    }
                                }
                                .onTapGesture {
                                    selectedColor = color
                                }
                        }
                    }
                    .padding(.vertical, 8)
                }
                
                Section("Icon") {
                    LazyVGrid(columns: Array(repeating: GridItem(.flexible()), count: 6), spacing: 12) {
                        ForEach(PlaylistIcon.allCases, id: \.self) { icon in
                            ZStack {
                                RoundedRectangle(cornerRadius: 8)
                                    .fill(selectedIcon == icon ? selectedColor.color.opacity(0.2) : Color.gray.opacity(0.1))
                                    .frame(width: 44, height: 44)
                                
                                Image(systemName: icon.systemName)
                                    .font(.title3)
                                    .foregroundColor(selectedIcon == icon ? selectedColor.color : .gray)
                            }
                            .onTapGesture {
                                selectedIcon = icon
                            }
                        }
                    }
                    .padding(.vertical, 8)
                }
                
                // Preview
                Section("Preview") {
                    HStack(spacing: 12) {
                        ZStack {
                            RoundedRectangle(cornerRadius: 8)
                                .fill(selectedColor.color.opacity(0.2))
                                .frame(width: 50, height: 50)
                            
                            Image(systemName: selectedIcon.systemName)
                                .font(.title2)
                                .foregroundColor(selectedColor.color)
                        }
                        
                        VStack(alignment: .leading) {
                            Text(name.isEmpty ? "Playlist Name" : name)
                                .font(.headline)
                                .foregroundColor(name.isEmpty ? .gray : .primary)
                            
                            Text(description.isEmpty ? "No description" : description)
                                .font(.subheadline)
                                .foregroundColor(.secondary)
                                .lineLimit(1)
                        }
                    }
                }
            }
            .navigationTitle(isEditing ? "Edit Playlist" : "New Playlist")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .confirmationAction) {
                    Button(isEditing ? "Save" : "Create") {
                        save()
                        dismiss()
                    }
                    .disabled(!isValid)
                }
            }
            .onAppear {
                if case .edit(let playlist) = mode {
                    name = playlist.name
                    description = playlist.description
                    selectedColor = playlist.color
                    selectedIcon = playlist.icon
                }
            }
        }
    }
    
    private func save() {
        let trimmedName = name.trimmingCharacters(in: .whitespaces)
        let trimmedDescription = description.trimmingCharacters(in: .whitespaces)
        
        switch mode {
        case .create:
            _ = playlistManager.createPlaylist(
                name: trimmedName,
                description: trimmedDescription,
                color: selectedColor,
                icon: selectedIcon
            )
        case .edit(var playlist):
            playlist.name = trimmedName
            playlist.description = trimmedDescription
            playlist.color = selectedColor
            playlist.icon = selectedIcon
            playlistManager.updatePlaylist(playlist)
        }
    }
}

// MARK: - Add To Playlist Sheet

/// Sheet for adding recordings to a specific playlist
struct AddToPlaylistSheet: View {
    let playlist: Playlist
    
    @StateObject private var recordingsManager = RecordingsManager.shared
    @StateObject private var playlistManager = PlaylistManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var selectedRecordings: Set<Recording> = []
    
    // Recordings not already in the playlist
    private var availableRecordings: [Recording] {
        recordingsManager.recordings.filter { !playlist.contains($0) }
    }
    
    var body: some View {
        NavigationStack {
            Group {
                if availableRecordings.isEmpty {
                    VStack(spacing: 16) {
                        Image(systemName: "checkmark.circle")
                            .font(.system(size: 50))
                            .foregroundColor(.green)
                        
                        Text("All Recordings Added")
                            .font(.headline)
                        
                        Text("All available recordings are already in this playlist")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                    }
                    .padding()
                } else {
                    List(availableRecordings, selection: $selectedRecordings) { recording in
                        PlaylistRecordingRow(recording: recording)
                    }
                    .environment(\.editMode, .constant(.active))
                }
            }
            .navigationTitle("Add Recordings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .confirmationAction) {
                    Button("Add (\(selectedRecordings.count))") {
                        playlistManager.addRecordings(Array(selectedRecordings), to: playlist)
                        dismiss()
                    }
                    .disabled(selectedRecordings.isEmpty)
                }
            }
        }
    }
}

// MARK: - Add Recording To Playlists Sheet

/// Sheet for adding a single recording to multiple playlists
struct AddRecordingToPlaylistsSheet: View {
    let recording: Recording?
    let recordingID: String
    let recordingName: String
    
    // Convenience init for local recordings
    init(recording: Recording) {
        self.recording = recording
        self.recordingID = recording.id
        self.recordingName = recording.displayName
    }
    
    // Init for cloud storage recordings (Google Drive, Dropbox)
    init(recordingID: String, recordingName: String) {
        self.recording = nil
        self.recordingID = recordingID
        self.recordingName = recordingName
    }
    
    @StateObject private var playlistManager = PlaylistManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var showCreateSheet = false
    
    var body: some View {
        NavigationStack {
            List {
                if playlistManager.playlists.isEmpty {
                    VStack(spacing: 16) {
                        Image(systemName: "folder.badge.plus")
                            .font(.system(size: 50))
                            .foregroundColor(.gray)
                        
                        Text("No Playlists")
                            .font(.headline)
                        
                        Text("Create a playlist first to organize this recording")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                            .multilineTextAlignment(.center)
                        
                        Button {
                            showCreateSheet = true
                        } label: {
                            Label("Create Playlist", systemImage: "plus")
                        }
                        .buttonStyle(.bordered)
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 20)
                    .listRowBackground(Color.clear)
                } else {
                    ForEach(playlistManager.playlists) { playlist in
                        PlaylistSelectionRowByID(playlist: playlist, recordingID: recordingID, recordingName: recordingName)
                    }
                }
            }
            .navigationTitle("Add to Playlist")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Done") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .topBarTrailing) {
                    Button {
                        showCreateSheet = true
                    } label: {
                        Image(systemName: "plus")
                    }
                }
            }
            .sheet(isPresented: $showCreateSheet) {
                PlaylistEditSheet(mode: .create)
            }
        }
    }
}

struct PlaylistSelectionRowByID: View {
    let playlist: Playlist
    let recordingID: String
    let recordingName: String
    
    @StateObject private var playlistManager = PlaylistManager.shared
    
    private var isInPlaylist: Bool {
        playlist.recordingIDs.contains(recordingID)
    }
    
    var body: some View {
        Button {
            if isInPlaylist {
                playlistManager.removeRecordingByID(recordingID, from: playlist)
            } else {
                playlistManager.addRecordingByID(recordingID, to: playlist)
            }
        } label: {
            HStack(spacing: 12) {
                ZStack {
                    RoundedRectangle(cornerRadius: 6)
                        .fill(playlist.color.color.opacity(0.2))
                        .frame(width: 40, height: 40)
                    
                    Image(systemName: playlist.icon.systemName)
                        .foregroundColor(playlist.color.color)
                }
                
                VStack(alignment: .leading) {
                    Text(playlist.name)
                        .font(.headline)
                        .foregroundColor(.primary)
                    
                    Text("\(playlist.recordingCount) recording\(playlist.recordingCount == 1 ? "" : "s")")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                
                Spacer()
                
                Image(systemName: isInPlaylist ? "checkmark.circle.fill" : "circle")
                    .font(.title2)
                    .foregroundColor(isInPlaylist ? .green : .gray)
            }
        }
    }
}

struct PlaylistSelectionRow: View {
    let playlist: Playlist
    let recording: Recording
    
    @StateObject private var playlistManager = PlaylistManager.shared
    
    private var isInPlaylist: Bool {
        playlist.contains(recording)
    }
    
    var body: some View {
        Button {
            if isInPlaylist {
                playlistManager.removeRecording(recording, from: playlist)
            } else {
                playlistManager.addRecording(recording, to: playlist)
            }
        } label: {
            HStack(spacing: 12) {
                ZStack {
                    RoundedRectangle(cornerRadius: 6)
                        .fill(playlist.color.color.opacity(0.2))
                        .frame(width: 40, height: 40)
                    
                    Image(systemName: playlist.icon.systemName)
                        .foregroundColor(playlist.color.color)
                }
                
                VStack(alignment: .leading) {
                    Text(playlist.name)
                        .font(.headline)
                        .foregroundColor(.primary)
                    
                    Text("\(playlist.recordingCount) recording\(playlist.recordingCount == 1 ? "" : "s")")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                
                Spacer()
                
                Image(systemName: isInPlaylist ? "checkmark.circle.fill" : "circle")
                    .font(.title2)
                    .foregroundColor(isInPlaylist ? .green : .gray)
            }
        }
    }
}

#Preview("Playlists List") {
    NavigationStack {
        PlaylistsListView()
    }
}

#Preview("Create Playlist") {
    PlaylistEditSheet(mode: .create)
}
