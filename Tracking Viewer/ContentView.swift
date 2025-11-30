//
//  ContentView.swift
//  Tracking Viewer
//
//  Created by younghyopark on 11/29/25.
//

import SwiftUI

struct ContentView: View {
    @StateObject private var recordingsManager = RecordingsManager.shared
    @State private var isSelectionMode = false
    @State private var showExportSheet = false
    @State private var exportURLs: [URL] = []
    @State private var showDeleteConfirmation = false
    @State private var recordingToDelete: Recording?
    @State private var showDeleteSelectedConfirmation = false
    @State private var showExportAllSheet = false
    
    private let columns = [
        GridItem(.adaptive(minimum: 160, maximum: 200), spacing: 16)
    ]
    
    var body: some View {
        NavigationStack {
            Group {
                if recordingsManager.isLoading {
                    ProgressView("Loading recordings...")
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                } else if recordingsManager.recordings.isEmpty {
                    emptyStateView
                } else {
                    recordingsGridView
                }
            }
            .navigationTitle("Recordings")
            .toolbar {
                ToolbarItem(placement: .topBarLeading) {
                    Button(action: {
                        Task { await recordingsManager.loadRecordings() }
                    }) {
                        Image(systemName: "arrow.clockwise")
                    }
                }
                
                ToolbarItem(placement: .topBarTrailing) {
                    if !recordingsManager.recordings.isEmpty {
                        HStack {
                            Menu {
                                Button(action: {
                                    Task {
                                        exportURLs = await recordingsManager.exportRecordings(recordingsManager.recordings)
                                        showExportAllSheet = true
                                    }
                                }) {
                                    Label("Export All", systemImage: "square.and.arrow.up.on.square")
                                }
                            } label: {
                                Image(systemName: "ellipsis.circle")
                            }
                            
                            Button(isSelectionMode ? "Done" : "Select") {
                                withAnimation {
                                    isSelectionMode.toggle()
                                    if !isSelectionMode {
                                        recordingsManager.deselectAll()
                                    }
                                }
                            }
                        }
                    }
                }
            }
            .overlay(alignment: .bottom) {
                if isSelectionMode && !recordingsManager.selectedRecordings.isEmpty {
                    selectionToolbar
                }
            }
            .task {
                await recordingsManager.loadRecordings()
            }
            .sheet(isPresented: $showExportSheet) {
                ShareSheet(items: exportURLs)
            }
            .alert("Delete Recording?", isPresented: $showDeleteConfirmation) {
                Button("Cancel", role: .cancel) {}
                Button("Delete", role: .destructive) {
                    if let recording = recordingToDelete {
                        Task {
                            try? await recordingsManager.deleteRecording(recording)
                        }
                    }
                }
            } message: {
                Text("This will permanently delete the recording and all its data.")
            }
            .alert("Delete \(recordingsManager.selectedRecordings.count) Recordings?", isPresented: $showDeleteSelectedConfirmation) {
                Button("Cancel", role: .cancel) {}
                Button("Delete All", role: .destructive) {
                    Task {
                        for recording in recordingsManager.selectedRecordings {
                            try? await recordingsManager.deleteRecording(recording)
                        }
                        isSelectionMode = false
                    }
                }
            } message: {
                Text("This will permanently delete all selected recordings and their data.")
            }
            .sheet(isPresented: $showExportAllSheet) {
                ShareSheet(items: exportURLs)
            }
        }
    }
    
    // MARK: - Empty State
    
    private var emptyStateView: some View {
        VStack(spacing: 16) {
            Image(systemName: "video.badge.plus")
                .font(.system(size: 60))
                .foregroundColor(.gray)
            
            Text("No Recordings")
                .font(.title2)
                .fontWeight(.semibold)
            
            Text("Recordings from Vision Pro will appear here.\nMake sure iCloud Drive is enabled.")
                .font(.subheadline)
                .foregroundColor(.secondary)
                .multilineTextAlignment(.center)
                .padding(.horizontal, 40)
            
            Button(action: {
                Task { await recordingsManager.loadRecordings() }
            }) {
                Label("Refresh", systemImage: "arrow.clockwise")
                    .padding(.horizontal, 20)
                    .padding(.vertical, 10)
            }
            .buttonStyle(.bordered)
            .padding(.top)
            
            if let error = recordingsManager.errorMessage {
                Text(error)
                    .font(.caption)
                    .foregroundColor(.red)
                    .padding(.top)
            }
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
    
    // MARK: - Recordings Grid
    
    private var recordingsGridView: some View {
        ScrollView {
            LazyVGrid(columns: columns, spacing: 20) {
                ForEach(recordingsManager.recordings) { recording in
                    if isSelectionMode {
                        // Selection mode - tap to select
                        Button(action: {
                            recordingsManager.toggleSelection(recording)
                        }) {
                            RecordingThumbnailView(
                                recording: recording,
                                isSelected: recordingsManager.selectedRecordings.contains(recording)
                            )
                        }
                        .buttonStyle(.plain)
                        .contextMenu {
                            recordingContextMenu(for: recording)
                        }
                    } else {
                        // Normal mode - tap to view details
                        NavigationLink(destination: RecordingDetailView(recording: recording)) {
                            RecordingThumbnailView(
                                recording: recording,
                                isSelected: false
                            )
                        }
                        .buttonStyle(.plain)
                        .contextMenu {
                            recordingContextMenu(for: recording)
                        }
                    }
                }
            }
            .padding()
        }
    }
    
    // MARK: - Context Menu
    
    @ViewBuilder
    private func recordingContextMenu(for recording: Recording) -> some View {
        Button(action: {
            exportURLs = [recording.folderURL]
            showExportSheet = true
        }) {
            Label("Export", systemImage: "square.and.arrow.up")
        }
        
        Button(role: .destructive, action: {
            recordingToDelete = recording
            showDeleteConfirmation = true
        }) {
            Label("Delete", systemImage: "trash")
        }
    }
    
    // MARK: - Selection Toolbar
    
    private var selectionToolbar: some View {
        HStack(spacing: 16) {
            Button(action: {
                if recordingsManager.selectedRecordings.count == recordingsManager.recordings.count {
                    recordingsManager.deselectAll()
                } else {
                    recordingsManager.selectAll()
                }
            }) {
                Text(recordingsManager.selectedRecordings.count == recordingsManager.recordings.count 
                     ? "Deselect All" : "Select All")
                    .font(.subheadline)
            }
            
            Spacer()
            
            Text("\(recordingsManager.selectedRecordings.count) selected")
                .foregroundColor(.secondary)
                .font(.subheadline)
            
            Spacer()
            
            // Delete button
            Button(action: {
                showDeleteSelectedConfirmation = true
            }) {
                Image(systemName: "trash")
                    .font(.title3)
                    .foregroundColor(.red)
            }
            .disabled(recordingsManager.selectedRecordings.isEmpty)
            
            // Export button
            Button(action: {
                Task {
                    let selected = Array(recordingsManager.selectedRecordings)
                    exportURLs = await recordingsManager.exportRecordings(selected)
                    showExportSheet = true
                }
            }) {
                Image(systemName: "square.and.arrow.up")
                    .font(.title3)
            }
            .disabled(recordingsManager.selectedRecordings.isEmpty)
        }
        .padding()
        .background(.regularMaterial)
    }
}

#Preview {
    ContentView()
}
