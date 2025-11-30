//
//  RecordingThumbnailView.swift
//  Tracking Viewer
//
//  Created on 11/29/25.
//

import SwiftUI

/// Displays a video thumbnail with recording info
struct RecordingThumbnailView: View {
    let recording: Recording
    let isSelected: Bool
    
    @State private var thumbnail: UIImage?
    @State private var isLoadingThumbnail: Bool = true
    
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            // Thumbnail
            GeometryReader { geometry in
                ZStack {
                    if let thumbnail = thumbnail {
                        Image(uiImage: thumbnail)
                            .resizable()
                            .aspectRatio(contentMode: .fill)
                            .frame(width: geometry.size.width, height: geometry.size.height)
                            .clipped()
                    } else {
                        Rectangle()
                            .fill(Color.gray.opacity(0.3))
                            .overlay {
                                if isLoadingThumbnail {
                                    ProgressView()
                                } else {
                                    Image(systemName: "video.slash")
                                        .font(.largeTitle)
                                        .foregroundColor(.gray)
                                }
                            }
                    }
                    
                    // Duration badge
                    VStack {
                        Spacer()
                        HStack {
                            Spacer()
                            Text(recording.durationString)
                                .font(.caption)
                                .fontWeight(.semibold)
                                .foregroundColor(.white)
                                .padding(.horizontal, 6)
                                .padding(.vertical, 2)
                                .background(Color.black.opacity(0.7))
                                .cornerRadius(4)
                                .padding(6)
                        }
                    }
                    
                    // Selection indicator
                    if isSelected {
                        VStack {
                            HStack {
                                Image(systemName: "checkmark.circle.fill")
                                    .font(.title2)
                                    .foregroundColor(.blue)
                                    .background(Color.white.clipShape(Circle()))
                                    .padding(6)
                                Spacer()
                            }
                            Spacer()
                        }
                    }
                }
            }
            .frame(height: 120)
            .cornerRadius(8)
            .overlay(
                RoundedRectangle(cornerRadius: 8)
                    .stroke(isSelected ? Color.blue : Color.clear, lineWidth: 3)
            )
            
            // Info
            VStack(alignment: .leading, spacing: 2) {
                Text(recording.displayName)
                    .font(.subheadline)
                    .fontWeight(.medium)
                    .lineLimit(1)
                
                HStack(spacing: 8) {
                    Label(recording.frameCountString, systemImage: "photo.on.rectangle")
                    Label(recording.fileSizeString, systemImage: "doc")
                }
                .font(.caption)
                .foregroundColor(.secondary)
            }
            .padding(.horizontal, 4)
        }
        .task {
            await loadThumbnail()
        }
    }
    
    private func loadThumbnail() async {
        isLoadingThumbnail = true
        thumbnail = await RecordingsManager.shared.getThumbnail(for: recording)
        isLoadingThumbnail = false
    }
}

#Preview {
    RecordingThumbnailView(
        recording: Recording(
            id: "test",
            folderURL: URL(fileURLWithPath: "/test"),
            metadata: nil,
            videoURL: nil,
            trackingURL: nil
        ),
        isSelected: false
    )
    .frame(width: 180)
    .padding()
}
