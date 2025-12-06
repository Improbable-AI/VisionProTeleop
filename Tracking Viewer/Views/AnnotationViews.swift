//
//  AnnotationViews.swift
//  Tracking Viewer
//
//  Created on 12/02/25.
//

import SwiftUI

// MARK: - Tags Management View

/// View for managing all tags (create, edit, delete)
struct TagsManagementView: View {
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @State private var showCreateSheet = false
    @State private var tagToEdit: Tag?
    @State private var showDeleteConfirmation = false
    @State private var tagToDelete: Tag?
    
    var body: some View {
        List {
            if annotationsManager.allTags.isEmpty {
                VStack(spacing: 16) {
                    Image(systemName: "tag")
                        .font(.system(size: 50))
                        .foregroundColor(.gray)
                    
                    Text("No Tags")
                        .font(.headline)
                    
                    Text("Create tags to organize your recordings")
                        .font(.subheadline)
                        .foregroundColor(.secondary)
                    
                    Button {
                        showCreateSheet = true
                    } label: {
                        Label("Create Tag", systemImage: "plus")
                    }
                    .buttonStyle(.bordered)
                }
                .frame(maxWidth: .infinity)
                .padding(.vertical, 20)
                .listRowBackground(Color.clear)
            } else {
                ForEach(annotationsManager.allTags) { tag in
                    TagManagementRow(tag: tag)
                        .swipeActions(edge: .trailing, allowsFullSwipe: false) {
                            Button(role: .destructive) {
                                tagToDelete = tag
                                showDeleteConfirmation = true
                            } label: {
                                Label("Delete", systemImage: "trash")
                            }
                            
                            Button {
                                tagToEdit = tag
                            } label: {
                                Label("Edit", systemImage: "pencil")
                            }
                            .tint(.orange)
                        }
                }
            }
        }
        .navigationTitle("Manage Tags")
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
            TagEditSheet(mode: .create)
        }
        .sheet(item: $tagToEdit) { tag in
            TagEditSheet(mode: .edit(tag))
        }
        .alert("Delete Tag?", isPresented: $showDeleteConfirmation) {
            Button("Cancel", role: .cancel) {}
            Button("Delete", role: .destructive) {
                if let tag = tagToDelete {
                    annotationsManager.deleteTag(tag)
                }
            }
        } message: {
            if let tag = tagToDelete {
                Text("This will remove '\(tag.name)' from all recordings.")
            }
        }
    }
}

struct TagManagementRow: View {
    let tag: Tag
    @StateObject private var annotationsManager = AnnotationsManager.shared
    
    var body: some View {
        HStack(spacing: 12) {
            Circle()
                .fill(tag.color.color)
                .frame(width: 24, height: 24)
            
            Text(tag.name)
                .font(.body)
            
            Spacer()
            
            Text("\(annotationsManager.recordingsCount(for: tag)) recordings")
                .font(.caption)
                .foregroundColor(.secondary)
        }
        .padding(.vertical, 4)
    }
}

// MARK: - Tag Edit Sheet

struct TagEditSheet: View {
    enum Mode {
        case create
        case edit(Tag)
    }
    
    let mode: Mode
    
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var name: String = ""
    @State private var selectedColor: TagColor = .gray
    
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
                Section("Tag Name") {
                    TextField("Name", text: $name)
                }
                
                Section("Color") {
                    LazyVGrid(columns: Array(repeating: GridItem(.flexible()), count: 6), spacing: 12) {
                        ForEach(TagColor.allCases, id: \.self) { color in
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
                
                // Preview
                Section("Preview") {
                    HStack(spacing: 8) {
                        Circle()
                            .fill(selectedColor.color)
                            .frame(width: 12, height: 12)
                        Text(name.isEmpty ? "Tag Name" : name)
                            .font(.caption)
                            .foregroundColor(name.isEmpty ? .gray : .primary)
                    }
                    .padding(.horizontal, 10)
                    .padding(.vertical, 6)
                    .background(selectedColor.color.opacity(0.2))
                    .cornerRadius(12)
                }
            }
            .navigationTitle(isEditing ? "Edit Tag" : "New Tag")
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
                if case .edit(let tag) = mode {
                    name = tag.name
                    selectedColor = tag.color
                }
            }
        }
    }
    
    private func save() {
        let trimmedName = name.trimmingCharacters(in: .whitespaces)
        
        switch mode {
        case .create:
            _ = annotationsManager.createTag(name: trimmedName, color: selectedColor)
        case .edit(let tag):
            annotationsManager.updateTag(tag, name: trimmedName, color: selectedColor)
        }
    }
}

// MARK: - Tag Picker Sheet

/// Sheet for selecting tags for a recording
struct TagPickerSheet: View {
    let recording: Recording
    
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var showCreateSheet = false
    
    var body: some View {
        NavigationStack {
            List {
                if annotationsManager.allTags.isEmpty {
                    VStack(spacing: 12) {
                        Text("No tags available")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                        
                        Button {
                            showCreateSheet = true
                        } label: {
                            Label("Create Tag", systemImage: "plus")
                        }
                        .buttonStyle(.bordered)
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 12)
                    .listRowBackground(Color.clear)
                } else {
                    ForEach(annotationsManager.allTags) { tag in
                        Button {
                            annotationsManager.toggleTag(tag, for: recording)
                        } label: {
                            HStack(spacing: 12) {
                                Circle()
                                    .fill(tag.color.color)
                                    .frame(width: 20, height: 20)
                                
                                Text(tag.name)
                                    .foregroundColor(.primary)
                                
                                Spacer()
                                
                                if annotationsManager.hasTag(recording, tag: tag) {
                                    Image(systemName: "checkmark")
                                        .foregroundColor(.blue)
                                }
                            }
                        }
                    }
                }
            }
            .navigationTitle("Select Tags")
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
                TagEditSheet(mode: .create)
            }
        }
    }
}

// MARK: - Rename Recording Sheet

struct RenameRecordingSheet: View {
    let recording: Recording
    
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var customName: String = ""
    @FocusState private var isFocused: Bool
    
    var body: some View {
        NavigationStack {
            Form {
                Section("Custom Name") {
                    TextField("Enter name", text: $customName)
                        .focused($isFocused)
                }
                
                Section {
                    VStack(alignment: .leading, spacing: 8) {
                        Text("Original Name")
                            .font(.caption)
                            .foregroundColor(.secondary)
                        Text(recording.displayName)
                            .font(.subheadline)
                    }
                    
                    if !customName.isEmpty {
                        Button("Use Original Name") {
                            customName = ""
                        }
                        .foregroundColor(.red)
                    }
                }
            }
            .navigationTitle("Rename Recording")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .confirmationAction) {
                    Button("Save") {
                        let name = customName.trimmingCharacters(in: .whitespaces)
                        annotationsManager.setCustomName(for: recording, name.isEmpty ? nil : name)
                        dismiss()
                    }
                }
            }
            .onAppear {
                customName = annotationsManager.customName(for: recording) ?? ""
                isFocused = true
            }
        }
    }
}

// MARK: - Tag Chips View

/// Horizontal scrolling view of tag chips
struct TagChipsView: View {
    let tags: [Tag]
    var showRemoveButton: Bool = false
    var onRemove: ((Tag) -> Void)? = nil
    
    var body: some View {
        ScrollView(.horizontal, showsIndicators: false) {
            HStack(spacing: 6) {
                ForEach(tags) { tag in
                    HStack(spacing: 4) {
                        Circle()
                            .fill(tag.color.color)
                            .frame(width: 8, height: 8)
                        
                        Text(tag.name)
                            .font(.caption)
                        
                        if showRemoveButton {
                            Button {
                                onRemove?(tag)
                            } label: {
                                Image(systemName: "xmark.circle.fill")
                                    .font(.caption2)
                                    .foregroundColor(.secondary)
                            }
                        }
                    }
                    .padding(.horizontal, 8)
                    .padding(.vertical, 4)
                    .background(tag.color.color.opacity(0.2))
                    .cornerRadius(12)
                }
            }
        }
    }
}

// MARK: - Favorite Button

struct FavoriteButton: View {
    let recording: Recording
    @StateObject private var annotationsManager = AnnotationsManager.shared
    
    var body: some View {
        Button {
            annotationsManager.toggleFavorite(recording)
        } label: {
            Image(systemName: annotationsManager.isFavorite(recording) ? "star.fill" : "star")
                .foregroundColor(annotationsManager.isFavorite(recording) ? .yellow : .gray)
        }
    }
}

// MARK: - Tag Picker View (by recording ID)

/// Tag picker that takes a recording ID instead of Recording object
struct TagPickerView: View {
    let recordingID: String
    
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var showCreateSheet = false
    
    var body: some View {
        NavigationStack {
            List {
                if annotationsManager.allTags.isEmpty {
                    VStack(spacing: 12) {
                        Text("No tags available")
                            .font(.subheadline)
                            .foregroundColor(.secondary)
                        
                        Button {
                            showCreateSheet = true
                        } label: {
                            Label("Create Tag", systemImage: "plus")
                        }
                        .buttonStyle(.bordered)
                    }
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 12)
                    .listRowBackground(Color.clear)
                } else {
                    ForEach(annotationsManager.allTags) { tag in
                        Button {
                            annotationsManager.toggleTag(tag, for: recordingID)
                        } label: {
                            HStack(spacing: 12) {
                                Circle()
                                    .fill(tag.color.color)
                                    .frame(width: 20, height: 20)
                                
                                Text(tag.name)
                                    .foregroundColor(.primary)
                                
                                Spacer()
                                
                                if annotationsManager.hasTag(recordingID, tag: tag) {
                                    Image(systemName: "checkmark")
                                        .foregroundColor(.blue)
                                }
                            }
                        }
                    }
                }
            }
            .navigationTitle("Select Tags")
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
                TagEditSheet(mode: .create)
            }
        }
    }
}

// MARK: - Notes Editor (by recording ID)

struct NotesEditorSheet: View {
    let recordingID: String
    let recordingName: String
    
    @StateObject private var annotationsManager = AnnotationsManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var notes: String = ""
    @FocusState private var isFocused: Bool
    
    // Convenience init for Recording object
    init(recording: Recording) {
        self.recordingID = recording.id
        self.recordingName = recording.displayName
    }
    
    // Primary init
    init(recordingID: String, recordingName: String) {
        self.recordingID = recordingID
        self.recordingName = recordingName
    }
    
    var body: some View {
        NavigationStack {
            VStack(spacing: 0) {
                TextEditor(text: $notes)
                    .focused($isFocused)
                    .padding()
                
                // Character count
                HStack {
                    Spacer()
                    Text("\(notes.count) characters")
                        .font(.caption)
                        .foregroundColor(.secondary)
                }
                .padding(.horizontal)
                .padding(.bottom, 8)
            }
            .navigationTitle("Notes - \(recordingName)")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .cancellationAction) {
                    Button("Cancel") {
                        dismiss()
                    }
                }
                
                ToolbarItem(placement: .confirmationAction) {
                    Button("Save") {
                        annotationsManager.setNotes(notes, for: recordingID)
                        dismiss()
                    }
                }
            }
            .onAppear {
                notes = annotationsManager.notes(for: recordingID) ?? ""
                isFocused = true
            }
        }
    }
}

// MARK: - Flow Layout

/// A layout that flows items horizontally and wraps to new lines
struct FlowLayout: Layout {
    var spacing: CGFloat = 8
    
    func sizeThatFits(proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) -> CGSize {
        let sizes = subviews.map { $0.sizeThatFits(.unspecified) }
        return layoutFlow(sizes: sizes, proposal: proposal).size
    }
    
    func placeSubviews(in bounds: CGRect, proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) {
        let sizes = subviews.map { $0.sizeThatFits(.unspecified) }
        let result = layoutFlow(sizes: sizes, proposal: proposal)
        
        for (index, subview) in subviews.enumerated() {
            let position = result.positions[index]
            subview.place(
                at: CGPoint(x: bounds.minX + position.x, y: bounds.minY + position.y),
                proposal: .unspecified
            )
        }
    }
    
    private func layoutFlow(sizes: [CGSize], proposal: ProposedViewSize) -> (size: CGSize, positions: [CGPoint]) {
        let maxWidth = proposal.width ?? .infinity
        var positions: [CGPoint] = []
        var currentX: CGFloat = 0
        var currentY: CGFloat = 0
        var lineHeight: CGFloat = 0
        var maxX: CGFloat = 0
        
        for size in sizes {
            if currentX + size.width > maxWidth && currentX > 0 {
                // Move to next line
                currentX = 0
                currentY += lineHeight + spacing
                lineHeight = 0
            }
            
            positions.append(CGPoint(x: currentX, y: currentY))
            currentX += size.width + spacing
            lineHeight = max(lineHeight, size.height)
            maxX = max(maxX, currentX - spacing)
        }
        
        let height = currentY + lineHeight
        return (CGSize(width: maxX, height: height), positions)
    }
}

#Preview("Tags Management") {
    NavigationStack {
        TagsManagementView()
    }
}

#Preview("Tag Edit") {
    TagEditSheet(mode: .create)
}
