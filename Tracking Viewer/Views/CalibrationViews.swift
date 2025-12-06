//
//  CalibrationViews.swift
//  Tracking Viewer
//
//  Views for displaying calibration markers and patterns on iPhone.
//

import SwiftUI

// MARK: - ArUco Marker Display View

/// Full-screen ArUco marker display for calibration
struct ArucoMarkerDisplayView: View {
    @StateObject private var manager = CalibrationDisplayManager.shared
    @StateObject private var multipeerManager = MultipeerCalibrationManager_iOS.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var showControls = true
    @State private var hideControlsTimer: Timer?
    @State private var showPositionChangeAlert = false
    @State private var brightness: CGFloat = UIScreen.main.brightness
    
    /// Whether to hide marker when phone is moving
    /// Works both in connected mode (pauses Vision Pro collection) and standalone mode (visual feedback)
    private var shouldHideMarker: Bool {
        // Hide marker when motion is detected (phone not stationary)
        // This works regardless of connection status for visual feedback
        !multipeerManager.isPhoneStationary && multipeerManager.motionDetector.isRunning
    }
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // Background color changes based on motion state
                (shouldHideMarker ? Color.black : Color.white)
                    .ignoresSafeArea()
                    .animation(.easeInOut(duration: 0.3), value: shouldHideMarker)
                
                // Show marker only when stationary (or not connected)
                if !shouldHideMarker {
                    // Centered marker with shadow for depth
                    if let markerImage = manager.generateArucoMarker(
                        id: manager.currentMarkerId,
                        sizePoints: manager.markerSizePoints
                    ) {
                        Image(uiImage: markerImage)
                            .interpolation(.none)
                            .resizable()
                            .frame(width: manager.markerSizePoints, height: manager.markerSizePoints)
                            .transition(.opacity)
                    }
                } else {
                    // Motion detected - show warning screen
                    motionWarningOverlay
                        .transition(.opacity)
                }
                
                // Elegant controls overlay
                if showControls {
                    // Gradient overlays
                    VStack(spacing: 0) {
                        // Top gradient
                        LinearGradient(
                            colors: [Color.black.opacity(0.4), Color.clear],
                            startPoint: .top,
                            endPoint: .bottom
                        )
                        .frame(height: 140)
                        
                        Spacer()
                        
                        // Bottom gradient
                        LinearGradient(
                            colors: [Color.clear, Color.black.opacity(0.6)],
                            startPoint: .top,
                            endPoint: .bottom
                        )
                        .frame(height: 320)
                    }
                    .ignoresSafeArea()
                    .allowsHitTesting(false)
                    
                    VStack {
                        // Top bar
                        HStack(alignment: .top) {
                            // Close button
                            Button(action: { dismiss() }) {
                                Image(systemName: "xmark")
                                    .font(.system(size: 16, weight: .bold))
                                    .foregroundColor(.white)
                                    .frame(width: 36, height: 36)
                                    .background(Color.black.opacity(0.5))
                                    .clipShape(Circle())
                            }
                            
                            Spacer()
                            
                            // Center info pill
                            HStack(spacing: 16) {
                                VStack(spacing: 2) {
                                    Text("ID")
                                        .font(.system(size: 10, weight: .medium))
                                        .foregroundColor(.white.opacity(0.7))
                                    Text("\(manager.currentMarkerId)")
                                        .font(.system(size: 24, weight: .bold, design: .rounded))
                                        .foregroundColor(.white)
                                }
                                
                                Rectangle()
                                    .fill(Color.white.opacity(0.3))
                                    .frame(width: 1, height: 30)
                                
                                VStack(spacing: 2) {
                                    Text("SIZE")
                                        .font(.system(size: 10, weight: .medium))
                                        .foregroundColor(.white.opacity(0.7))
                                    Text("\(Int(manager.arucoConfig.markerSizeMM))mm")
                                        .font(.system(size: 16, weight: .semibold, design: .rounded))
                                        .foregroundColor(.white)
                                }
                            }
                            .padding(.horizontal, 20)
                            .padding(.vertical, 12)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Capsule())
                            
                            Spacer()
                            
                            // Placeholder for symmetry
                            Color.clear
                                .frame(width: 36, height: 36)
                        }
                        .padding(.horizontal, 20)
                        .padding(.top, 8)
                        
                        Spacer()
                        
                        // Bottom controls
                        VStack(spacing: 16) {
                            // Connection status (if connected to Vision Pro)
                            if multipeerManager.isConnected {
                                HStack(spacing: 8) {
                                    Image(systemName: "visionpro")
                                        .font(.system(size: 14))
                                    Text("Connected to Vision Pro")
                                        .font(.system(size: 12, weight: .medium))
                                    
                                    Spacer()
                                    
                                    // Motion indicator
                                    HStack(spacing: 4) {
                                        Circle()
                                            .fill(multipeerManager.isPhoneStationary ? Color.green : Color.orange)
                                            .frame(width: 8, height: 8)
                                        Text(multipeerManager.isPhoneStationary ? "Collecting" : "Moving")
                                            .font(.system(size: 11, weight: .medium))
                                    }
                                }
                                .foregroundColor(.white)
                                .padding(.horizontal, 16)
                                .padding(.vertical, 10)
                                .background(Color.green.opacity(0.8))
                                .clipShape(Capsule())
                            }
                            
                            // Position indicator
                            HStack(spacing: 8) {
                                ForEach(0..<manager.arucoConfig.availableMarkerIds.count, id: \.self) { index in
                                    Circle()
                                        .fill(index == manager.sessionState.currentPositionIndex ? Color.white : Color.white.opacity(0.3))
                                        .frame(width: index == manager.sessionState.currentPositionIndex ? 10 : 8,
                                               height: index == manager.sessionState.currentPositionIndex ? 10 : 8)
                                        .animation(.spring(response: 0.3), value: manager.sessionState.currentPositionIndex)
                                }
                            }
                            .padding(.vertical, 8)
                            
                            // Warning/status banner
                            HStack(spacing: 10) {
                                Image(systemName: multipeerManager.isPhoneStationary ? "hand.raised.fill" : "figure.walk")
                                    .font(.system(size: 16))
                                Text(multipeerManager.isConnected 
                                     ? (multipeerManager.isPhoneStationary ? "Steady - collecting samples" : "Moving - paused")
                                     : "Hold phone steady while collecting samples")
                                    .font(.system(size: 13, weight: .medium))
                            }
                            .foregroundColor(.white)
                            .padding(.horizontal, 16)
                            .padding(.vertical, 10)
                            .background(multipeerManager.isPhoneStationary ? Color.green.opacity(0.9) : Color.orange.opacity(0.9))
                            .clipShape(Capsule())
                            
                            // Navigation controls (only show when not connected, or Vision Pro controls marker switching)
                            if !multipeerManager.isConnected {
                                HStack(spacing: 12) {
                                    // Previous button
                                    Button(action: { 
                                        withAnimation(.spring(response: 0.3)) {
                                            manager.previousMarkerId()
                                        }
                                    }) {
                                        Image(systemName: "chevron.left")
                                            .font(.system(size: 20, weight: .semibold))
                                            .foregroundColor(.white)
                                            .frame(width: 50, height: 50)
                                            .background(Color.white.opacity(0.2))
                                            .clipShape(Circle())
                                    }
                                    
                                    // Main action button
                                    Button(action: { showPositionChangeAlert = true }) {
                                        HStack(spacing: 8) {
                                            Image(systemName: "arrow.right.circle.fill")
                                                .font(.system(size: 18))
                                            Text("Next Position")
                                                .font(.system(size: 15, weight: .semibold))
                                        }
                                        .foregroundColor(.black)
                                        .padding(.horizontal, 24)
                                        .padding(.vertical, 14)
                                        .background(Color.white)
                                        .clipShape(Capsule())
                                    }
                                    
                                    // Next button
                                    Button(action: { 
                                        withAnimation(.spring(response: 0.3)) {
                                            manager.nextMarkerId()
                                        }
                                    }) {
                                        Image(systemName: "chevron.right")
                                            .font(.system(size: 20, weight: .semibold))
                                            .foregroundColor(.white)
                                            .frame(width: 50, height: 50)
                                            .background(Color.white.opacity(0.2))
                                            .clipShape(Circle())
                                    }
                                }
                            } else {
                                // When connected, show Vision Pro is controlling
                                HStack(spacing: 8) {
                                    Image(systemName: "visionpro")
                                    Text("Marker switching controlled by Vision Pro")
                                        .font(.system(size: 13, weight: .medium))
                                }
                                .foregroundColor(.white.opacity(0.8))
                                .padding(.vertical, 8)
                            }
                            
                            // Tap hint
                            Text("Tap anywhere to hide controls")
                                .font(.system(size: 11))
                                .foregroundColor(.white.opacity(0.5))
                                .padding(.top, 8)
                        }
                        .padding(.bottom, 40)
                    }
                    .transition(.opacity.combined(with: .scale(scale: 0.98)))
                }
            }
            .onTapGesture {
                withAnimation(.easeInOut(duration: 0.2)) {
                    showControls.toggle()
                }
                if showControls {
                    resetHideTimer()
                }
            }
        }
        .statusBarHidden(true)
        .onAppear {
            // Max brightness for best marker detection
            brightness = UIScreen.main.brightness
            UIScreen.main.brightness = 1.0
            manager.startDisplayingMarker()
            resetHideTimer()
            
            // Always start motion detection when displaying markers
            // This enables the motion indicator even when not connected
            multipeerManager.startStatusUpdates()
            print("📱 [ArucoMarkerDisplayView] Started - motion detection enabled")
        }
        .onDisappear {
            // Restore brightness
            UIScreen.main.brightness = brightness
            manager.stopDisplayingMarker()
            hideControlsTimer?.invalidate()
            
            // Stop motion detection
            multipeerManager.stopStatusUpdates()
            print("📱 [ArucoMarkerDisplayView] Stopped")
        }
        .alert("Move to New Position", isPresented: $showPositionChangeAlert) {
            Button("Cancel", role: .cancel) {}
            Button("I've Moved") {
                withAnimation(.spring(response: 0.3)) {
                    manager.nextMarkerId()
                    manager.signalPositionChange()
                }
            }
        } message: {
            Text("Place your iPhone at a different location, then tap 'I've Moved'. Hold the phone completely still during sample collection.")
        }
    }
    
    // MARK: - Motion Warning Overlay
    
    private var motionWarningOverlay: some View {
        VStack(spacing: 24) {
            Spacer()
            
            // Animated motion icon
            Image(systemName: "iphone.gen3.radiowaves.left.and.right")
                .font(.system(size: 80))
                .foregroundColor(.orange)
                .symbolEffect(.pulse, options: .repeating)
            
            VStack(spacing: 12) {
                Text("Motion Detected")
                    .font(.system(size: 28, weight: .bold))
                    .foregroundColor(.white)
                
                Text("Hold iPhone still to display marker")
                    .font(.system(size: 17))
                    .foregroundColor(.white.opacity(0.8))
                
                // Motion magnitude indicator
                HStack(spacing: 4) {
                    ForEach(0..<5, id: \.self) { index in
                        RoundedRectangle(cornerRadius: 2)
                            .fill(index < Int(multipeerManager.motionMagnitude * 5) + 1 ? Color.orange : Color.white.opacity(0.3))
                            .frame(width: 20, height: 8)
                    }
                }
                .padding(.top, 8)
                
                // Debug info
                Text("Motion: \(String(format: "%.1f%%", multipeerManager.motionMagnitude * 100))")
                    .font(.system(size: 12, design: .monospaced))
                    .foregroundColor(.white.opacity(0.5))
            }
            
            Spacer()
            
            // Hint text
            VStack(spacing: 8) {
                Text("Marker ID \(manager.currentMarkerId)")
                    .font(.system(size: 15, weight: .semibold))
                    .foregroundColor(.white.opacity(0.6))
                
                Text("Marker will appear when phone is steady")
                    .font(.system(size: 13))
                    .foregroundColor(.white.opacity(0.5))
            }
            .padding(.bottom, 60)
        }
    }
    
    private func resetHideTimer() {
        hideControlsTimer?.invalidate()
        hideControlsTimer = Timer.scheduledTimer(withTimeInterval: 4.0, repeats: false) { _ in
            withAnimation(.easeInOut(duration: 0.2)) {
                showControls = false
            }
        }
    }
}

// MARK: - Checkerboard Display View

/// Full-screen checkerboard pattern display for intrinsic calibration
struct CheckerboardDisplayView: View {
    @StateObject private var manager = CalibrationDisplayManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var showControls = true
    @State private var hideControlsTimer: Timer?
    @State private var brightness: CGFloat = UIScreen.main.brightness
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // White background
                Color.white
                    .ignoresSafeArea()
                
                // Checkerboard pattern - rotated 90° to fit better on portrait screen
                CheckerboardPatternView(
                    innerCornersX: manager.checkerboardConfig.innerCornersX,
                    innerCornersY: manager.checkerboardConfig.innerCornersY,
                    squareSizePoints: manager.mmToPoints(Double(manager.checkerboardConfig.squareSizeMM))
                )
                .rotationEffect(.degrees(90))
                .frame(maxWidth: geometry.size.width, maxHeight: geometry.size.height)
                .clipped()
                
                // Elegant controls overlay
                if showControls {
                    // Gradient overlays
                    VStack(spacing: 0) {
                        // Top gradient
                        LinearGradient(
                            colors: [Color.black.opacity(0.4), Color.clear],
                            startPoint: .top,
                            endPoint: .bottom
                        )
                        .frame(height: 140)
                        
                        Spacer()
                        
                        // Bottom gradient
                        LinearGradient(
                            colors: [Color.clear, Color.black.opacity(0.6)],
                            startPoint: .top,
                            endPoint: .bottom
                        )
                        .frame(height: 220)
                    }
                    .ignoresSafeArea()
                    .allowsHitTesting(false)
                    
                    VStack {
                        // Top bar - SAME PATTERN AS ARUCO
                        HStack(alignment: .top) {
                            // Close button
                            Button(action: { dismiss() }) {
                                Image(systemName: "xmark")
                                    .font(.system(size: 16, weight: .bold))
                                    .foregroundColor(.white)
                                    .frame(width: 36, height: 36)
                                    .background(Color.black.opacity(0.5))
                                    .clipShape(Circle())
                            }
                            
                            Spacer()
                            
                            // Center info pill
                            HStack(spacing: 16) {
                                VStack(spacing: 2) {
                                    Text("GRID")
                                        .font(.system(size: 10, weight: .medium))
                                        .foregroundColor(.white.opacity(0.7))
                                    Text("\(manager.checkerboardConfig.innerCornersX + 1)×\(manager.checkerboardConfig.innerCornersY + 1)")
                                        .font(.system(size: 18, weight: .bold, design: .rounded))
                                        .foregroundColor(.white)
                                }
                                
                                Rectangle()
                                    .fill(Color.white.opacity(0.3))
                                    .frame(width: 1, height: 30)
                                
                                VStack(spacing: 2) {
                                    Text("SQUARE")
                                        .font(.system(size: 10, weight: .medium))
                                        .foregroundColor(.white.opacity(0.7))
                                    Text("\(Int(manager.checkerboardConfig.squareSizeMM))mm")
                                        .font(.system(size: 16, weight: .semibold, design: .rounded))
                                        .foregroundColor(.white)
                                }
                            }
                            .padding(.horizontal, 20)
                            .padding(.vertical, 12)
                            .background(Color.black.opacity(0.5))
                            .clipShape(Capsule())
                            
                            Spacer()
                            
                            // Placeholder for symmetry
                            Color.clear
                                .frame(width: 36, height: 36)
                        }
                        .padding(.horizontal, 20)
                        .padding(.top, 8)
                        
                        Spacer()
                        
                        // Bottom info
                        VStack(spacing: 12) {
                            // Instruction banner
                            HStack(spacing: 10) {
                                Image(systemName: "move.3d")
                                    .font(.system(size: 16))
                                Text("Move pattern around in front of camera")
                                    .font(.system(size: 13, weight: .medium))
                            }
                            .foregroundColor(.white)
                            .padding(.horizontal, 16)
                            .padding(.vertical, 10)
                            .background(Color.blue.opacity(0.9))
                            .clipShape(Capsule())
                            
                            // Tap hint
                            Text("Tap anywhere to hide controls")
                                .font(.system(size: 11))
                                .foregroundColor(.white.opacity(0.5))
                        }
                        .padding(.bottom, 40)
                    }
                    .transition(.opacity.combined(with: .scale(scale: 0.98)))
                }
            }
            .onTapGesture {
                withAnimation(.easeInOut(duration: 0.2)) {
                    showControls.toggle()
                }
                if showControls {
                    resetHideTimer()
                }
            }
        }
        .statusBarHidden(true)
        .onAppear {
            // Max brightness for best pattern detection
            brightness = UIScreen.main.brightness
            UIScreen.main.brightness = 1.0
            resetHideTimer()
        }
        .onDisappear {
            // Restore brightness
            UIScreen.main.brightness = brightness
            hideControlsTimer?.invalidate()
        }
    }
    
    private func resetHideTimer() {
        hideControlsTimer?.invalidate()
        hideControlsTimer = Timer.scheduledTimer(withTimeInterval: 4.0, repeats: false) { _ in
            withAnimation(.easeInOut(duration: 0.2)) {
                showControls = false
            }
        }
    }
}

/// Draws a checkerboard pattern
struct CheckerboardPatternView: View {
    let innerCornersX: Int
    let innerCornersY: Int
    let squareSizePoints: CGFloat
    
    var squaresX: Int { innerCornersX + 1 }
    var squaresY: Int { innerCornersY + 1 }
    
    var body: some View {
        VStack(spacing: 0) {
            ForEach(0..<squaresY, id: \.self) { row in
                HStack(spacing: 0) {
                    ForEach(0..<squaresX, id: \.self) { col in
                        Rectangle()
                            .fill((row + col) % 2 == 0 ? Color.white : Color.black)
                            .frame(width: squareSizePoints, height: squareSizePoints)
                    }
                }
            }
        }
    }
}

// MARK: - Screen Size Validation View

/// View for validating screen size using a credit card
struct ScreenSizeValidationView: View {
    @StateObject private var manager = CalibrationDisplayManager.shared
    @Environment(\.dismiss) private var dismiss
    
    @State private var cardWidth: CGFloat = 0
    @State private var isValidating = false
    
    private var expectedWidth: CGFloat {
        manager.mmToPoints(CreditCardDimensions.widthMM)
    }
    
    private var difference: CGFloat {
        abs(cardWidth - expectedWidth)
    }
    
    private var isAccurate: Bool {
        difference < 5
    }
    
    // After 90° rotation, width becomes height and vice versa
    private var rotatedCardHeight: CGFloat {
        cardWidth
    }
    
    private var rotatedCardWidth: CGFloat {
        cardWidth * CGFloat(CreditCardDimensions.heightMM / CreditCardDimensions.widthMM)
    }
    
    var body: some View {
        GeometryReader { geometry in
            ZStack {
                // Dark background
                Color(.systemBackground)
                    .ignoresSafeArea()
                
                VStack(spacing: 0) {
                    // Top bar - SAME PATTERN AS ARUCO
                    HStack(alignment: .top) {
                        // Close button
                        Button(action: { dismiss() }) {
                            Image(systemName: "xmark")
                                .font(.system(size: 16, weight: .bold))
                                .foregroundColor(.primary)
                                .frame(width: 36, height: 36)
                                .background(Color(.systemGray5))
                                .clipShape(Circle())
                        }
                        
                        Spacer()
                        
                        // Center title
                        VStack(spacing: 2) {
                            Text("Screen Validation")
                                .font(.system(size: 17, weight: .semibold))
                            Text("Match outline to credit card")
                                .font(.system(size: 12))
                                .foregroundColor(.secondary)
                        }
                        
                        Spacer()
                        
                        // Placeholder for symmetry
                        Color.clear
                            .frame(width: 36, height: 36)
                    }
                    .padding(.horizontal, 20)
                    .padding(.top, 16)
                    .padding(.bottom, 12)
                    
                    // Card display area
                    ZStack {
                        // Subtle grid background
                        GridPattern()
                            .stroke(Color.gray.opacity(0.15), lineWidth: 0.5)
                        
                        // Credit card outline (rotated 90°)
                        CreditCardOutlineView(width: $cardWidth, isAccurate: isAccurate)
                            .rotationEffect(.degrees(90))
                    }
                    .frame(maxWidth: geometry.size.width - 40)
                    .frame(height: max(rotatedCardHeight + 60, geometry.size.height * 0.45))
                    .background(Color(.systemGray6))
                    .clipShape(RoundedRectangle(cornerRadius: 16))
                    .padding(.horizontal, 20)
                    
                    // Rotation hint
                    HStack(spacing: 6) {
                        Image(systemName: "rotate.right")
                            .font(.system(size: 11))
                        Text("Card rotated 90°")
                            .font(.system(size: 11, weight: .medium))
                    }
                    .foregroundColor(.orange)
                    .padding(.horizontal, 10)
                    .padding(.vertical, 5)
                    .background(Color.orange.opacity(0.1))
                    .clipShape(Capsule())
                    .padding(.top, 12)
                    
                    Spacer()
                    
                    // Bottom controls
                    VStack(spacing: 16) {
                        // Compact stats row
                        HStack(spacing: 0) {
                            StatItem(label: "CURRENT", value: "\(Int(cardWidth))", unit: "pts", color: .primary)
                            
                            Rectangle()
                                .fill(Color.gray.opacity(0.3))
                                .frame(width: 1, height: 40)
                            
                            StatItem(label: "EXPECTED", value: "\(Int(expectedWidth))", unit: "pts", color: .secondary)
                            
                            Rectangle()
                                .fill(Color.gray.opacity(0.3))
                                .frame(width: 1, height: 40)
                            
                            VStack(spacing: 2) {
                                Image(systemName: isAccurate ? "checkmark.circle.fill" : "exclamationmark.circle.fill")
                                    .font(.system(size: 20))
                                    .foregroundColor(isAccurate ? .green : .orange)
                                Text(isAccurate ? "Good" : "Adjust")
                                    .font(.system(size: 10, weight: .medium))
                                    .foregroundColor(isAccurate ? .green : .orange)
                            }
                            .frame(maxWidth: .infinity)
                        }
                        .padding(.vertical, 12)
                        .background(Color(.systemGray6))
                        .clipShape(RoundedRectangle(cornerRadius: 12))
                        
                        // Slider
                        VStack(spacing: 6) {
                            Slider(value: $cardWidth, in: 200...400, step: 1)
                                .tint(.blue)
                            
                            HStack {
                                Text("Smaller")
                                    .font(.system(size: 10))
                                    .foregroundColor(.secondary)
                                Spacer()
                                Text("Larger")
                                    .font(.system(size: 10))
                                    .foregroundColor(.secondary)
                            }
                        }
                        
                        // Action buttons
                        VStack(spacing: 10) {
                            Button(action: {
                                withAnimation(.spring(response: 0.3)) {
                                    isValidating = true
                                }
                                DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                                    manager.completeDisplayValidation(measuredWidthPoints: cardWidth)
                                    dismiss()
                                }
                            }) {
                                HStack(spacing: 8) {
                                    if isValidating {
                                        ProgressView()
                                            .progressViewStyle(CircularProgressViewStyle(tint: .white))
                                    } else {
                                        Image(systemName: "checkmark.circle.fill")
                                    }
                                    Text(isValidating ? "Validating..." : "Confirm Size")
                                        .font(.system(size: 16, weight: .semibold))
                                }
                                .foregroundColor(.white)
                                .frame(maxWidth: .infinity)
                                .padding(.vertical, 14)
                                .background(Color.blue)
                                .clipShape(RoundedRectangle(cornerRadius: 12))
                            }
                            .disabled(isValidating)
                            
                            Button(action: {
                                withAnimation(.spring(response: 0.3)) {
                                    cardWidth = expectedWidth
                                }
                            }) {
                                Text("Reset to Default")
                                    .font(.system(size: 13, weight: .medium))
                                    .foregroundColor(.blue)
                            }
                        }
                    }
                    .padding(.horizontal, 20)
                    .padding(.bottom, 30)
                }
            }
        }
        .onAppear {
            cardWidth = expectedWidth
        }
    }
}

struct StatItem: View {
    let label: String
    let value: String
    let unit: String
    let color: Color
    
    var body: some View {
        VStack(spacing: 2) {
            Text(label)
                .font(.system(size: 9, weight: .semibold))
                .foregroundColor(.secondary)
            Text(value)
                .font(.system(size: 22, weight: .bold, design: .rounded))
                .foregroundColor(color)
            Text(unit)
                .font(.system(size: 9))
                .foregroundColor(.secondary)
        }
        .frame(maxWidth: .infinity)
    }
}

/// Grid pattern background
struct GridPattern: Shape {
    func path(in rect: CGRect) -> Path {
        var path = Path()
        let spacing: CGFloat = 20
        
        for x in stride(from: 0, through: rect.width, by: spacing) {
            path.move(to: CGPoint(x: x, y: 0))
            path.addLine(to: CGPoint(x: x, y: rect.height))
        }
        
        for y in stride(from: 0, through: rect.height, by: spacing) {
            path.move(to: CGPoint(x: 0, y: y))
            path.addLine(to: CGPoint(x: rect.width, y: y))
        }
        
        return path
    }
}

/// Adjustable credit card outline for validation
struct CreditCardOutlineView: View {
    @Binding var width: CGFloat
    var isAccurate: Bool
    
    var height: CGFloat {
        width * CGFloat(CreditCardDimensions.heightMM / CreditCardDimensions.widthMM)
    }
    
    var cornerRadius: CGFloat {
        width * CGFloat(CreditCardDimensions.cornerRadiusMM / CreditCardDimensions.widthMM)
    }
    
    var body: some View {
        ZStack {
            // Outer glow
            RoundedRectangle(cornerRadius: cornerRadius + 2)
                .stroke(isAccurate ? Color.green.opacity(0.3) : Color.blue.opacity(0.3), lineWidth: 8)
                .frame(width: width, height: height)
                .blur(radius: 4)
            
            // Main outline
            RoundedRectangle(cornerRadius: cornerRadius)
                .stroke(
                    LinearGradient(
                        colors: isAccurate ? [Color.green, Color.green.opacity(0.7)] : [Color.blue, Color.cyan],
                        startPoint: .topLeading,
                        endPoint: .bottomTrailing
                    ),
                    lineWidth: 3
                )
                .frame(width: width, height: height)
            
            // Corner indicators
            ForEach(0..<4, id: \.self) { corner in
                CornerIndicator(isAccurate: isAccurate)
                    .position(cornerPosition(for: corner))
            }
        }
        .frame(width: width, height: height)
    }
    
    private func cornerPosition(for corner: Int) -> CGPoint {
        let halfW = width / 2
        let halfH = height / 2
        
        switch corner {
        case 0: return CGPoint(x: halfW - halfW + 6, y: halfH - halfH + 6)
        case 1: return CGPoint(x: halfW + halfW - 6, y: halfH - halfH + 6)
        case 2: return CGPoint(x: halfW - halfW + 6, y: halfH + halfH - 6)
        case 3: return CGPoint(x: halfW + halfW - 6, y: halfH + halfH - 6)
        default: return .zero
        }
    }
}

struct CornerIndicator: View {
    var isAccurate: Bool
    
    var body: some View {
        ZStack {
            Circle()
                .fill(isAccurate ? Color.green : Color.blue)
                .frame(width: 12, height: 12)
            Circle()
                .fill(Color.white)
                .frame(width: 6, height: 6)
        }
    }
}

// MARK: - Calibration Results View

/// View showing calibration results from Vision Pro
struct CalibrationResultsView: View {
    @StateObject private var manager = CalibrationDisplayManager.shared
    @State private var selectedTab = 0
    @State private var isRefreshing = false
    
    var body: some View {
        VStack(spacing: 0) {
            // Dropdown picker instead of swipeable tabs
            HStack {
                Menu {
                    Button(action: { selectedTab = 0 }) {
                        Label("Extrinsic", systemImage: selectedTab == 0 ? "checkmark" : "")
                    }
                    Button(action: { selectedTab = 1 }) {
                        Label("Intrinsic", systemImage: selectedTab == 1 ? "checkmark" : "")
                    }
                } label: {
                    HStack(spacing: 8) {
                        Image(systemName: selectedTab == 0 ? "arrow.triangle.swap" : "camera.aperture")
                            .font(.system(size: 14, weight: .medium))
                        Text(selectedTab == 0 ? "Extrinsic" : "Intrinsic")
                            .font(.system(size: 15, weight: .semibold))
                        Image(systemName: "chevron.down")
                            .font(.system(size: 12, weight: .medium))
                    }
                    .foregroundColor(.primary)
                    .padding(.horizontal, 16)
                    .padding(.vertical, 10)
                    .background(Color(.secondarySystemBackground))
                    .clipShape(Capsule())
                }
                
                Spacer()
                
                // Result count badge
                let count = selectedTab == 0 ? manager.extrinsicResults.count : manager.intrinsicResults.count
                if count > 0 {
                    Text("\(count)")
                        .font(.system(size: 13, weight: .semibold))
                        .foregroundColor(.secondary)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 4)
                        .background(Color(.tertiarySystemBackground))
                        .clipShape(Capsule())
                }
            }
            .padding(.horizontal, 20)
            .padding(.top, 16)
            .padding(.bottom, 12)
            
            // Content without swipe gestures
            if selectedTab == 0 {
                ExtrinsicResultsList(results: manager.extrinsicResults)
            } else {
                IntrinsicResultsList(results: manager.intrinsicResults)
            }
        }
        .navigationTitle("Results")
        .navigationBarTitleDisplayMode(.inline)
        .toolbar {
            ToolbarItem(placement: .topBarTrailing) {
                Button(action: refreshFromiCloud) {
                    if isRefreshing {
                        ProgressView()
                    } else {
                        Image(systemName: "arrow.clockwise")
                    }
                }
                .disabled(isRefreshing)
            }
        }
        .refreshable {
            await refreshFromiCloudAsync()
        }
        .onAppear {
            // Sync from iCloud when view appears
            manager.refreshFromiCloud()
        }
    }
    
    private func refreshFromiCloud() {
        isRefreshing = true
        manager.refreshFromiCloud()
        DispatchQueue.main.asyncAfter(deadline: .now() + 1) {
            isRefreshing = false
        }
    }
    
    private func refreshFromiCloudAsync() async {
        manager.refreshFromiCloud()
        try? await Task.sleep(nanoseconds: 500_000_000)
    }
}

struct ExtrinsicResultsList: View {
    let results: [ExtrinsicCalibrationResult]
    
    var body: some View {
        if results.isEmpty {
            EmptyStateView(
                icon: "arrow.triangle.swap",
                title: "No Extrinsic Calibrations",
                subtitle: "Calibrate from Vision Pro to see results here"
            )
        } else {
            ScrollView {
                LazyVStack(spacing: 12) {
                    ForEach(results, id: \.cameraDeviceId) { result in
                        ExtrinsicResultCard(result: result)
                    }
                }
                .padding(.horizontal, 20)
                .padding(.vertical, 12)
            }
        }
    }
}

struct IntrinsicResultsList: View {
    let results: [IntrinsicCalibrationResult]
    
    var body: some View {
        if results.isEmpty {
            EmptyStateView(
                icon: "camera.aperture",
                title: "No Intrinsic Calibrations",
                subtitle: "Calibrate from Vision Pro to see results here"
            )
        } else {
            ScrollView {
                LazyVStack(spacing: 12) {
                    ForEach(results, id: \.cameraDeviceId) { result in
                        IntrinsicResultCard(result: result)
                    }
                }
                .padding(.horizontal, 20)
                .padding(.vertical, 12)
            }
        }
    }
}

struct EmptyStateView: View {
    let icon: String
    let title: String
    let subtitle: String
    
    var body: some View {
        VStack(spacing: 16) {
            Spacer()
            
            ZStack {
                Circle()
                    .fill(Color(.systemGray5))
                    .frame(width: 80, height: 80)
                
                Image(systemName: icon)
                    .font(.system(size: 32))
                    .foregroundColor(.secondary)
            }
            
            VStack(spacing: 6) {
                Text(title)
                    .font(.system(size: 17, weight: .semibold))
                
                Text(subtitle)
                    .font(.system(size: 14))
                    .foregroundColor(.secondary)
                    .multilineTextAlignment(.center)
            }
            
            Spacer()
        }
        .padding(40)
    }
}

struct ExtrinsicResultCard: View {
    let result: ExtrinsicCalibrationResult
    @State private var isExpanded = false
    @State private var show3DViewer = false
    
    var body: some View {
        VStack(spacing: 0) {
            // Header
            Button(action: { withAnimation(.spring(response: 0.3)) { isExpanded.toggle() } }) {
                HStack(spacing: 12) {
                    // Icon
                    ZStack {
                        Circle()
                            .fill(Color.purple.opacity(0.1))
                            .frame(width: 44, height: 44)
                        
                        Image(systemName: "video.fill")
                            .font(.system(size: 18))
                            .foregroundColor(.purple)
                    }
                    
                    VStack(alignment: .leading, spacing: 4) {
                        Text(result.cameraDeviceName)
                            .font(.system(size: 15, weight: .semibold))
                            .foregroundColor(.primary)
                        
                        HStack(spacing: 8) {
                            Label(result.isStereo ? "Stereo" : "Mono", systemImage: result.isStereo ? "eye.fill" : "circle.fill")
                                .font(.system(size: 11, weight: .medium))
                                .foregroundColor(result.isStereo ? .blue : .gray)
                            
                            Text("•")
                                .foregroundColor(.secondary)
                            
                            Text(result.calibrationDate, style: .date)
                                .font(.system(size: 11))
                                .foregroundColor(.secondary)
                        }
                    }
                    
                    Spacer()
                    
                    // Error badge
                    VStack(alignment: .trailing, spacing: 2) {
                        Text(String(format: "%.1fmm", result.reprojectionError * 1000))
                            .font(.system(size: 14, weight: .bold, design: .rounded))
                            .foregroundColor(result.reprojectionError < 0.01 ? .green : .orange)
                        Text("error")
                            .font(.system(size: 10))
                            .foregroundColor(.secondary)
                    }
                    
                    Image(systemName: "chevron.right")
                        .font(.system(size: 12, weight: .semibold))
                        .foregroundColor(.secondary)
                        .rotationEffect(.degrees(isExpanded ? 90 : 0))
                }
                .padding(16)
            }
            .buttonStyle(.plain)
            
            // Expanded content
            if isExpanded {
                Divider()
                    .padding(.horizontal, 16)
                
                VStack(alignment: .leading, spacing: 16) {
                    // 3D Visualization - tap to open full screen
                    VStack(alignment: .leading, spacing: 8) {
                        HStack {
                            Text("Camera Pose (relative to Head)")
                                .font(.system(size: 12, weight: .semibold))
                                .foregroundColor(.secondary)
                            Spacer()
                            if result.isStereo {
                                HStack(spacing: 4) {
                                    Circle().fill(Color.purple).frame(width: 6, height: 6)
                                    Text("L")
                                    Circle().fill(Color.cyan).frame(width: 6, height: 6)
                                    Text("R")
                                }
                                .font(.system(size: 9, weight: .medium))
                                .foregroundColor(.secondary)
                            }
                        }
                        
                        // Tap to open full-screen 3D viewer
                        Button(action: { show3DViewer = true }) {
                            ZStack {
                                Extrinsic3DVisualization(result: result)
                                    .frame(height: 180)
                                    .allowsHitTesting(false)  // Disable gestures in inline view
                                
                                // Overlay hint
                                VStack {
                                    Spacer()
                                    HStack {
                                        Spacer()
                                        HStack(spacing: 4) {
                                            Image(systemName: "arrow.up.left.and.arrow.down.right")
                                                .font(.system(size: 10, weight: .medium))
                                            Text("Tap to interact")
                                                .font(.system(size: 10, weight: .medium))
                                        }
                                        .foregroundColor(.white.opacity(0.8))
                                        .padding(.horizontal, 8)
                                        .padding(.vertical, 4)
                                        .background(Color.black.opacity(0.5))
                                        .clipShape(Capsule())
                                        .padding(8)
                                    }
                                }
                            }
                            .background(Color.black.opacity(0.3))
                            .clipShape(RoundedRectangle(cornerRadius: 12))
                        }
                        .buttonStyle(.plain)
                    }
                    
                    // Left/Mono 4x4 Transform Matrix
                    VStack(alignment: .leading, spacing: 8) {
                        HStack {
                            Text(result.isStereo ? "Left Camera Transform (4×4)" : "Head → Camera Transform (4×4)")
                                .font(.system(size: 12, weight: .semibold))
                                .foregroundColor(.secondary)
                            if result.isStereo {
                                Spacer()
                                Text("Purple")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundColor(.purple)
                            }
                        }
                        
                        TransformMatrixView(matrix: result.headToCamera, accentColor: .purple)
                        
                        // Reproj error for left
                        Text(String(format: "Reproj: %.4fm • %d samples", result.reprojectionError, result.sampleCount))
                            .font(.system(size: 10))
                            .foregroundColor(result.reprojectionError < 0.01 ? .green : .orange)
                    }
                    
                    // Right 4x4 Transform Matrix (stereo only)
                    if result.isStereo, let rightMatrix = result.rightHeadToCamera {
                        VStack(alignment: .leading, spacing: 8) {
                            HStack {
                                Text("Right Camera Transform (4×4)")
                                    .font(.system(size: 12, weight: .semibold))
                                    .foregroundColor(.secondary)
                                Spacer()
                                Text("Cyan")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundColor(.cyan)
                            }
                            
                            TransformMatrixView(matrix: rightMatrix, accentColor: .cyan)
                            
                            // Reproj error for right
                            if let rightError = result.rightReprojectionError, let rightCount = result.rightSampleCount {
                                Text(String(format: "Reproj: %.4fm • %d samples", rightError, rightCount))
                                    .font(.system(size: 10))
                                    .foregroundColor(rightError < 0.01 ? .green : .orange)
                            }
                        }
                    }
                    
                    // Translation summary (left/mono only for simplicity)
                    VStack(alignment: .leading, spacing: 8) {
                        Text(result.isStereo ? "Left Translation (cm)" : "Translation (cm)")
                            .font(.system(size: 12, weight: .semibold))
                            .foregroundColor(.secondary)
                        
                        HStack(spacing: 12) {
                            TranslationBadge(axis: "X", value: result.translation.x, color: .red)
                            TranslationBadge(axis: "Y", value: result.translation.y, color: .green)
                            TranslationBadge(axis: "Z", value: result.translation.z, color: .blue)
                        }
                    }
                    
                    // Rotation (Euler angles)
                    VStack(alignment: .leading, spacing: 8) {
                        Text(result.isStereo ? "Left Rotation (Euler °)" : "Rotation (Euler °)")
                            .font(.system(size: 12, weight: .semibold))
                            .foregroundColor(.secondary)
                        
                        HStack(spacing: 12) {
                            RotationBadge(axis: "Pitch", value: result.eulerAngles.pitch, color: .red)
                            RotationBadge(axis: "Yaw", value: result.eulerAngles.yaw, color: .green)
                            RotationBadge(axis: "Roll", value: result.eulerAngles.roll, color: .blue)
                        }
                    }
                }
                .padding(16)
                .transition(.opacity.combined(with: .move(edge: .top)))
            }
        }
        .background(Color(.secondarySystemBackground))
        .clipShape(RoundedRectangle(cornerRadius: 16))
        .sheet(isPresented: $show3DViewer) {
            Extrinsic3DViewerSheet(result: result)
        }
    }
}

// MARK: - Full-Screen 3D Viewer Sheet

struct Extrinsic3DViewerSheet: View {
    let result: ExtrinsicCalibrationResult
    @Environment(\.dismiss) private var dismiss
    
    var body: some View {
        NavigationStack {
            ZStack {
                Color.black.ignoresSafeArea()
                
                VStack(spacing: 0) {
                    // Legend for stereo
                    if result.isStereo {
                        HStack(spacing: 16) {
                            HStack(spacing: 6) {
                                Circle().fill(Color.gray).frame(width: 8, height: 8)
                                Text("Head")
                            }
                            HStack(spacing: 6) {
                                Circle().fill(Color.purple).frame(width: 8, height: 8)
                                Text("Left Camera")
                            }
                            HStack(spacing: 6) {
                                Circle().fill(Color.cyan).frame(width: 8, height: 8)
                                Text("Right Camera")
                            }
                        }
                        .font(.system(size: 11, weight: .medium))
                        .foregroundColor(.white.opacity(0.7))
                        .padding(.vertical, 12)
                    }
                    
                    // Full-screen 3D visualization with gestures enabled
                    Extrinsic3DVisualization(result: result)
                        .padding(.horizontal, 8)
                }
            }
            .navigationTitle(result.cameraDeviceName)
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button("Done") { dismiss() }
                }
            }
            .toolbarBackground(.visible, for: .navigationBar)
            .toolbarColorScheme(.dark, for: .navigationBar)
        }
        .preferredColorScheme(.dark)
    }
}

// MARK: - Translation Badge

struct TranslationBadge: View {
    let axis: String
    let value: Double
    let color: Color
    
    var body: some View {
        VStack(spacing: 4) {
            Text(axis)
                .font(.system(size: 10, weight: .bold))
                .foregroundColor(color)
            Text(String(format: "%.1f", value * 100))
                .font(.system(size: 16, weight: .bold, design: .rounded))
                .foregroundColor(.primary)
            Text("cm")
                .font(.system(size: 9))
                .foregroundColor(.secondary)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 10)
        .background(color.opacity(0.1))
        .clipShape(RoundedRectangle(cornerRadius: 10))
    }
}

// MARK: - Rotation Badge

struct RotationBadge: View {
    let axis: String
    let value: Double
    let color: Color
    
    var body: some View {
        VStack(spacing: 4) {
            Text(axis)
                .font(.system(size: 10, weight: .bold))
                .foregroundColor(color)
            Text(String(format: "%.1f", value))
                .font(.system(size: 16, weight: .bold, design: .rounded))
                .foregroundColor(.primary)
            Text("°")
                .font(.system(size: 9))
                .foregroundColor(.secondary)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 10)
        .background(color.opacity(0.1))
        .clipShape(RoundedRectangle(cornerRadius: 10))
    }
}

// MARK: - Transform Matrix View (4x4)

struct TransformMatrixView: View {
    let matrix: [Double]
    var accentColor: Color = .purple
    
    var body: some View {
        VStack(spacing: 2) {
            if matrix.count == 16 {
                // Display as column-major (OpenGL/Metal convention)
                // Row 0: [m0, m4, m8, m12]
                // Row 1: [m1, m5, m9, m13]
                // Row 2: [m2, m6, m10, m14]
                // Row 3: [m3, m7, m11, m15]
                ForEach(0..<4, id: \.self) { row in
                    HStack(spacing: 4) {
                        Text("[")
                            .font(.system(size: 10, design: .monospaced))
                            .foregroundColor(.secondary)
                        ForEach(0..<4, id: \.self) { col in
                            let idx = col * 4 + row  // column-major
                            let value = matrix[idx]
                            Text(formatMatrixValue(value))
                                .font(.system(size: 10, weight: .medium, design: .monospaced))
                                .foregroundColor(matrixCellColor(row: row, col: col, value: value))
                                .frame(width: 60, alignment: .trailing)
                        }
                        Text("]")
                            .font(.system(size: 10, design: .monospaced))
                            .foregroundColor(.secondary)
                    }
                }
            } else {
                Text("Invalid matrix (\(matrix.count) elements)")
                    .font(.system(size: 10))
                    .foregroundColor(.red)
            }
        }
        .padding(12)
        .background(accentColor.opacity(0.05))
        .clipShape(RoundedRectangle(cornerRadius: 8))
    }
    
    private func formatMatrixValue(_ value: Double) -> String {
        if abs(value) < 0.0001 {
            return "0.000"
        } else if abs(value - 1.0) < 0.0001 {
            return "1.000"
        } else {
            return String(format: "%+.3f", value)
        }
    }
    
    private func matrixCellColor(row: Int, col: Int, value: Double) -> Color {
        // Rotation part (top-left 3x3)
        if row < 3 && col < 3 {
            return accentColor
        }
        // Translation part (rightmost column, rows 0-2)
        if col == 3 && row < 3 {
            return .blue
        }
        // Bottom row
        return .secondary
    }
}

// MARK: - Camera Intrinsic Matrix View (3x3)

struct CameraIntrinsicMatrixView: View {
    let result: IntrinsicCalibrationResult
    
    var body: some View {
        VStack(spacing: 2) {
            // Row 0: [fx, 0, cx]
            HStack(spacing: 4) {
                Text("[")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundColor(.secondary)
                Text(String(format: "%.1f", result.fx))
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.red)
                    .frame(width: 70, alignment: .trailing)
                Text("0.0")
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.secondary)
                    .frame(width: 70, alignment: .trailing)
                Text(String(format: "%.1f", result.cx))
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.blue)
                    .frame(width: 70, alignment: .trailing)
                Text("]")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundColor(.secondary)
            }
            
            // Row 1: [0, fy, cy]
            HStack(spacing: 4) {
                Text("[")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundColor(.secondary)
                Text("0.0")
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.secondary)
                    .frame(width: 70, alignment: .trailing)
                Text(String(format: "%.1f", result.fy))
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.green)
                    .frame(width: 70, alignment: .trailing)
                Text(String(format: "%.1f", result.cy))
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.purple)
                    .frame(width: 70, alignment: .trailing)
                Text("]")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundColor(.secondary)
            }
            
            // Row 2: [0, 0, 1]
            HStack(spacing: 4) {
                Text("[")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundColor(.secondary)
                Text("0.0")
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.secondary)
                    .frame(width: 70, alignment: .trailing)
                Text("0.0")
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.secondary)
                    .frame(width: 70, alignment: .trailing)
                Text("1.0")
                    .font(.system(size: 10, weight: .medium, design: .monospaced))
                    .foregroundColor(.secondary)
                    .frame(width: 70, alignment: .trailing)
                Text("]")
                    .font(.system(size: 10, design: .monospaced))
                    .foregroundColor(.secondary)
            }
        }
        .padding(12)
        .background(Color.orange.opacity(0.05))
        .clipShape(RoundedRectangle(cornerRadius: 8))
    }
}

// MARK: - Distortion Coefficients View

struct DistortionCoefficientsView: View {
    let coeffs: [Double]
    
    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            // Coefficient labels
            HStack(spacing: 4) {
                ForEach(Array(coeffNames.prefix(coeffs.count).enumerated()), id: \.offset) { idx, name in
                    VStack(spacing: 2) {
                        Text(name)
                            .font(.system(size: 9, weight: .bold, design: .monospaced))
                            .foregroundColor(.secondary)
                        Text(String(format: "%.4f", coeffs[idx]))
                            .font(.system(size: 9, weight: .medium, design: .monospaced))
                            .foregroundColor(.orange)
                    }
                    .frame(maxWidth: .infinity)
                }
            }
        }
        .padding(10)
        .background(Color.orange.opacity(0.05))
        .clipShape(RoundedRectangle(cornerRadius: 8))
    }
    
    private var coeffNames: [String] {
        ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
    }
}

// MARK: - Intrinsic Badge

struct IntrinsicBadge: View {
    let label: String
    let value: Double
    let unit: String
    let color: Color
    
    var body: some View {
        VStack(spacing: 2) {
            Text(label)
                .font(.system(size: 9, weight: .bold))
                .foregroundColor(color)
            Text(String(format: "%.1f", value))
                .font(.system(size: 12, weight: .bold, design: .rounded))
                .foregroundColor(.primary)
                .lineLimit(1)
                .minimumScaleFactor(0.6)
            Text(unit)
                .font(.system(size: 8))
                .foregroundColor(.secondary)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 8)
        .background(color.opacity(0.1))
        .clipShape(RoundedRectangle(cornerRadius: 8))
    }
}

// MARK: - 3D Extrinsic Visualization (Canvas-based)

/// A 2D canvas that draws a 3D visualization of the camera pose relative to the head frame
struct Extrinsic3DVisualization: View {
    let result: ExtrinsicCalibrationResult
    
    // View rotation state (radians)
    @State private var viewAngleX: Float = -0.4  // tilt down
    @State private var viewAngleY: Float = 0.3   // rotate right
    @State private var lastDragLocation: CGPoint? = nil
    
    // Zoom state
    @State private var zoomScale: CGFloat = 1.0
    @State private var lastZoomScale: CGFloat = 1.0
    
    // Pan offset state
    @State private var panOffset: CGSize = .zero
    @State private var lastPanOffset: CGSize = .zero
    
    // Convert column-major [Double] to a usable format
    private func arrayToMatrix(_ array: [Double]) -> [[Float]] {
        guard array.count == 16 else {
            return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        }
        // Column-major: columns[col][row] = array[col*4 + row]
        var m: [[Float]] = Array(repeating: Array(repeating: 0, count: 4), count: 4)
        for col in 0..<4 {
            for row in 0..<4 {
                m[col][row] = Float(array[col * 4 + row])
            }
        }
        return m
    }
    
    private var leftTransformMatrix: [[Float]] {
        arrayToMatrix(result.headToCamera)
    }
    
    private var rightTransformMatrix: [[Float]]? {
        guard let rightMatrix = result.rightHeadToCamera else { return nil }
        return arrayToMatrix(rightMatrix)
    }
    
    var body: some View {
        Canvas { context, size in
            // Apply pan offset to center
            let center = CGPoint(
                x: size.width / 2 + panOffset.width,
                y: size.height / 2 + panOffset.height
            )
            let baseScale: CGFloat = 400  // pixels per meter
            let scale = baseScale * zoomScale
            
            // Draw grid on XZ plane (draw first so it's behind)
            drawGrid(context: context, center: center, scale: scale,
                    viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            
            // Draw head frame at origin (coordinate axes)
            let identity: [[Float]] = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
            drawCoordinateAxes(context: context, center: center, scale: scale,
                             transform: identity,
                             viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                             label: "Head", axisLength: 0.04, labelColor: .gray)
            
            // Get left/mono camera pose in head frame (inverse of headToCamera)
            let leftCameraInHead = invertMatrix(leftTransformMatrix)
            drawCoordinateAxes(context: context, center: center, scale: scale,
                             transform: leftCameraInHead,
                             viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                             label: result.isStereo ? "L" : "Cam", axisLength: 0.025, labelColor: .purple)
            
            // Draw camera frustum for left camera
            drawCameraFrustum(context: context, center: center, scale: scale,
                            transform: leftCameraInHead,
                            viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                            color: .purple.opacity(0.6))
            
            // Draw right camera if stereo
            if result.isStereo, let rightMatrix = rightTransformMatrix {
                let rightCameraInHead = invertMatrix(rightMatrix)
                drawCoordinateAxes(context: context, center: center, scale: scale,
                                 transform: rightCameraInHead,
                                 viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                                 label: "R", axisLength: 0.025, labelColor: .cyan)
                
                drawCameraFrustum(context: context, center: center, scale: scale,
                                transform: rightCameraInHead,
                                viewAngleX: viewAngleX, viewAngleY: viewAngleY,
                                color: .cyan.opacity(0.6))
            }
            
            // Draw hints
            let zoomText = String(format: "%.1fx", zoomScale)
            context.draw(Text(zoomText).font(.system(size: 8, weight: .medium)).foregroundColor(.white.opacity(0.5)),
                        at: CGPoint(x: 20, y: size.height - 10), anchor: .leading)
            
            let hintText = "1-finger: rotate • 2-finger: pan/zoom"
            context.draw(Text(hintText).font(.system(size: 7)).foregroundColor(.white.opacity(0.3)),
                        at: CGPoint(x: size.width / 2, y: size.height - 10), anchor: .center)
        }
        .gesture(
            DragGesture()
                .onChanged { value in
                    if let last = lastDragLocation {
                        let deltaX = Float(value.location.x - last.x) * 0.01
                        let deltaY = Float(value.location.y - last.y) * 0.01
                        viewAngleY += deltaX
                        viewAngleX -= deltaY
                        // Clamp vertical angle to avoid flipping
                        viewAngleX = max(-Float.pi / 2 + 0.1, min(Float.pi / 2 - 0.1, viewAngleX))
                    }
                    lastDragLocation = value.location
                }
                .onEnded { _ in
                    lastDragLocation = nil
                }
        )
        .simultaneousGesture(
            MagnificationGesture()
                .onChanged { value in
                    let newScale = lastZoomScale * value
                    zoomScale = min(max(newScale, 0.3), 5.0)  // Clamp between 0.3x and 5x
                }
                .onEnded { _ in
                    lastZoomScale = zoomScale
                }
        )
        .simultaneousGesture(
            DragGesture(minimumDistance: 10)
                .simultaneously(with: MagnificationGesture())
                .onChanged { value in
                    // When pinching (two fingers), also allow panning
                    if let dragValue = value.first {
                        panOffset = CGSize(
                            width: lastPanOffset.width + dragValue.translation.width,
                            height: lastPanOffset.height + dragValue.translation.height
                        )
                    }
                }
                .onEnded { _ in
                    lastPanOffset = panOffset
                }
        )
        .simultaneousGesture(
            TapGesture(count: 2)
                .onEnded {
                    // Double-tap to reset view
                    withAnimation(.spring(response: 0.3)) {
                        viewAngleX = -0.4
                        viewAngleY = 0.3
                        zoomScale = 1.0
                        lastZoomScale = 1.0
                        panOffset = .zero
                        lastPanOffset = .zero
                    }
                }
        )
    }
    
    /// Simple 4x4 matrix inversion (for rigid transforms)
    private func invertMatrix(_ m: [[Float]]) -> [[Float]] {
        // For a rigid transform, inverse is: R^T | -R^T * t
        // Extract rotation (3x3)
        let r00 = m[0][0], r01 = m[1][0], r02 = m[2][0]
        let r10 = m[0][1], r11 = m[1][1], r12 = m[2][1]
        let r20 = m[0][2], r21 = m[1][2], r22 = m[2][2]
        
        // Extract translation
        let tx = m[3][0], ty = m[3][1], tz = m[3][2]
        
        // R^T
        let rt00 = r00, rt01 = r10, rt02 = r20
        let rt10 = r01, rt11 = r11, rt12 = r21
        let rt20 = r02, rt21 = r12, rt22 = r22
        
        // -R^T * t
        let ntx = -(rt00 * tx + rt01 * ty + rt02 * tz)
        let nty = -(rt10 * tx + rt11 * ty + rt12 * tz)
        let ntz = -(rt20 * tx + rt21 * ty + rt22 * tz)
        
        return [
            [rt00, rt10, rt20, 0],
            [rt01, rt11, rt21, 0],
            [rt02, rt12, rt22, 0],
            [ntx, nty, ntz, 1]
        ]
    }
    
    /// Project a 3D point to 2D screen coordinates
    private func project3DTo2D(point: (x: Float, y: Float, z: Float), center: CGPoint, scale: CGFloat,
                               viewAngleX: Float, viewAngleY: Float) -> CGPoint {
        // Convert from ARKit convention to display convention (negate Z)
        let displayPoint = (x: point.x, y: point.y, z: -point.z)
        
        // Apply view rotation
        let cosX = cos(viewAngleX)
        let sinX = sin(viewAngleX)
        let cosY = cos(viewAngleY)
        let sinY = sin(viewAngleY)
        
        // Rotate around Y axis
        let x1 = displayPoint.x * cosY - displayPoint.z * sinY
        let z1 = displayPoint.x * sinY + displayPoint.z * cosY
        let y1 = displayPoint.y
        
        // Rotate around X axis
        let y2 = y1 * cosX - z1 * sinX
        let x2 = x1
        
        // Orthographic projection
        let screenX = center.x + CGFloat(x2) * scale
        let screenY = center.y - CGFloat(y2) * scale
        
        return CGPoint(x: screenX, y: screenY)
    }
    
    /// Draw coordinate axes at a given transform
    private func drawCoordinateAxes(context: GraphicsContext, center: CGPoint, scale: CGFloat,
                                    transform: [[Float]], viewAngleX: Float, viewAngleY: Float,
                                    label: String, axisLength: Float, labelColor: Color) {
        let origin = (x: transform[3][0], y: transform[3][1], z: transform[3][2])
        
        // Extract rotation axes from transform (column vectors)
        let xAxis = (x: transform[0][0], y: transform[0][1], z: transform[0][2])
        let yAxis = (x: transform[1][0], y: transform[1][1], z: transform[1][2])
        let zAxis = (x: transform[2][0], y: transform[2][1], z: transform[2][2])
        
        let originScreen = project3DTo2D(point: origin, center: center, scale: scale,
                                         viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        
        // Draw X axis (red)
        let xEnd = (x: origin.x + xAxis.x * axisLength, y: origin.y + xAxis.y * axisLength, z: origin.z + xAxis.z * axisLength)
        let xEndScreen = project3DTo2D(point: xEnd, center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        var xPath = Path()
        xPath.move(to: originScreen)
        xPath.addLine(to: xEndScreen)
        context.stroke(xPath, with: .color(.red), lineWidth: 2)
        
        // Draw Y axis (green)
        let yEnd = (x: origin.x + yAxis.x * axisLength, y: origin.y + yAxis.y * axisLength, z: origin.z + yAxis.z * axisLength)
        let yEndScreen = project3DTo2D(point: yEnd, center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        var yPath = Path()
        yPath.move(to: originScreen)
        yPath.addLine(to: yEndScreen)
        context.stroke(yPath, with: .color(.green), lineWidth: 2)
        
        // Draw Z axis (blue)
        let zEnd = (x: origin.x + zAxis.x * axisLength, y: origin.y + zAxis.y * axisLength, z: origin.z + zAxis.z * axisLength)
        let zEndScreen = project3DTo2D(point: zEnd, center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        var zPath = Path()
        zPath.move(to: originScreen)
        zPath.addLine(to: zEndScreen)
        context.stroke(zPath, with: .color(.blue), lineWidth: 2)
        
        // Draw label
        let labelPoint = CGPoint(x: originScreen.x + 5, y: originScreen.y - 10)
        context.draw(Text(label).font(.system(size: 9, weight: .bold)).foregroundColor(labelColor),
                    at: labelPoint, anchor: .leading)
    }
    
    /// Draw a simple camera frustum
    private func drawCameraFrustum(context: GraphicsContext, center: CGPoint, scale: CGFloat,
                                   transform: [[Float]], viewAngleX: Float, viewAngleY: Float,
                                   color: Color) {
        let origin = (x: transform[3][0], y: transform[3][1], z: transform[3][2])
        let zAxis = (x: transform[2][0], y: transform[2][1], z: transform[2][2])
        let xAxis = (x: transform[0][0], y: transform[0][1], z: transform[0][2])
        let yAxis = (x: transform[1][0], y: transform[1][1], z: transform[1][2])
        
        let frustumDepth: Float = 0.03
        let frustumWidth: Float = 0.015
        let frustumHeight: Float = 0.01
        
        // Frustum corners
        let corners: [(x: Float, y: Float, z: Float)] = [
            (origin.x + zAxis.x * frustumDepth + xAxis.x * frustumWidth + yAxis.x * frustumHeight,
             origin.y + zAxis.y * frustumDepth + xAxis.y * frustumWidth + yAxis.y * frustumHeight,
             origin.z + zAxis.z * frustumDepth + xAxis.z * frustumWidth + yAxis.z * frustumHeight),
            (origin.x + zAxis.x * frustumDepth - xAxis.x * frustumWidth + yAxis.x * frustumHeight,
             origin.y + zAxis.y * frustumDepth - xAxis.y * frustumWidth + yAxis.y * frustumHeight,
             origin.z + zAxis.z * frustumDepth - xAxis.z * frustumWidth + yAxis.z * frustumHeight),
            (origin.x + zAxis.x * frustumDepth - xAxis.x * frustumWidth - yAxis.x * frustumHeight,
             origin.y + zAxis.y * frustumDepth - xAxis.y * frustumWidth - yAxis.y * frustumHeight,
             origin.z + zAxis.z * frustumDepth - xAxis.z * frustumWidth - yAxis.z * frustumHeight),
            (origin.x + zAxis.x * frustumDepth + xAxis.x * frustumWidth - yAxis.x * frustumHeight,
             origin.y + zAxis.y * frustumDepth + xAxis.y * frustumWidth - yAxis.y * frustumHeight,
             origin.z + zAxis.z * frustumDepth + xAxis.z * frustumWidth - yAxis.z * frustumHeight)
        ]
        
        let originScreen = project3DTo2D(point: origin, center: center, scale: scale,
                                         viewAngleX: viewAngleX, viewAngleY: viewAngleY)
        let cornerScreens = corners.map { project3DTo2D(point: $0, center: center, scale: scale,
                                                        viewAngleX: viewAngleX, viewAngleY: viewAngleY) }
        
        // Draw frustum edges from origin to corners
        for cornerScreen in cornerScreens {
            var edgePath = Path()
            edgePath.move(to: originScreen)
            edgePath.addLine(to: cornerScreen)
            context.stroke(edgePath, with: .color(color), lineWidth: 1)
        }
        
        // Draw frustum rectangle
        var rectPath = Path()
        rectPath.move(to: cornerScreens[0])
        for i in 1..<cornerScreens.count {
            rectPath.addLine(to: cornerScreens[i])
        }
        rectPath.closeSubpath()
        context.stroke(rectPath, with: .color(color), lineWidth: 1)
    }
    
    /// Draw a reference grid on the XZ plane
    private func drawGrid(context: GraphicsContext, center: CGPoint, scale: CGFloat,
                         viewAngleX: Float, viewAngleY: Float) {
        let gridSize: Float = 0.08  // 8cm
        let gridLines = 4
        
        for i in -gridLines...gridLines {
            let offset = Float(i) * gridSize / Float(gridLines)
            
            // Lines parallel to X
            let xStart = project3DTo2D(point: (x: -gridSize, y: 0, z: offset), center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            let xEnd = project3DTo2D(point: (x: gridSize, y: 0, z: offset), center: center, scale: scale,
                                     viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            var xPath = Path()
            xPath.move(to: xStart)
            xPath.addLine(to: xEnd)
            context.stroke(xPath, with: .color(.white.opacity(0.15)), lineWidth: 0.5)
            
            // Lines parallel to Z
            let zStart = project3DTo2D(point: (x: offset, y: 0, z: -gridSize), center: center, scale: scale,
                                       viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            let zEnd = project3DTo2D(point: (x: offset, y: 0, z: gridSize), center: center, scale: scale,
                                     viewAngleX: viewAngleX, viewAngleY: viewAngleY)
            var zPath = Path()
            zPath.move(to: zStart)
            zPath.addLine(to: zEnd)
            context.stroke(zPath, with: .color(.white.opacity(0.15)), lineWidth: 0.5)
        }
    }
}

struct IntrinsicResultCard: View {
    let result: IntrinsicCalibrationResult
    @State private var isExpanded = false
    
    var body: some View {
        VStack(spacing: 0) {
            // Header
            Button(action: { withAnimation(.spring(response: 0.3)) { isExpanded.toggle() } }) {
                HStack(spacing: 12) {
                    // Icon
                    ZStack {
                        Circle()
                            .fill(Color.orange.opacity(0.1))
                            .frame(width: 44, height: 44)
                        
                        Image(systemName: "camera.aperture")
                            .font(.system(size: 18))
                            .foregroundColor(.orange)
                    }
                    
                    VStack(alignment: .leading, spacing: 4) {
                        Text(result.cameraDeviceName)
                            .font(.system(size: 15, weight: .semibold))
                            .foregroundColor(.primary)
                        
                        HStack(spacing: 8) {
                            Text("\(result.imageWidth)×\(result.imageHeight)")
                                .font(.system(size: 11, weight: .medium))
                                .foregroundColor(.blue)
                            
                            Text("•")
                                .foregroundColor(.secondary)
                            
                            Text(result.calibrationDate, style: .date)
                                .font(.system(size: 11))
                                .foregroundColor(.secondary)
                        }
                    }
                    
                    Spacer()
                    
                    // Error badge
                    VStack(alignment: .trailing, spacing: 2) {
                        Text(String(format: "%.2fpx", result.reprojectionError))
                            .font(.system(size: 14, weight: .bold, design: .rounded))
                            .foregroundColor(result.reprojectionError < 0.5 ? .green : .orange)
                        Text("reproj")
                            .font(.system(size: 10))
                            .foregroundColor(.secondary)
                    }
                    
                    Image(systemName: "chevron.right")
                        .font(.system(size: 12, weight: .semibold))
                        .foregroundColor(.secondary)
                        .rotationEffect(.degrees(isExpanded ? 90 : 0))
                }
                .padding(16)
            }
            .buttonStyle(.plain)
            
            // Expanded content
            if isExpanded {
                Divider()
                    .padding(.horizontal, 16)
                
                VStack(alignment: .leading, spacing: 16) {
                    // Camera Intrinsic Matrix (3x3)
                    VStack(alignment: .leading, spacing: 8) {
                        Text("Camera Matrix (K)")
                            .font(.system(size: 12, weight: .semibold))
                            .foregroundColor(.secondary)
                        
                        CameraIntrinsicMatrixView(result: result)
                    }
                    
                    // Distortion Coefficients
                    VStack(alignment: .leading, spacing: 8) {
                        Text("Distortion Coefficients")
                            .font(.system(size: 12, weight: .semibold))
                            .foregroundColor(.secondary)
                        
                        DistortionCoefficientsView(coeffs: result.distortionCoeffs)
                    }
                    
                    // Parameter summary
                    VStack(alignment: .leading, spacing: 8) {
                        Text("Principal Point & Focal Length")
                            .font(.system(size: 12, weight: .semibold))
                            .foregroundColor(.secondary)
                        
                        HStack(spacing: 8) {
                            IntrinsicBadge(label: "fx", value: result.fx, unit: "px", color: .red)
                            IntrinsicBadge(label: "fy", value: result.fy, unit: "px", color: .green)
                            IntrinsicBadge(label: "cx", value: result.cx, unit: "px", color: .blue)
                            IntrinsicBadge(label: "cy", value: result.cy, unit: "px", color: .purple)
                        }
                    }
                }
                .padding(16)
                .transition(.opacity.combined(with: .move(edge: .top)))
            }
        }
        .background(Color(.secondarySystemBackground))
        .clipShape(RoundedRectangle(cornerRadius: 16))
    }
}

// MARK: - Main Calibration Tab View

/// Main calibration tab that integrates all calibration features
struct CalibrationTabView: View {
    @StateObject private var manager = CalibrationDisplayManager.shared
    @StateObject private var multipeerManager = MultipeerCalibrationManager_iOS.shared
    
    @State private var showArucoDisplay = false
    @State private var showCheckerboardDisplay = false
    @State private var showScreenValidation = false
    @State private var showCalibrationCoordinator = false
    
    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 20) {
                    // Vision Pro Guided Calibration - Primary option
                    VisionProCalibrationCard(
                        isConnected: multipeerManager.isConnected,
                        connectionStatus: multipeerManager.connectionStatus,
                        action: { showCalibrationCoordinator = true }
                    )
                    
                    // Divider with "OR" text
                    HStack {
                        Rectangle()
                            .fill(Color.gray.opacity(0.3))
                            .frame(height: 1)
                        Text("OR")
                            .font(.system(size: 12, weight: .medium))
                            .foregroundColor(.secondary)
                            .padding(.horizontal, 12)
                        Rectangle()
                            .fill(Color.gray.opacity(0.3))
                            .frame(height: 1)
                    }
                    .padding(.horizontal, 8)
                    
                    // Device card
                    DeviceInfoCard(
                        displaySpec: manager.displaySpec,
                        isValidated: manager.isDisplayValidated,
                        onValidate: { showScreenValidation = true }
                    )
                    
                    // Manual calibration options header
                    HStack {
                        Text("Manual Display")
                            .font(.system(size: 14, weight: .semibold))
                            .foregroundColor(.secondary)
                        Spacer()
                    }
                    .padding(.horizontal, 4)
                    
                    // Calibration options
                    VStack(spacing: 12) {
                        // Extrinsic (ArUco)
                        CalibrationOptionCard(
                            title: "Extrinsic Calibration",
                            subtitle: "Camera-to-headset transform",
                            icon: "qrcode",
                            iconColor: .purple,
                            details: [
                                ("Marker Size", "\(Int(manager.arucoConfig.markerSizeMM))mm"),
                                ("Dictionary", manager.arucoConfig.dictionary.displayName),
                                ("Positions", "\(manager.arucoConfig.minUniquePositions)+")
                            ],
                            warningText: manager.markerFitsOnScreen ? nil : "Marker too large for screen",
                            actionTitle: "Display ArUco Marker",
                            isEnabled: manager.markerFitsOnScreen,
                            action: { showArucoDisplay = true }
                        )
                        
                        // Intrinsic (Checkerboard)
                        CalibrationOptionCard(
                            title: "Intrinsic Calibration",
                            subtitle: "Camera focal length & distortion",
                            icon: "checkerboard.rectangle",
                            iconColor: .orange,
                            details: [
                                ("Pattern", "\(manager.checkerboardConfig.innerCornersX + 1)×\(manager.checkerboardConfig.innerCornersY + 1)"),
                                ("Square", "\(Int(manager.checkerboardConfig.squareSizeMM))mm")
                            ],
                            warningText: manager.checkerboardFitsOnScreen ? nil : "Pattern too large for screen",
                            actionTitle: "Display Checkerboard",
                            isEnabled: manager.checkerboardFitsOnScreen,
                            action: { showCheckerboardDisplay = true }
                        )
                    }
                    
                    // Results card
                    ResultsPreviewCard(
                        extrinsicCount: manager.extrinsicResults.count,
                        intrinsicCount: manager.intrinsicResults.count
                    )
                }
                .padding(.horizontal, 20)
                .padding(.vertical, 16)
            }
            .background(Color(.systemGroupedBackground))
            .navigationTitle("Calibration")
            .fullScreenCover(isPresented: $showArucoDisplay) {
                ArucoMarkerDisplayView()
            }
            .fullScreenCover(isPresented: $showCheckerboardDisplay) {
                CheckerboardDisplayView()
            }
            .fullScreenCover(isPresented: $showCalibrationCoordinator) {
                CalibrationCoordinatorView()
            }
            .sheet(isPresented: $showScreenValidation) {
                ScreenSizeValidationView()
            }
        }
    }
}

// MARK: - Vision Pro Calibration Card

/// Card for starting Vision Pro guided calibration
struct VisionProCalibrationCard: View {
    let isConnected: Bool
    let connectionStatus: String
    let action: () -> Void
    
    var body: some View {
        VStack(spacing: 16) {
            // Header with Vision Pro icon
            HStack(spacing: 16) {
                ZStack {
                    RoundedRectangle(cornerRadius: 16)
                        .fill(LinearGradient(
                            colors: [Color.blue.opacity(0.2), Color.purple.opacity(0.15)],
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        ))
                        .frame(width: 60, height: 60)
                    
                    Image(systemName: "visionpro")
                        .font(.system(size: 28))
                        .foregroundStyle(LinearGradient(
                            colors: [Color.blue, Color.purple],
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        ))
                }
                
                VStack(alignment: .leading, spacing: 4) {
                    Text("Vision Pro Guided")
                        .font(.system(size: 18, weight: .bold))
                    
                    Text("Automatic calibration wizard")
                        .font(.system(size: 13))
                        .foregroundColor(.secondary)
                }
                
                Spacer()
                
                // Connection indicator
                VStack(spacing: 4) {
                    Circle()
                        .fill(isConnected ? Color.green : Color.orange)
                        .frame(width: 12, height: 12)
                    Text(isConnected ? "Ready" : "Waiting")
                        .font(.system(size: 10, weight: .medium))
                        .foregroundColor(isConnected ? .green : .orange)
                }
            }
            
            // Description
            VStack(alignment: .leading, spacing: 8) {
                FeatureRow(icon: "arrow.triangle.2.circlepath", text: "Automatic pattern switching")
                FeatureRow(icon: "waveform.path.ecg", text: "Motion-aware sample collection")
                FeatureRow(icon: "checkmark.circle", text: "Guided step-by-step process")
            }
            .padding(.horizontal, 4)
            
            // Action button
            Button(action: action) {
                HStack(spacing: 10) {
                    Image(systemName: "play.fill")
                        .font(.system(size: 14))
                    Text("Start Calibration")
                        .font(.system(size: 16, weight: .semibold))
                }
                .foregroundColor(.white)
                .frame(maxWidth: .infinity)
                .padding(.vertical, 16)
                .background(
                    LinearGradient(
                        colors: [Color.blue, Color.purple],
                        startPoint: .leading,
                        endPoint: .trailing
                    )
                )
                .clipShape(RoundedRectangle(cornerRadius: 14))
            }
        }
        .padding(20)
        .background(
            RoundedRectangle(cornerRadius: 24)
                .fill(Color(.secondarySystemBackground))
                .shadow(color: Color.blue.opacity(0.1), radius: 10, x: 0, y: 4)
        )
    }
}

struct FeatureRow: View {
    let icon: String
    let text: String
    
    var body: some View {
        HStack(spacing: 10) {
            Image(systemName: icon)
                .font(.system(size: 12))
                .foregroundColor(.blue)
                .frame(width: 20)
            Text(text)
                .font(.system(size: 13))
                .foregroundColor(.secondary)
        }
    }
}

// MARK: - Card Components

struct DeviceInfoCard: View {
    let displaySpec: iPhoneDisplaySpec
    let isValidated: Bool
    let onValidate: () -> Void
    
    var body: some View {
        VStack(spacing: 16) {
            HStack(spacing: 16) {
                // Device icon
                ZStack {
                    RoundedRectangle(cornerRadius: 12)
                        .fill(LinearGradient(
                            colors: [Color.blue.opacity(0.15), Color.cyan.opacity(0.1)],
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        ))
                        .frame(width: 56, height: 56)
                    
                    Image(systemName: "iphone")
                        .font(.system(size: 24))
                        .foregroundStyle(LinearGradient(
                            colors: [Color.blue, Color.cyan],
                            startPoint: .topLeading,
                            endPoint: .bottomTrailing
                        ))
                }
                
                VStack(alignment: .leading, spacing: 4) {
                    Text(displaySpec.displayName)
                        .font(.system(size: 17, weight: .semibold))
                    
                    Text("\(String(format: "%.1f", displaySpec.screenWidthMM)) × \(String(format: "%.1f", displaySpec.screenHeightMM)) mm")
                        .font(.system(size: 13))
                        .foregroundColor(.secondary)
                }
                
                Spacer()
                
                // Validation status
                VStack(spacing: 4) {
                    Image(systemName: isValidated ? "checkmark.seal.fill" : "exclamationmark.triangle.fill")
                        .font(.system(size: 20))
                        .foregroundColor(isValidated ? .green : .orange)
                    
                    Text(isValidated ? "Verified" : "Unverified")
                        .font(.system(size: 10, weight: .medium))
                        .foregroundColor(isValidated ? .green : .orange)
                }
            }
            
            Button(action: onValidate) {
                HStack(spacing: 8) {
                    Image(systemName: "creditcard")
                        .font(.system(size: 14))
                    Text(isValidated ? "Re-validate" : "Validate with Credit Card")
                        .font(.system(size: 14, weight: .medium))
                }
                .foregroundColor(.blue)
                .frame(maxWidth: .infinity)
                .padding(.vertical, 12)
                .background(Color.blue.opacity(0.1))
                .clipShape(RoundedRectangle(cornerRadius: 10))
            }
        }
        .padding(16)
        .background(Color(.secondarySystemBackground))
        .clipShape(RoundedRectangle(cornerRadius: 20))
    }
}

struct CalibrationOptionCard: View {
    let title: String
    let subtitle: String
    let icon: String
    let iconColor: Color
    let details: [(String, String)]
    let warningText: String?
    let actionTitle: String
    let isEnabled: Bool
    let action: () -> Void
    
    var body: some View {
        VStack(spacing: 16) {
            // Header
            HStack(spacing: 12) {
                ZStack {
                    Circle()
                        .fill(iconColor.opacity(0.1))
                        .frame(width: 44, height: 44)
                    
                    Image(systemName: icon)
                        .font(.system(size: 20))
                        .foregroundColor(iconColor)
                }
                
                VStack(alignment: .leading, spacing: 2) {
                    Text(title)
                        .font(.system(size: 16, weight: .semibold))
                    
                    Text(subtitle)
                        .font(.system(size: 12))
                        .foregroundColor(.secondary)
                }
                
                Spacer()
            }
            
            // Details grid
            HStack(spacing: 0) {
                ForEach(Array(details.enumerated()), id: \.offset) { index, detail in
                    if index > 0 {
                        Rectangle()
                            .fill(Color.gray.opacity(0.2))
                            .frame(width: 1, height: 36)
                    }
                    
                    VStack(spacing: 4) {
                        Text(detail.0)
                            .font(.system(size: 10, weight: .medium))
                            .foregroundColor(.secondary)
                        Text(detail.1)
                            .font(.system(size: 14, weight: .semibold, design: .rounded))
                            .foregroundColor(iconColor)
                    }
                    .frame(maxWidth: .infinity)
                }
            }
            .padding(.vertical, 12)
            .background(Color(.systemGray6))
            .clipShape(RoundedRectangle(cornerRadius: 10))
            
            // Warning if any
            if let warning = warningText {
                HStack(spacing: 6) {
                    Image(systemName: "exclamationmark.triangle.fill")
                        .font(.system(size: 12))
                    Text(warning)
                        .font(.system(size: 12, weight: .medium))
                }
                .foregroundColor(.red)
                .padding(.horizontal, 12)
                .padding(.vertical, 8)
                .frame(maxWidth: .infinity)
                .background(Color.red.opacity(0.1))
                .clipShape(RoundedRectangle(cornerRadius: 8))
            }
            
            // Action button
            Button(action: action) {
                HStack(spacing: 8) {
                    Text(actionTitle)
                        .font(.system(size: 15, weight: .semibold))
                    Image(systemName: "arrow.up.right")
                        .font(.system(size: 12, weight: .bold))
                }
                .foregroundColor(.white)
                .frame(maxWidth: .infinity)
                .padding(.vertical, 14)
                .background(
                    LinearGradient(
                        colors: isEnabled ? [iconColor, iconColor.opacity(0.8)] : [Color.gray, Color.gray.opacity(0.8)],
                        startPoint: .leading,
                        endPoint: .trailing
                    )
                )
                .clipShape(RoundedRectangle(cornerRadius: 12))
            }
            .disabled(!isEnabled)
        }
        .padding(16)
        .background(Color(.secondarySystemBackground))
        .clipShape(RoundedRectangle(cornerRadius: 20))
    }
}

struct ResultsPreviewCard: View {
    let extrinsicCount: Int
    let intrinsicCount: Int
    
    var totalCount: Int { extrinsicCount + intrinsicCount }
    
    var body: some View {
        NavigationLink(destination: CalibrationResultsView()) {
            HStack(spacing: 16) {
                // Icon
                ZStack {
                    Circle()
                        .fill(Color.green.opacity(0.1))
                        .frame(width: 44, height: 44)
                    
                    Image(systemName: "doc.text.magnifyingglass")
                        .font(.system(size: 18))
                        .foregroundColor(.green)
                }
                
                VStack(alignment: .leading, spacing: 4) {
                    Text("Calibration Results")
                        .font(.system(size: 16, weight: .semibold))
                        .foregroundColor(.primary)
                    
                    Text("From Vision Pro via iCloud")
                        .font(.system(size: 12))
                        .foregroundColor(.secondary)
                }
                
                Spacer()
                
                // Count badges
                if totalCount > 0 {
                    HStack(spacing: 8) {
                        if extrinsicCount > 0 {
                            ResultBadge(count: extrinsicCount, color: .purple, icon: "arrow.triangle.swap")
                        }
                        if intrinsicCount > 0 {
                            ResultBadge(count: intrinsicCount, color: .orange, icon: "camera.aperture")
                        }
                    }
                }
                
                Image(systemName: "chevron.right")
                    .font(.system(size: 14, weight: .semibold))
                    .foregroundColor(.secondary)
            }
            .padding(16)
            .background(Color(.secondarySystemBackground))
            .clipShape(RoundedRectangle(cornerRadius: 20))
        }
    }
}

struct ResultBadge: View {
    let count: Int
    let color: Color
    let icon: String
    
    var body: some View {
        HStack(spacing: 4) {
            Image(systemName: icon)
                .font(.system(size: 10))
            Text("\(count)")
                .font(.system(size: 12, weight: .bold, design: .rounded))
        }
        .foregroundColor(color)
        .padding(.horizontal, 8)
        .padding(.vertical, 4)
        .background(color.opacity(0.1))
        .clipShape(Capsule())
    }
}

// MARK: - Previews

#Preview("Calibration Tab") {
    CalibrationTabView()
}

#Preview("ArUco Marker") {
    ArucoMarkerDisplayView()
}

#Preview("Checkerboard") {
    CheckerboardDisplayView()
}

#Preview("Screen Validation") {
    ScreenSizeValidationView()
}
