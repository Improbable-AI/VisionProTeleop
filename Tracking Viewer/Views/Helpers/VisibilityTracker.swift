//
//  VisibilityTracker.swift
//  Tracking Viewer
//
//  Created on 12/2/25.
//

import SwiftUI

/// Preference key to aggregate visibility data (Frame in global space)
struct VisibilityPreferenceKey: PreferenceKey {
    typealias Value = [String: CGRect]
    
    static var defaultValue: [String: CGRect] = [:]
    
    static func reduce(value: inout [String: CGRect], nextValue: () -> [String: CGRect]) {
        value.merge(nextValue()) { _, new in new }
    }
}

/// View modifier to track visibility of an item within a ScrollView
struct VisibilityTracker: ViewModifier {
    let id: String
    let coordinateSpace: CoordinateSpace
    
    func body(content: Content) -> some View {
        content
            .background(
                GeometryReader { geo in
                    Color.clear
                        .preference(
                            key: VisibilityPreferenceKey.self,
                            value: [id: geo.frame(in: coordinateSpace)]
                        )
                }
            )
    }
}

extension View {
    func trackVisibility(id: String, in space: CoordinateSpace) -> some View {
        modifier(VisibilityTracker(id: id, coordinateSpace: space))
    }
}
