// DO NOT EDIT.
// swift-format-ignore-file
//
// Generated by the Swift generator plugin for the protocol buffer compiler.
// Source: handtracking.proto
//
// For information on using the generated types, please see the documentation:
//   https://github.com/apple/swift-protobuf/

import Foundation
import SwiftProtobuf

// If the compiler emits an error on this type, it is because this file
// was generated by a version of the `protoc` Swift plug-in that is
// incompatible with the version of SwiftProtobuf to which you are linking.
// Please ensure that you are building against the same version of the API
// that was used to generate this file.
fileprivate struct _GeneratedWithProtocGenSwiftVersion: SwiftProtobuf.ProtobufAPIVersionCheck {
  struct _2: SwiftProtobuf.ProtobufAPIVersion_2 {}
  typealias Version = _2
}

/// Represents a 4x4 transformation matrix for a joint
struct Handtracking_Matrix4x4 {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  var m00: Float = 0

  var m01: Float = 0

  var m02: Float = 0

  var m03: Float = 0

  var m10: Float = 0

  var m11: Float = 0

  var m12: Float = 0

  var m13: Float = 0

  var m20: Float = 0

  var m21: Float = 0

  var m22: Float = 0

  var m23: Float = 0

  var m30: Float = 0

  var m31: Float = 0

  var m32: Float = 0

  var m33: Float = 0

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}
}

/// The skeleton of a hand, comprising multiple 4x4 matrices (one per joint)
struct Handtracking_Skeleton {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  /// Array of 4x4 matrices, expecting 24 per hand based on your structure
  var jointMatrices: [Handtracking_Matrix4x4] = []

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}
}

/// The hand tracking information, including the full 4x4 matrix for the wrist and the skeleton
struct Handtracking_Hand {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  /// 4x4 matrix for the wrist position and orientation
  var wristMatrix: Handtracking_Matrix4x4 {
    get {return _storage._wristMatrix ?? Handtracking_Matrix4x4()}
    set {_uniqueStorage()._wristMatrix = newValue}
  }
  /// Returns true if `wristMatrix` has been explicitly set.
  var hasWristMatrix: Bool {return _storage._wristMatrix != nil}
  /// Clears the value of `wristMatrix`. Subsequent reads from it will return its default value.
  mutating func clearWristMatrix() {_uniqueStorage()._wristMatrix = nil}

  /// The hand's skeleton
  var skeleton: Handtracking_Skeleton {
    get {return _storage._skeleton ?? Handtracking_Skeleton()}
    set {_uniqueStorage()._skeleton = newValue}
  }
  /// Returns true if `skeleton` has been explicitly set.
  var hasSkeleton: Bool {return _storage._skeleton != nil}
  /// Clears the value of `skeleton`. Subsequent reads from it will return its default value.
  mutating func clearSkeleton() {_uniqueStorage()._skeleton = nil}

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}

  fileprivate var _storage = _StorageClass.defaultInstance
}

/// The overall hand update message, including data for both hands
struct Handtracking_HandUpdate {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  var leftHand: Handtracking_Hand {
    get {return _storage._leftHand ?? Handtracking_Hand()}
    set {_uniqueStorage()._leftHand = newValue}
  }
  /// Returns true if `leftHand` has been explicitly set.
  var hasLeftHand: Bool {return _storage._leftHand != nil}
  /// Clears the value of `leftHand`. Subsequent reads from it will return its default value.
  mutating func clearLeftHand() {_uniqueStorage()._leftHand = nil}

  var rightHand: Handtracking_Hand {
    get {return _storage._rightHand ?? Handtracking_Hand()}
    set {_uniqueStorage()._rightHand = newValue}
  }
  /// Returns true if `rightHand` has been explicitly set.
  var hasRightHand: Bool {return _storage._rightHand != nil}
  /// Clears the value of `rightHand`. Subsequent reads from it will return its default value.
  mutating func clearRightHand() {_uniqueStorage()._rightHand = nil}

  var head: Handtracking_Matrix4x4 {
    get {return _storage._head ?? Handtracking_Matrix4x4()}
    set {_uniqueStorage()._head = newValue}
  }
  /// Returns true if `head` has been explicitly set.
  var hasHead: Bool {return _storage._head != nil}
  /// Clears the value of `head`. Subsequent reads from it will return its default value.
  mutating func clearHead() {_uniqueStorage()._head = nil}

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}

  fileprivate var _storage = _StorageClass.defaultInstance
}

/// Acknowledgement message for hand updates
struct Handtracking_HandUpdateAck {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  var message: String = String()

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}
}

#if swift(>=5.5) && canImport(_Concurrency)
extension Handtracking_Matrix4x4: @unchecked Sendable {}
extension Handtracking_Skeleton: @unchecked Sendable {}
extension Handtracking_Hand: @unchecked Sendable {}
extension Handtracking_HandUpdate: @unchecked Sendable {}
extension Handtracking_HandUpdateAck: @unchecked Sendable {}
#endif  // swift(>=5.5) && canImport(_Concurrency)

// MARK: - Code below here is support for the SwiftProtobuf runtime.

fileprivate let _protobuf_package = "handtracking"

extension Handtracking_Matrix4x4: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = _protobuf_package + ".Matrix4x4"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .same(proto: "m00"),
    2: .same(proto: "m01"),
    3: .same(proto: "m02"),
    4: .same(proto: "m03"),
    5: .same(proto: "m10"),
    6: .same(proto: "m11"),
    7: .same(proto: "m12"),
    8: .same(proto: "m13"),
    9: .same(proto: "m20"),
    10: .same(proto: "m21"),
    11: .same(proto: "m22"),
    12: .same(proto: "m23"),
    13: .same(proto: "m30"),
    14: .same(proto: "m31"),
    15: .same(proto: "m32"),
    16: .same(proto: "m33"),
  ]

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    while let fieldNumber = try decoder.nextFieldNumber() {
      // The use of inline closures is to circumvent an issue where the compiler
      // allocates stack space for every case branch when no optimizations are
      // enabled. https://github.com/apple/swift-protobuf/issues/1034
      switch fieldNumber {
      case 1: try { try decoder.decodeSingularFloatField(value: &self.m00) }()
      case 2: try { try decoder.decodeSingularFloatField(value: &self.m01) }()
      case 3: try { try decoder.decodeSingularFloatField(value: &self.m02) }()
      case 4: try { try decoder.decodeSingularFloatField(value: &self.m03) }()
      case 5: try { try decoder.decodeSingularFloatField(value: &self.m10) }()
      case 6: try { try decoder.decodeSingularFloatField(value: &self.m11) }()
      case 7: try { try decoder.decodeSingularFloatField(value: &self.m12) }()
      case 8: try { try decoder.decodeSingularFloatField(value: &self.m13) }()
      case 9: try { try decoder.decodeSingularFloatField(value: &self.m20) }()
      case 10: try { try decoder.decodeSingularFloatField(value: &self.m21) }()
      case 11: try { try decoder.decodeSingularFloatField(value: &self.m22) }()
      case 12: try { try decoder.decodeSingularFloatField(value: &self.m23) }()
      case 13: try { try decoder.decodeSingularFloatField(value: &self.m30) }()
      case 14: try { try decoder.decodeSingularFloatField(value: &self.m31) }()
      case 15: try { try decoder.decodeSingularFloatField(value: &self.m32) }()
      case 16: try { try decoder.decodeSingularFloatField(value: &self.m33) }()
      default: break
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    if self.m00 != 0 {
      try visitor.visitSingularFloatField(value: self.m00, fieldNumber: 1)
    }
    if self.m01 != 0 {
      try visitor.visitSingularFloatField(value: self.m01, fieldNumber: 2)
    }
    if self.m02 != 0 {
      try visitor.visitSingularFloatField(value: self.m02, fieldNumber: 3)
    }
    if self.m03 != 0 {
      try visitor.visitSingularFloatField(value: self.m03, fieldNumber: 4)
    }
    if self.m10 != 0 {
      try visitor.visitSingularFloatField(value: self.m10, fieldNumber: 5)
    }
    if self.m11 != 0 {
      try visitor.visitSingularFloatField(value: self.m11, fieldNumber: 6)
    }
    if self.m12 != 0 {
      try visitor.visitSingularFloatField(value: self.m12, fieldNumber: 7)
    }
    if self.m13 != 0 {
      try visitor.visitSingularFloatField(value: self.m13, fieldNumber: 8)
    }
    if self.m20 != 0 {
      try visitor.visitSingularFloatField(value: self.m20, fieldNumber: 9)
    }
    if self.m21 != 0 {
      try visitor.visitSingularFloatField(value: self.m21, fieldNumber: 10)
    }
    if self.m22 != 0 {
      try visitor.visitSingularFloatField(value: self.m22, fieldNumber: 11)
    }
    if self.m23 != 0 {
      try visitor.visitSingularFloatField(value: self.m23, fieldNumber: 12)
    }
    if self.m30 != 0 {
      try visitor.visitSingularFloatField(value: self.m30, fieldNumber: 13)
    }
    if self.m31 != 0 {
      try visitor.visitSingularFloatField(value: self.m31, fieldNumber: 14)
    }
    if self.m32 != 0 {
      try visitor.visitSingularFloatField(value: self.m32, fieldNumber: 15)
    }
    if self.m33 != 0 {
      try visitor.visitSingularFloatField(value: self.m33, fieldNumber: 16)
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: Handtracking_Matrix4x4, rhs: Handtracking_Matrix4x4) -> Bool {
    if lhs.m00 != rhs.m00 {return false}
    if lhs.m01 != rhs.m01 {return false}
    if lhs.m02 != rhs.m02 {return false}
    if lhs.m03 != rhs.m03 {return false}
    if lhs.m10 != rhs.m10 {return false}
    if lhs.m11 != rhs.m11 {return false}
    if lhs.m12 != rhs.m12 {return false}
    if lhs.m13 != rhs.m13 {return false}
    if lhs.m20 != rhs.m20 {return false}
    if lhs.m21 != rhs.m21 {return false}
    if lhs.m22 != rhs.m22 {return false}
    if lhs.m23 != rhs.m23 {return false}
    if lhs.m30 != rhs.m30 {return false}
    if lhs.m31 != rhs.m31 {return false}
    if lhs.m32 != rhs.m32 {return false}
    if lhs.m33 != rhs.m33 {return false}
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension Handtracking_Skeleton: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = _protobuf_package + ".Skeleton"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .same(proto: "jointMatrices"),
  ]

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    while let fieldNumber = try decoder.nextFieldNumber() {
      // The use of inline closures is to circumvent an issue where the compiler
      // allocates stack space for every case branch when no optimizations are
      // enabled. https://github.com/apple/swift-protobuf/issues/1034
      switch fieldNumber {
      case 1: try { try decoder.decodeRepeatedMessageField(value: &self.jointMatrices) }()
      default: break
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    if !self.jointMatrices.isEmpty {
      try visitor.visitRepeatedMessageField(value: self.jointMatrices, fieldNumber: 1)
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: Handtracking_Skeleton, rhs: Handtracking_Skeleton) -> Bool {
    if lhs.jointMatrices != rhs.jointMatrices {return false}
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension Handtracking_Hand: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = _protobuf_package + ".Hand"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .same(proto: "wristMatrix"),
    2: .same(proto: "skeleton"),
  ]

  fileprivate class _StorageClass {
    var _wristMatrix: Handtracking_Matrix4x4? = nil
    var _skeleton: Handtracking_Skeleton? = nil

    static let defaultInstance = _StorageClass()

    private init() {}

    init(copying source: _StorageClass) {
      _wristMatrix = source._wristMatrix
      _skeleton = source._skeleton
    }
  }

  fileprivate mutating func _uniqueStorage() -> _StorageClass {
    if !isKnownUniquelyReferenced(&_storage) {
      _storage = _StorageClass(copying: _storage)
    }
    return _storage
  }

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    _ = _uniqueStorage()
    try withExtendedLifetime(_storage) { (_storage: _StorageClass) in
      while let fieldNumber = try decoder.nextFieldNumber() {
        // The use of inline closures is to circumvent an issue where the compiler
        // allocates stack space for every case branch when no optimizations are
        // enabled. https://github.com/apple/swift-protobuf/issues/1034
        switch fieldNumber {
        case 1: try { try decoder.decodeSingularMessageField(value: &_storage._wristMatrix) }()
        case 2: try { try decoder.decodeSingularMessageField(value: &_storage._skeleton) }()
        default: break
        }
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    try withExtendedLifetime(_storage) { (_storage: _StorageClass) in
      // The use of inline closures is to circumvent an issue where the compiler
      // allocates stack space for every if/case branch local when no optimizations
      // are enabled. https://github.com/apple/swift-protobuf/issues/1034 and
      // https://github.com/apple/swift-protobuf/issues/1182
      try { if let v = _storage._wristMatrix {
        try visitor.visitSingularMessageField(value: v, fieldNumber: 1)
      } }()
      try { if let v = _storage._skeleton {
        try visitor.visitSingularMessageField(value: v, fieldNumber: 2)
      } }()
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: Handtracking_Hand, rhs: Handtracking_Hand) -> Bool {
    if lhs._storage !== rhs._storage {
      let storagesAreEqual: Bool = withExtendedLifetime((lhs._storage, rhs._storage)) { (_args: (_StorageClass, _StorageClass)) in
        let _storage = _args.0
        let rhs_storage = _args.1
        if _storage._wristMatrix != rhs_storage._wristMatrix {return false}
        if _storage._skeleton != rhs_storage._skeleton {return false}
        return true
      }
      if !storagesAreEqual {return false}
    }
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension Handtracking_HandUpdate: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = _protobuf_package + ".HandUpdate"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .standard(proto: "left_hand"),
    2: .standard(proto: "right_hand"),
    3: .same(proto: "Head"),
  ]

  fileprivate class _StorageClass {
    var _leftHand: Handtracking_Hand? = nil
    var _rightHand: Handtracking_Hand? = nil
    var _head: Handtracking_Matrix4x4? = nil

    static let defaultInstance = _StorageClass()

    private init() {}

    init(copying source: _StorageClass) {
      _leftHand = source._leftHand
      _rightHand = source._rightHand
      _head = source._head
    }
  }

  fileprivate mutating func _uniqueStorage() -> _StorageClass {
    if !isKnownUniquelyReferenced(&_storage) {
      _storage = _StorageClass(copying: _storage)
    }
    return _storage
  }

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    _ = _uniqueStorage()
    try withExtendedLifetime(_storage) { (_storage: _StorageClass) in
      while let fieldNumber = try decoder.nextFieldNumber() {
        // The use of inline closures is to circumvent an issue where the compiler
        // allocates stack space for every case branch when no optimizations are
        // enabled. https://github.com/apple/swift-protobuf/issues/1034
        switch fieldNumber {
        case 1: try { try decoder.decodeSingularMessageField(value: &_storage._leftHand) }()
        case 2: try { try decoder.decodeSingularMessageField(value: &_storage._rightHand) }()
        case 3: try { try decoder.decodeSingularMessageField(value: &_storage._head) }()
        default: break
        }
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    try withExtendedLifetime(_storage) { (_storage: _StorageClass) in
      // The use of inline closures is to circumvent an issue where the compiler
      // allocates stack space for every if/case branch local when no optimizations
      // are enabled. https://github.com/apple/swift-protobuf/issues/1034 and
      // https://github.com/apple/swift-protobuf/issues/1182
      try { if let v = _storage._leftHand {
        try visitor.visitSingularMessageField(value: v, fieldNumber: 1)
      } }()
      try { if let v = _storage._rightHand {
        try visitor.visitSingularMessageField(value: v, fieldNumber: 2)
      } }()
      try { if let v = _storage._head {
        try visitor.visitSingularMessageField(value: v, fieldNumber: 3)
      } }()
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: Handtracking_HandUpdate, rhs: Handtracking_HandUpdate) -> Bool {
    if lhs._storage !== rhs._storage {
      let storagesAreEqual: Bool = withExtendedLifetime((lhs._storage, rhs._storage)) { (_args: (_StorageClass, _StorageClass)) in
        let _storage = _args.0
        let rhs_storage = _args.1
        if _storage._leftHand != rhs_storage._leftHand {return false}
        if _storage._rightHand != rhs_storage._rightHand {return false}
        if _storage._head != rhs_storage._head {return false}
        return true
      }
      if !storagesAreEqual {return false}
    }
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension Handtracking_HandUpdateAck: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = _protobuf_package + ".HandUpdateAck"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .same(proto: "message"),
  ]

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    while let fieldNumber = try decoder.nextFieldNumber() {
      // The use of inline closures is to circumvent an issue where the compiler
      // allocates stack space for every case branch when no optimizations are
      // enabled. https://github.com/apple/swift-protobuf/issues/1034
      switch fieldNumber {
      case 1: try { try decoder.decodeSingularStringField(value: &self.message) }()
      default: break
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    if !self.message.isEmpty {
      try visitor.visitSingularStringField(value: self.message, fieldNumber: 1)
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: Handtracking_HandUpdateAck, rhs: Handtracking_HandUpdateAck) -> Bool {
    if lhs.message != rhs.message {return false}
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}
