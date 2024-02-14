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

/// The hand tracking information message.
struct Handtracking_HandUpdate {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  var leftHand: Handtracking_HandUpdate.Hand {
    get {return _leftHand ?? Handtracking_HandUpdate.Hand()}
    set {_leftHand = newValue}
  }
  /// Returns true if `leftHand` has been explicitly set.
  var hasLeftHand: Bool {return self._leftHand != nil}
  /// Clears the value of `leftHand`. Subsequent reads from it will return its default value.
  mutating func clearLeftHand() {self._leftHand = nil}

  var rightHand: Handtracking_HandUpdate.Hand {
    get {return _rightHand ?? Handtracking_HandUpdate.Hand()}
    set {_rightHand = newValue}
  }
  /// Returns true if `rightHand` has been explicitly set.
  var hasRightHand: Bool {return self._rightHand != nil}
  /// Clears the value of `rightHand`. Subsequent reads from it will return its default value.
  mutating func clearRightHand() {self._rightHand = nil}

  var unknownFields = SwiftProtobuf.UnknownStorage()

  struct Hand {
    // SwiftProtobuf.Message conformance is added in an extension below. See the
    // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
    // methods supported on all messages.

    var x: Float = 0

    var y: Float = 0

    var z: Float = 0

    var qx: Float = 0

    var qy: Float = 0

    var qz: Float = 0

    var qw: Float = 0

    var unknownFields = SwiftProtobuf.UnknownStorage()

    init() {}
  }

  init() {}

  fileprivate var _leftHand: Handtracking_HandUpdate.Hand? = nil
  fileprivate var _rightHand: Handtracking_HandUpdate.Hand? = nil
}

struct Handtracking_HandUpdateAck {
  // SwiftProtobuf.Message conformance is added in an extension below. See the
  // `Message` and `Message+*Additions` files in the SwiftProtobuf library for
  // methods supported on all messages.

  var message: String = String()

  var unknownFields = SwiftProtobuf.UnknownStorage()

  init() {}
}

#if swift(>=5.5) && canImport(_Concurrency)
extension Handtracking_HandUpdate: @unchecked Sendable {}
extension Handtracking_HandUpdate.Hand: @unchecked Sendable {}
extension Handtracking_HandUpdateAck: @unchecked Sendable {}
#endif  // swift(>=5.5) && canImport(_Concurrency)

// MARK: - Code below here is support for the SwiftProtobuf runtime.

fileprivate let _protobuf_package = "handtracking"

extension Handtracking_HandUpdate: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = _protobuf_package + ".HandUpdate"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .standard(proto: "left_hand"),
    2: .standard(proto: "right_hand"),
  ]

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    while let fieldNumber = try decoder.nextFieldNumber() {
      // The use of inline closures is to circumvent an issue where the compiler
      // allocates stack space for every case branch when no optimizations are
      // enabled. https://github.com/apple/swift-protobuf/issues/1034
      switch fieldNumber {
      case 1: try { try decoder.decodeSingularMessageField(value: &self._leftHand) }()
      case 2: try { try decoder.decodeSingularMessageField(value: &self._rightHand) }()
      default: break
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    // The use of inline closures is to circumvent an issue where the compiler
    // allocates stack space for every if/case branch local when no optimizations
    // are enabled. https://github.com/apple/swift-protobuf/issues/1034 and
    // https://github.com/apple/swift-protobuf/issues/1182
    try { if let v = self._leftHand {
      try visitor.visitSingularMessageField(value: v, fieldNumber: 1)
    } }()
    try { if let v = self._rightHand {
      try visitor.visitSingularMessageField(value: v, fieldNumber: 2)
    } }()
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: Handtracking_HandUpdate, rhs: Handtracking_HandUpdate) -> Bool {
    if lhs._leftHand != rhs._leftHand {return false}
    if lhs._rightHand != rhs._rightHand {return false}
    if lhs.unknownFields != rhs.unknownFields {return false}
    return true
  }
}

extension Handtracking_HandUpdate.Hand: SwiftProtobuf.Message, SwiftProtobuf._MessageImplementationBase, SwiftProtobuf._ProtoNameProviding {
  static let protoMessageName: String = Handtracking_HandUpdate.protoMessageName + ".Hand"
  static let _protobuf_nameMap: SwiftProtobuf._NameMap = [
    1: .same(proto: "x"),
    2: .same(proto: "y"),
    3: .same(proto: "z"),
    4: .same(proto: "qx"),
    5: .same(proto: "qy"),
    6: .same(proto: "qz"),
    7: .same(proto: "qw"),
  ]

  mutating func decodeMessage<D: SwiftProtobuf.Decoder>(decoder: inout D) throws {
    while let fieldNumber = try decoder.nextFieldNumber() {
      // The use of inline closures is to circumvent an issue where the compiler
      // allocates stack space for every case branch when no optimizations are
      // enabled. https://github.com/apple/swift-protobuf/issues/1034
      switch fieldNumber {
      case 1: try { try decoder.decodeSingularFloatField(value: &self.x) }()
      case 2: try { try decoder.decodeSingularFloatField(value: &self.y) }()
      case 3: try { try decoder.decodeSingularFloatField(value: &self.z) }()
      case 4: try { try decoder.decodeSingularFloatField(value: &self.qx) }()
      case 5: try { try decoder.decodeSingularFloatField(value: &self.qy) }()
      case 6: try { try decoder.decodeSingularFloatField(value: &self.qz) }()
      case 7: try { try decoder.decodeSingularFloatField(value: &self.qw) }()
      default: break
      }
    }
  }

  func traverse<V: SwiftProtobuf.Visitor>(visitor: inout V) throws {
    if self.x != 0 {
      try visitor.visitSingularFloatField(value: self.x, fieldNumber: 1)
    }
    if self.y != 0 {
      try visitor.visitSingularFloatField(value: self.y, fieldNumber: 2)
    }
    if self.z != 0 {
      try visitor.visitSingularFloatField(value: self.z, fieldNumber: 3)
    }
    if self.qx != 0 {
      try visitor.visitSingularFloatField(value: self.qx, fieldNumber: 4)
    }
    if self.qy != 0 {
      try visitor.visitSingularFloatField(value: self.qy, fieldNumber: 5)
    }
    if self.qz != 0 {
      try visitor.visitSingularFloatField(value: self.qz, fieldNumber: 6)
    }
    if self.qw != 0 {
      try visitor.visitSingularFloatField(value: self.qw, fieldNumber: 7)
    }
    try unknownFields.traverse(visitor: &visitor)
  }

  static func ==(lhs: Handtracking_HandUpdate.Hand, rhs: Handtracking_HandUpdate.Hand) -> Bool {
    if lhs.x != rhs.x {return false}
    if lhs.y != rhs.y {return false}
    if lhs.z != rhs.z {return false}
    if lhs.qx != rhs.qx {return false}
    if lhs.qy != rhs.qy {return false}
    if lhs.qz != rhs.qz {return false}
    if lhs.qw != rhs.qw {return false}
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