// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: static_layer_setting.proto

#ifndef PROTOBUF_static_5flayer_5fsetting_2eproto__INCLUDED
#define PROTOBUF_static_5flayer_5fsetting_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3004000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
namespace or_costmap {
class ParaStaticLayer;
class ParaStaticLayerDefaultTypeInternal;
extern ParaStaticLayerDefaultTypeInternal _ParaStaticLayer_default_instance_;
}  // namespace or_costmap

namespace or_costmap {

namespace protobuf_static_5flayer_5fsetting_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static void InitDefaultsImpl();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_static_5flayer_5fsetting_2eproto

// ===================================================================

class ParaStaticLayer : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:or_costmap.ParaStaticLayer) */ {
 public:
  ParaStaticLayer();
  virtual ~ParaStaticLayer();

  ParaStaticLayer(const ParaStaticLayer& from);

  inline ParaStaticLayer& operator=(const ParaStaticLayer& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ParaStaticLayer(ParaStaticLayer&& from) noexcept
    : ParaStaticLayer() {
    *this = ::std::move(from);
  }

  inline ParaStaticLayer& operator=(ParaStaticLayer&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ParaStaticLayer& default_instance();

  static inline const ParaStaticLayer* internal_default_instance() {
    return reinterpret_cast<const ParaStaticLayer*>(
               &_ParaStaticLayer_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(ParaStaticLayer* other);
  friend void swap(ParaStaticLayer& a, ParaStaticLayer& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ParaStaticLayer* New() const PROTOBUF_FINAL { return New(NULL); }

  ParaStaticLayer* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ParaStaticLayer& from);
  void MergeFrom(const ParaStaticLayer& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(ParaStaticLayer* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required string topic_name = 8;
  bool has_topic_name() const;
  void clear_topic_name();
  static const int kTopicNameFieldNumber = 8;
  const ::std::string& topic_name() const;
  void set_topic_name(const ::std::string& value);
  #if LANG_CXX11
  void set_topic_name(::std::string&& value);
  #endif
  void set_topic_name(const char* value);
  void set_topic_name(const char* value, size_t size);
  ::std::string* mutable_topic_name();
  ::std::string* release_topic_name();
  void set_allocated_topic_name(::std::string* topic_name);

  // required bool first_map_only = 1;
  bool has_first_map_only() const;
  void clear_first_map_only();
  static const int kFirstMapOnlyFieldNumber = 1;
  bool first_map_only() const;
  void set_first_map_only(bool value);

  // required bool subscribe_to_updates = 2;
  bool has_subscribe_to_updates() const;
  void clear_subscribe_to_updates();
  static const int kSubscribeToUpdatesFieldNumber = 2;
  bool subscribe_to_updates() const;
  void set_subscribe_to_updates(bool value);

  // required bool track_unknown_space = 3;
  bool has_track_unknown_space() const;
  void clear_track_unknown_space();
  static const int kTrackUnknownSpaceFieldNumber = 3;
  bool track_unknown_space() const;
  void set_track_unknown_space(bool value);

  // required bool use_maximum = 4;
  bool has_use_maximum() const;
  void clear_use_maximum();
  static const int kUseMaximumFieldNumber = 4;
  bool use_maximum() const;
  void set_use_maximum(bool value);

  // required int32 unknown_cost_value = 5;
  bool has_unknown_cost_value() const;
  void clear_unknown_cost_value();
  static const int kUnknownCostValueFieldNumber = 5;
  ::google::protobuf::int32 unknown_cost_value() const;
  void set_unknown_cost_value(::google::protobuf::int32 value);

  // required bool trinary_map = 6;
  bool has_trinary_map() const;
  void clear_trinary_map();
  static const int kTrinaryMapFieldNumber = 6;
  bool trinary_map() const;
  void set_trinary_map(bool value);

  // required int32 lethal_threshold = 7;
  bool has_lethal_threshold() const;
  void clear_lethal_threshold();
  static const int kLethalThresholdFieldNumber = 7;
  ::google::protobuf::int32 lethal_threshold() const;
  void set_lethal_threshold(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:or_costmap.ParaStaticLayer)
 private:
  void set_has_first_map_only();
  void clear_has_first_map_only();
  void set_has_subscribe_to_updates();
  void clear_has_subscribe_to_updates();
  void set_has_track_unknown_space();
  void clear_has_track_unknown_space();
  void set_has_use_maximum();
  void clear_has_use_maximum();
  void set_has_unknown_cost_value();
  void clear_has_unknown_cost_value();
  void set_has_trinary_map();
  void clear_has_trinary_map();
  void set_has_lethal_threshold();
  void clear_has_lethal_threshold();
  void set_has_topic_name();
  void clear_has_topic_name();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr topic_name_;
  bool first_map_only_;
  bool subscribe_to_updates_;
  bool track_unknown_space_;
  bool use_maximum_;
  ::google::protobuf::int32 unknown_cost_value_;
  bool trinary_map_;
  ::google::protobuf::int32 lethal_threshold_;
  friend struct protobuf_static_5flayer_5fsetting_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ParaStaticLayer

// required bool first_map_only = 1;
inline bool ParaStaticLayer::has_first_map_only() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ParaStaticLayer::set_has_first_map_only() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ParaStaticLayer::clear_has_first_map_only() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ParaStaticLayer::clear_first_map_only() {
  first_map_only_ = false;
  clear_has_first_map_only();
}
inline bool ParaStaticLayer::first_map_only() const {
  // @@protoc_insertion_point(field_get:or_costmap.ParaStaticLayer.first_map_only)
  return first_map_only_;
}
inline void ParaStaticLayer::set_first_map_only(bool value) {
  set_has_first_map_only();
  first_map_only_ = value;
  // @@protoc_insertion_point(field_set:or_costmap.ParaStaticLayer.first_map_only)
}

// required bool subscribe_to_updates = 2;
inline bool ParaStaticLayer::has_subscribe_to_updates() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ParaStaticLayer::set_has_subscribe_to_updates() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ParaStaticLayer::clear_has_subscribe_to_updates() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ParaStaticLayer::clear_subscribe_to_updates() {
  subscribe_to_updates_ = false;
  clear_has_subscribe_to_updates();
}
inline bool ParaStaticLayer::subscribe_to_updates() const {
  // @@protoc_insertion_point(field_get:or_costmap.ParaStaticLayer.subscribe_to_updates)
  return subscribe_to_updates_;
}
inline void ParaStaticLayer::set_subscribe_to_updates(bool value) {
  set_has_subscribe_to_updates();
  subscribe_to_updates_ = value;
  // @@protoc_insertion_point(field_set:or_costmap.ParaStaticLayer.subscribe_to_updates)
}

// required bool track_unknown_space = 3;
inline bool ParaStaticLayer::has_track_unknown_space() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void ParaStaticLayer::set_has_track_unknown_space() {
  _has_bits_[0] |= 0x00000008u;
}
inline void ParaStaticLayer::clear_has_track_unknown_space() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void ParaStaticLayer::clear_track_unknown_space() {
  track_unknown_space_ = false;
  clear_has_track_unknown_space();
}
inline bool ParaStaticLayer::track_unknown_space() const {
  // @@protoc_insertion_point(field_get:or_costmap.ParaStaticLayer.track_unknown_space)
  return track_unknown_space_;
}
inline void ParaStaticLayer::set_track_unknown_space(bool value) {
  set_has_track_unknown_space();
  track_unknown_space_ = value;
  // @@protoc_insertion_point(field_set:or_costmap.ParaStaticLayer.track_unknown_space)
}

// required bool use_maximum = 4;
inline bool ParaStaticLayer::has_use_maximum() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void ParaStaticLayer::set_has_use_maximum() {
  _has_bits_[0] |= 0x00000010u;
}
inline void ParaStaticLayer::clear_has_use_maximum() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void ParaStaticLayer::clear_use_maximum() {
  use_maximum_ = false;
  clear_has_use_maximum();
}
inline bool ParaStaticLayer::use_maximum() const {
  // @@protoc_insertion_point(field_get:or_costmap.ParaStaticLayer.use_maximum)
  return use_maximum_;
}
inline void ParaStaticLayer::set_use_maximum(bool value) {
  set_has_use_maximum();
  use_maximum_ = value;
  // @@protoc_insertion_point(field_set:or_costmap.ParaStaticLayer.use_maximum)
}

// required int32 unknown_cost_value = 5;
inline bool ParaStaticLayer::has_unknown_cost_value() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void ParaStaticLayer::set_has_unknown_cost_value() {
  _has_bits_[0] |= 0x00000020u;
}
inline void ParaStaticLayer::clear_has_unknown_cost_value() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void ParaStaticLayer::clear_unknown_cost_value() {
  unknown_cost_value_ = 0;
  clear_has_unknown_cost_value();
}
inline ::google::protobuf::int32 ParaStaticLayer::unknown_cost_value() const {
  // @@protoc_insertion_point(field_get:or_costmap.ParaStaticLayer.unknown_cost_value)
  return unknown_cost_value_;
}
inline void ParaStaticLayer::set_unknown_cost_value(::google::protobuf::int32 value) {
  set_has_unknown_cost_value();
  unknown_cost_value_ = value;
  // @@protoc_insertion_point(field_set:or_costmap.ParaStaticLayer.unknown_cost_value)
}

// required bool trinary_map = 6;
inline bool ParaStaticLayer::has_trinary_map() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void ParaStaticLayer::set_has_trinary_map() {
  _has_bits_[0] |= 0x00000040u;
}
inline void ParaStaticLayer::clear_has_trinary_map() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void ParaStaticLayer::clear_trinary_map() {
  trinary_map_ = false;
  clear_has_trinary_map();
}
inline bool ParaStaticLayer::trinary_map() const {
  // @@protoc_insertion_point(field_get:or_costmap.ParaStaticLayer.trinary_map)
  return trinary_map_;
}
inline void ParaStaticLayer::set_trinary_map(bool value) {
  set_has_trinary_map();
  trinary_map_ = value;
  // @@protoc_insertion_point(field_set:or_costmap.ParaStaticLayer.trinary_map)
}

// required int32 lethal_threshold = 7;
inline bool ParaStaticLayer::has_lethal_threshold() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void ParaStaticLayer::set_has_lethal_threshold() {
  _has_bits_[0] |= 0x00000080u;
}
inline void ParaStaticLayer::clear_has_lethal_threshold() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void ParaStaticLayer::clear_lethal_threshold() {
  lethal_threshold_ = 0;
  clear_has_lethal_threshold();
}
inline ::google::protobuf::int32 ParaStaticLayer::lethal_threshold() const {
  // @@protoc_insertion_point(field_get:or_costmap.ParaStaticLayer.lethal_threshold)
  return lethal_threshold_;
}
inline void ParaStaticLayer::set_lethal_threshold(::google::protobuf::int32 value) {
  set_has_lethal_threshold();
  lethal_threshold_ = value;
  // @@protoc_insertion_point(field_set:or_costmap.ParaStaticLayer.lethal_threshold)
}

// required string topic_name = 8;
inline bool ParaStaticLayer::has_topic_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ParaStaticLayer::set_has_topic_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ParaStaticLayer::clear_has_topic_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ParaStaticLayer::clear_topic_name() {
  topic_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_topic_name();
}
inline const ::std::string& ParaStaticLayer::topic_name() const {
  // @@protoc_insertion_point(field_get:or_costmap.ParaStaticLayer.topic_name)
  return topic_name_.GetNoArena();
}
inline void ParaStaticLayer::set_topic_name(const ::std::string& value) {
  set_has_topic_name();
  topic_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:or_costmap.ParaStaticLayer.topic_name)
}
#if LANG_CXX11
inline void ParaStaticLayer::set_topic_name(::std::string&& value) {
  set_has_topic_name();
  topic_name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:or_costmap.ParaStaticLayer.topic_name)
}
#endif
inline void ParaStaticLayer::set_topic_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_topic_name();
  topic_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:or_costmap.ParaStaticLayer.topic_name)
}
inline void ParaStaticLayer::set_topic_name(const char* value, size_t size) {
  set_has_topic_name();
  topic_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:or_costmap.ParaStaticLayer.topic_name)
}
inline ::std::string* ParaStaticLayer::mutable_topic_name() {
  set_has_topic_name();
  // @@protoc_insertion_point(field_mutable:or_costmap.ParaStaticLayer.topic_name)
  return topic_name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* ParaStaticLayer::release_topic_name() {
  // @@protoc_insertion_point(field_release:or_costmap.ParaStaticLayer.topic_name)
  clear_has_topic_name();
  return topic_name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void ParaStaticLayer::set_allocated_topic_name(::std::string* topic_name) {
  if (topic_name != NULL) {
    set_has_topic_name();
  } else {
    clear_has_topic_name();
  }
  topic_name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), topic_name);
  // @@protoc_insertion_point(field_set_allocated:or_costmap.ParaStaticLayer.topic_name)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace or_costmap

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_static_5flayer_5fsetting_2eproto__INCLUDED
