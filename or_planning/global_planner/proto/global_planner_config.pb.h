// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: global_planner_config.proto

#ifndef PROTOBUF_global_5fplanner_5fconfig_2eproto__INCLUDED
#define PROTOBUF_global_5fplanner_5fconfig_2eproto__INCLUDED

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
namespace or_global_planner {
class GlobalPlannerConfig;
class GlobalPlannerConfigDefaultTypeInternal;
extern GlobalPlannerConfigDefaultTypeInternal _GlobalPlannerConfig_default_instance_;
}  // namespace or_global_planner

namespace or_global_planner {

namespace protobuf_global_5fplanner_5fconfig_2eproto {
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
}  // namespace protobuf_global_5fplanner_5fconfig_2eproto

// ===================================================================

class GlobalPlannerConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:or_global_planner.GlobalPlannerConfig) */ {
 public:
  GlobalPlannerConfig();
  virtual ~GlobalPlannerConfig();

  GlobalPlannerConfig(const GlobalPlannerConfig& from);

  inline GlobalPlannerConfig& operator=(const GlobalPlannerConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  GlobalPlannerConfig(GlobalPlannerConfig&& from) noexcept
    : GlobalPlannerConfig() {
    *this = ::std::move(from);
  }

  inline GlobalPlannerConfig& operator=(GlobalPlannerConfig&& from) noexcept {
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
  static const GlobalPlannerConfig& default_instance();

  static inline const GlobalPlannerConfig* internal_default_instance() {
    return reinterpret_cast<const GlobalPlannerConfig*>(
               &_GlobalPlannerConfig_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(GlobalPlannerConfig* other);
  friend void swap(GlobalPlannerConfig& a, GlobalPlannerConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline GlobalPlannerConfig* New() const PROTOBUF_FINAL { return New(NULL); }

  GlobalPlannerConfig* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const GlobalPlannerConfig& from);
  void MergeFrom(const GlobalPlannerConfig& from);
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
  void InternalSwap(GlobalPlannerConfig* other);
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

  // repeated string name = 1;
  int name_size() const;
  void clear_name();
  static const int kNameFieldNumber = 1;
  const ::std::string& name(int index) const;
  ::std::string* mutable_name(int index);
  void set_name(int index, const ::std::string& value);
  #if LANG_CXX11
  void set_name(int index, ::std::string&& value);
  #endif
  void set_name(int index, const char* value);
  void set_name(int index, const char* value, size_t size);
  ::std::string* add_name();
  void add_name(const ::std::string& value);
  #if LANG_CXX11
  void add_name(::std::string&& value);
  #endif
  void add_name(const char* value);
  void add_name(const char* value, size_t size);
  const ::google::protobuf::RepeatedPtrField< ::std::string>& name() const;
  ::google::protobuf::RepeatedPtrField< ::std::string>* mutable_name();

  // optional string selected_algorithm = 2;
  bool has_selected_algorithm() const;
  void clear_selected_algorithm();
  static const int kSelectedAlgorithmFieldNumber = 2;
  const ::std::string& selected_algorithm() const;
  void set_selected_algorithm(const ::std::string& value);
  #if LANG_CXX11
  void set_selected_algorithm(::std::string&& value);
  #endif
  void set_selected_algorithm(const char* value);
  void set_selected_algorithm(const char* value, size_t size);
  ::std::string* mutable_selected_algorithm();
  ::std::string* release_selected_algorithm();
  void set_allocated_selected_algorithm(::std::string* selected_algorithm);

  // required int32 frequency = 3;
  bool has_frequency() const;
  void clear_frequency();
  static const int kFrequencyFieldNumber = 3;
  ::google::protobuf::int32 frequency() const;
  void set_frequency(::google::protobuf::int32 value);

  // required int32 max_retries = 4;
  bool has_max_retries() const;
  void clear_max_retries();
  static const int kMaxRetriesFieldNumber = 4;
  ::google::protobuf::int32 max_retries() const;
  void set_max_retries(::google::protobuf::int32 value);

  // required double goal_distance_tolerance = 5;
  bool has_goal_distance_tolerance() const;
  void clear_goal_distance_tolerance();
  static const int kGoalDistanceToleranceFieldNumber = 5;
  double goal_distance_tolerance() const;
  void set_goal_distance_tolerance(double value);

  // required double goal_angle_tolerance = 6;
  bool has_goal_angle_tolerance() const;
  void clear_goal_angle_tolerance();
  static const int kGoalAngleToleranceFieldNumber = 6;
  double goal_angle_tolerance() const;
  void set_goal_angle_tolerance(double value);

  // @@protoc_insertion_point(class_scope:or_global_planner.GlobalPlannerConfig)
 private:
  void set_has_selected_algorithm();
  void clear_has_selected_algorithm();
  void set_has_frequency();
  void clear_has_frequency();
  void set_has_max_retries();
  void clear_has_max_retries();
  void set_has_goal_distance_tolerance();
  void clear_has_goal_distance_tolerance();
  void set_has_goal_angle_tolerance();
  void clear_has_goal_angle_tolerance();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::std::string> name_;
  ::google::protobuf::internal::ArenaStringPtr selected_algorithm_;
  ::google::protobuf::int32 frequency_;
  ::google::protobuf::int32 max_retries_;
  double goal_distance_tolerance_;
  double goal_angle_tolerance_;
  friend struct protobuf_global_5fplanner_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// GlobalPlannerConfig

// repeated string name = 1;
inline int GlobalPlannerConfig::name_size() const {
  return name_.size();
}
inline void GlobalPlannerConfig::clear_name() {
  name_.Clear();
}
inline const ::std::string& GlobalPlannerConfig::name(int index) const {
  // @@protoc_insertion_point(field_get:or_global_planner.GlobalPlannerConfig.name)
  return name_.Get(index);
}
inline ::std::string* GlobalPlannerConfig::mutable_name(int index) {
  // @@protoc_insertion_point(field_mutable:or_global_planner.GlobalPlannerConfig.name)
  return name_.Mutable(index);
}
inline void GlobalPlannerConfig::set_name(int index, const ::std::string& value) {
  // @@protoc_insertion_point(field_set:or_global_planner.GlobalPlannerConfig.name)
  name_.Mutable(index)->assign(value);
}
#if LANG_CXX11
inline void GlobalPlannerConfig::set_name(int index, ::std::string&& value) {
  // @@protoc_insertion_point(field_set:or_global_planner.GlobalPlannerConfig.name)
  name_.Mutable(index)->assign(std::move(value));
}
#endif
inline void GlobalPlannerConfig::set_name(int index, const char* value) {
  GOOGLE_DCHECK(value != NULL);
  name_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:or_global_planner.GlobalPlannerConfig.name)
}
inline void GlobalPlannerConfig::set_name(int index, const char* value, size_t size) {
  name_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:or_global_planner.GlobalPlannerConfig.name)
}
inline ::std::string* GlobalPlannerConfig::add_name() {
  // @@protoc_insertion_point(field_add_mutable:or_global_planner.GlobalPlannerConfig.name)
  return name_.Add();
}
inline void GlobalPlannerConfig::add_name(const ::std::string& value) {
  name_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:or_global_planner.GlobalPlannerConfig.name)
}
#if LANG_CXX11
inline void GlobalPlannerConfig::add_name(::std::string&& value) {
  name_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:or_global_planner.GlobalPlannerConfig.name)
}
#endif
inline void GlobalPlannerConfig::add_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  name_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:or_global_planner.GlobalPlannerConfig.name)
}
inline void GlobalPlannerConfig::add_name(const char* value, size_t size) {
  name_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:or_global_planner.GlobalPlannerConfig.name)
}
inline const ::google::protobuf::RepeatedPtrField< ::std::string>&
GlobalPlannerConfig::name() const {
  // @@protoc_insertion_point(field_list:or_global_planner.GlobalPlannerConfig.name)
  return name_;
}
inline ::google::protobuf::RepeatedPtrField< ::std::string>*
GlobalPlannerConfig::mutable_name() {
  // @@protoc_insertion_point(field_mutable_list:or_global_planner.GlobalPlannerConfig.name)
  return &name_;
}

// optional string selected_algorithm = 2;
inline bool GlobalPlannerConfig::has_selected_algorithm() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void GlobalPlannerConfig::set_has_selected_algorithm() {
  _has_bits_[0] |= 0x00000001u;
}
inline void GlobalPlannerConfig::clear_has_selected_algorithm() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void GlobalPlannerConfig::clear_selected_algorithm() {
  selected_algorithm_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_selected_algorithm();
}
inline const ::std::string& GlobalPlannerConfig::selected_algorithm() const {
  // @@protoc_insertion_point(field_get:or_global_planner.GlobalPlannerConfig.selected_algorithm)
  return selected_algorithm_.GetNoArena();
}
inline void GlobalPlannerConfig::set_selected_algorithm(const ::std::string& value) {
  set_has_selected_algorithm();
  selected_algorithm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:or_global_planner.GlobalPlannerConfig.selected_algorithm)
}
#if LANG_CXX11
inline void GlobalPlannerConfig::set_selected_algorithm(::std::string&& value) {
  set_has_selected_algorithm();
  selected_algorithm_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:or_global_planner.GlobalPlannerConfig.selected_algorithm)
}
#endif
inline void GlobalPlannerConfig::set_selected_algorithm(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_selected_algorithm();
  selected_algorithm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:or_global_planner.GlobalPlannerConfig.selected_algorithm)
}
inline void GlobalPlannerConfig::set_selected_algorithm(const char* value, size_t size) {
  set_has_selected_algorithm();
  selected_algorithm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:or_global_planner.GlobalPlannerConfig.selected_algorithm)
}
inline ::std::string* GlobalPlannerConfig::mutable_selected_algorithm() {
  set_has_selected_algorithm();
  // @@protoc_insertion_point(field_mutable:or_global_planner.GlobalPlannerConfig.selected_algorithm)
  return selected_algorithm_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* GlobalPlannerConfig::release_selected_algorithm() {
  // @@protoc_insertion_point(field_release:or_global_planner.GlobalPlannerConfig.selected_algorithm)
  clear_has_selected_algorithm();
  return selected_algorithm_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void GlobalPlannerConfig::set_allocated_selected_algorithm(::std::string* selected_algorithm) {
  if (selected_algorithm != NULL) {
    set_has_selected_algorithm();
  } else {
    clear_has_selected_algorithm();
  }
  selected_algorithm_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), selected_algorithm);
  // @@protoc_insertion_point(field_set_allocated:or_global_planner.GlobalPlannerConfig.selected_algorithm)
}

// required int32 frequency = 3;
inline bool GlobalPlannerConfig::has_frequency() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void GlobalPlannerConfig::set_has_frequency() {
  _has_bits_[0] |= 0x00000002u;
}
inline void GlobalPlannerConfig::clear_has_frequency() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void GlobalPlannerConfig::clear_frequency() {
  frequency_ = 0;
  clear_has_frequency();
}
inline ::google::protobuf::int32 GlobalPlannerConfig::frequency() const {
  // @@protoc_insertion_point(field_get:or_global_planner.GlobalPlannerConfig.frequency)
  return frequency_;
}
inline void GlobalPlannerConfig::set_frequency(::google::protobuf::int32 value) {
  set_has_frequency();
  frequency_ = value;
  // @@protoc_insertion_point(field_set:or_global_planner.GlobalPlannerConfig.frequency)
}

// required int32 max_retries = 4;
inline bool GlobalPlannerConfig::has_max_retries() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void GlobalPlannerConfig::set_has_max_retries() {
  _has_bits_[0] |= 0x00000004u;
}
inline void GlobalPlannerConfig::clear_has_max_retries() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void GlobalPlannerConfig::clear_max_retries() {
  max_retries_ = 0;
  clear_has_max_retries();
}
inline ::google::protobuf::int32 GlobalPlannerConfig::max_retries() const {
  // @@protoc_insertion_point(field_get:or_global_planner.GlobalPlannerConfig.max_retries)
  return max_retries_;
}
inline void GlobalPlannerConfig::set_max_retries(::google::protobuf::int32 value) {
  set_has_max_retries();
  max_retries_ = value;
  // @@protoc_insertion_point(field_set:or_global_planner.GlobalPlannerConfig.max_retries)
}

// required double goal_distance_tolerance = 5;
inline bool GlobalPlannerConfig::has_goal_distance_tolerance() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void GlobalPlannerConfig::set_has_goal_distance_tolerance() {
  _has_bits_[0] |= 0x00000008u;
}
inline void GlobalPlannerConfig::clear_has_goal_distance_tolerance() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void GlobalPlannerConfig::clear_goal_distance_tolerance() {
  goal_distance_tolerance_ = 0;
  clear_has_goal_distance_tolerance();
}
inline double GlobalPlannerConfig::goal_distance_tolerance() const {
  // @@protoc_insertion_point(field_get:or_global_planner.GlobalPlannerConfig.goal_distance_tolerance)
  return goal_distance_tolerance_;
}
inline void GlobalPlannerConfig::set_goal_distance_tolerance(double value) {
  set_has_goal_distance_tolerance();
  goal_distance_tolerance_ = value;
  // @@protoc_insertion_point(field_set:or_global_planner.GlobalPlannerConfig.goal_distance_tolerance)
}

// required double goal_angle_tolerance = 6;
inline bool GlobalPlannerConfig::has_goal_angle_tolerance() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void GlobalPlannerConfig::set_has_goal_angle_tolerance() {
  _has_bits_[0] |= 0x00000010u;
}
inline void GlobalPlannerConfig::clear_has_goal_angle_tolerance() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void GlobalPlannerConfig::clear_goal_angle_tolerance() {
  goal_angle_tolerance_ = 0;
  clear_has_goal_angle_tolerance();
}
inline double GlobalPlannerConfig::goal_angle_tolerance() const {
  // @@protoc_insertion_point(field_get:or_global_planner.GlobalPlannerConfig.goal_angle_tolerance)
  return goal_angle_tolerance_;
}
inline void GlobalPlannerConfig::set_goal_angle_tolerance(double value) {
  set_has_goal_angle_tolerance();
  goal_angle_tolerance_ = value;
  // @@protoc_insertion_point(field_set:or_global_planner.GlobalPlannerConfig.goal_angle_tolerance)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace or_global_planner

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_global_5fplanner_5fconfig_2eproto__INCLUDED