// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: a_star_planner_config.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "a_star_planner_config.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace or_global_planner {
class AStarPlannerConfigDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<AStarPlannerConfig>
     _instance;
} _AStarPlannerConfig_default_instance_;

namespace protobuf_a_5fstar_5fplanner_5fconfig_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[1];

}  // namespace

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTableField
    const TableStruct::entries[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  {0, 0, 0, ::google::protobuf::internal::kInvalidMask, 0, 0},
};

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::AuxillaryParseTableField
    const TableStruct::aux[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ::google::protobuf::internal::AuxillaryParseTableField(),
};
PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTable const
    TableStruct::schema[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { NULL, NULL, 0, -1, -1, -1, -1, NULL, false },
};

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AStarPlannerConfig, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AStarPlannerConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AStarPlannerConfig, inaccessible_cost_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AStarPlannerConfig, heuristic_factor_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AStarPlannerConfig, goal_search_tolerance_),
  1,
  2,
  0,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(AStarPlannerConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_AStarPlannerConfig_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "a_star_planner_config.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

}  // namespace
void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _AStarPlannerConfig_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_AStarPlannerConfig_default_instance_);}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\033a_star_planner_config.proto\022\021or_global"
      "_planner\"v\n\022AStarPlannerConfig\022\036\n\021inacce"
      "ssible_cost\030\001 \001(\r:\003253\022\033\n\020heuristic_fact"
      "or\030\002 \001(\002:\0011\022#\n\025goal_search_tolerance\030\003 \001"
      "(\002:\0040.25"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 168);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "a_star_planner_config.proto", &protobuf_RegisterTypes);
}
} // anonymous namespace

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_a_5fstar_5fplanner_5fconfig_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int AStarPlannerConfig::kInaccessibleCostFieldNumber;
const int AStarPlannerConfig::kHeuristicFactorFieldNumber;
const int AStarPlannerConfig::kGoalSearchToleranceFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

AStarPlannerConfig::AStarPlannerConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_a_5fstar_5fplanner_5fconfig_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:or_global_planner.AStarPlannerConfig)
}
AStarPlannerConfig::AStarPlannerConfig(const AStarPlannerConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&goal_search_tolerance_, &from.goal_search_tolerance_,
    static_cast<size_t>(reinterpret_cast<char*>(&heuristic_factor_) -
    reinterpret_cast<char*>(&goal_search_tolerance_)) + sizeof(heuristic_factor_));
  // @@protoc_insertion_point(copy_constructor:or_global_planner.AStarPlannerConfig)
}

void AStarPlannerConfig::SharedCtor() {
  _cached_size_ = 0;
  goal_search_tolerance_ = 0.25f;
  inaccessible_cost_ = 253u;
  heuristic_factor_ = 1;
}

AStarPlannerConfig::~AStarPlannerConfig() {
  // @@protoc_insertion_point(destructor:or_global_planner.AStarPlannerConfig)
  SharedDtor();
}

void AStarPlannerConfig::SharedDtor() {
}

void AStarPlannerConfig::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* AStarPlannerConfig::descriptor() {
  protobuf_a_5fstar_5fplanner_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_a_5fstar_5fplanner_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const AStarPlannerConfig& AStarPlannerConfig::default_instance() {
  protobuf_a_5fstar_5fplanner_5fconfig_2eproto::InitDefaults();
  return *internal_default_instance();
}

AStarPlannerConfig* AStarPlannerConfig::New(::google::protobuf::Arena* arena) const {
  AStarPlannerConfig* n = new AStarPlannerConfig;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void AStarPlannerConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:or_global_planner.AStarPlannerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 7u) {
    goal_search_tolerance_ = 0.25f;
    inaccessible_cost_ = 253u;
    heuristic_factor_ = 1;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool AStarPlannerConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:or_global_planner.AStarPlannerConfig)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional uint32 inaccessible_cost = 1 [default = 253];
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          set_has_inaccessible_cost();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &inaccessible_cost_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional float heuristic_factor = 2 [default = 1];
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {
          set_has_heuristic_factor();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &heuristic_factor_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional float goal_search_tolerance = 3 [default = 0.25];
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {
          set_has_goal_search_tolerance();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &goal_search_tolerance_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:or_global_planner.AStarPlannerConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:or_global_planner.AStarPlannerConfig)
  return false;
#undef DO_
}

void AStarPlannerConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:or_global_planner.AStarPlannerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 inaccessible_cost = 1 [default = 253];
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(1, this->inaccessible_cost(), output);
  }

  // optional float heuristic_factor = 2 [default = 1];
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->heuristic_factor(), output);
  }

  // optional float goal_search_tolerance = 3 [default = 0.25];
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->goal_search_tolerance(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:or_global_planner.AStarPlannerConfig)
}

::google::protobuf::uint8* AStarPlannerConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:or_global_planner.AStarPlannerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional uint32 inaccessible_cost = 1 [default = 253];
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(1, this->inaccessible_cost(), target);
  }

  // optional float heuristic_factor = 2 [default = 1];
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->heuristic_factor(), target);
  }

  // optional float goal_search_tolerance = 3 [default = 0.25];
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->goal_search_tolerance(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:or_global_planner.AStarPlannerConfig)
  return target;
}

size_t AStarPlannerConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:or_global_planner.AStarPlannerConfig)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 7u) {
    // optional float goal_search_tolerance = 3 [default = 0.25];
    if (has_goal_search_tolerance()) {
      total_size += 1 + 4;
    }

    // optional uint32 inaccessible_cost = 1 [default = 253];
    if (has_inaccessible_cost()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->inaccessible_cost());
    }

    // optional float heuristic_factor = 2 [default = 1];
    if (has_heuristic_factor()) {
      total_size += 1 + 4;
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void AStarPlannerConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:or_global_planner.AStarPlannerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const AStarPlannerConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const AStarPlannerConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:or_global_planner.AStarPlannerConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:or_global_planner.AStarPlannerConfig)
    MergeFrom(*source);
  }
}

void AStarPlannerConfig::MergeFrom(const AStarPlannerConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:or_global_planner.AStarPlannerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      goal_search_tolerance_ = from.goal_search_tolerance_;
    }
    if (cached_has_bits & 0x00000002u) {
      inaccessible_cost_ = from.inaccessible_cost_;
    }
    if (cached_has_bits & 0x00000004u) {
      heuristic_factor_ = from.heuristic_factor_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void AStarPlannerConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:or_global_planner.AStarPlannerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AStarPlannerConfig::CopyFrom(const AStarPlannerConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:or_global_planner.AStarPlannerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AStarPlannerConfig::IsInitialized() const {
  return true;
}

void AStarPlannerConfig::Swap(AStarPlannerConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void AStarPlannerConfig::InternalSwap(AStarPlannerConfig* other) {
  using std::swap;
  swap(goal_search_tolerance_, other->goal_search_tolerance_);
  swap(inaccessible_cost_, other->inaccessible_cost_);
  swap(heuristic_factor_, other->heuristic_factor_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata AStarPlannerConfig::GetMetadata() const {
  protobuf_a_5fstar_5fplanner_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_a_5fstar_5fplanner_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// AStarPlannerConfig

// optional uint32 inaccessible_cost = 1 [default = 253];
bool AStarPlannerConfig::has_inaccessible_cost() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void AStarPlannerConfig::set_has_inaccessible_cost() {
  _has_bits_[0] |= 0x00000002u;
}
void AStarPlannerConfig::clear_has_inaccessible_cost() {
  _has_bits_[0] &= ~0x00000002u;
}
void AStarPlannerConfig::clear_inaccessible_cost() {
  inaccessible_cost_ = 253u;
  clear_has_inaccessible_cost();
}
::google::protobuf::uint32 AStarPlannerConfig::inaccessible_cost() const {
  // @@protoc_insertion_point(field_get:or_global_planner.AStarPlannerConfig.inaccessible_cost)
  return inaccessible_cost_;
}
void AStarPlannerConfig::set_inaccessible_cost(::google::protobuf::uint32 value) {
  set_has_inaccessible_cost();
  inaccessible_cost_ = value;
  // @@protoc_insertion_point(field_set:or_global_planner.AStarPlannerConfig.inaccessible_cost)
}

// optional float heuristic_factor = 2 [default = 1];
bool AStarPlannerConfig::has_heuristic_factor() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void AStarPlannerConfig::set_has_heuristic_factor() {
  _has_bits_[0] |= 0x00000004u;
}
void AStarPlannerConfig::clear_has_heuristic_factor() {
  _has_bits_[0] &= ~0x00000004u;
}
void AStarPlannerConfig::clear_heuristic_factor() {
  heuristic_factor_ = 1;
  clear_has_heuristic_factor();
}
float AStarPlannerConfig::heuristic_factor() const {
  // @@protoc_insertion_point(field_get:or_global_planner.AStarPlannerConfig.heuristic_factor)
  return heuristic_factor_;
}
void AStarPlannerConfig::set_heuristic_factor(float value) {
  set_has_heuristic_factor();
  heuristic_factor_ = value;
  // @@protoc_insertion_point(field_set:or_global_planner.AStarPlannerConfig.heuristic_factor)
}

// optional float goal_search_tolerance = 3 [default = 0.25];
bool AStarPlannerConfig::has_goal_search_tolerance() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void AStarPlannerConfig::set_has_goal_search_tolerance() {
  _has_bits_[0] |= 0x00000001u;
}
void AStarPlannerConfig::clear_has_goal_search_tolerance() {
  _has_bits_[0] &= ~0x00000001u;
}
void AStarPlannerConfig::clear_goal_search_tolerance() {
  goal_search_tolerance_ = 0.25f;
  clear_has_goal_search_tolerance();
}
float AStarPlannerConfig::goal_search_tolerance() const {
  // @@protoc_insertion_point(field_get:or_global_planner.AStarPlannerConfig.goal_search_tolerance)
  return goal_search_tolerance_;
}
void AStarPlannerConfig::set_goal_search_tolerance(float value) {
  set_has_goal_search_tolerance();
  goal_search_tolerance_ = value;
  // @@protoc_insertion_point(field_set:or_global_planner.AStarPlannerConfig.goal_search_tolerance)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace or_global_planner

// @@protoc_insertion_point(global_scope)
