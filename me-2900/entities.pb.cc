// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: entities.proto

#include "entities.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace protobuf_entities_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_entities_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Point3d;
}  // namespace protobuf_entities_2eproto
namespace entities {
class Point3dDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Point3d>
      _instance;
} _Point3d_default_instance_;
class PointCloudDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<PointCloud>
      _instance;
} _PointCloud_default_instance_;
}  // namespace entities
namespace protobuf_entities_2eproto {
static void InitDefaultsPoint3d() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::entities::_Point3d_default_instance_;
    new (ptr) ::entities::Point3d();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::entities::Point3d::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_Point3d =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsPoint3d}, {}};

static void InitDefaultsPointCloud() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::entities::_PointCloud_default_instance_;
    new (ptr) ::entities::PointCloud();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::entities::PointCloud::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_PointCloud =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsPointCloud}, {
      &protobuf_entities_2eproto::scc_info_Point3d.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Point3d.base);
  ::google::protobuf::internal::InitSCC(&scc_info_PointCloud.base);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::entities::Point3d, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::entities::Point3d, x_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::entities::Point3d, y_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::entities::Point3d, z_),
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::entities::PointCloud, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::entities::PointCloud, points_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::entities::Point3d)},
  { 8, -1, sizeof(::entities::PointCloud)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::entities::_Point3d_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::entities::_PointCloud_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "entities.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\016entities.proto\022\010entities\"*\n\007Point3d\022\t\n"
      "\001X\030\001 \001(\001\022\t\n\001Y\030\002 \001(\001\022\t\n\001Z\030\003 \001(\001\"/\n\nPointC"
      "loud\022!\n\006Points\030\001 \003(\0132\021.entities.Point3db"
      "\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 127);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "entities.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_entities_2eproto
namespace entities {

// ===================================================================

void Point3d::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Point3d::kXFieldNumber;
const int Point3d::kYFieldNumber;
const int Point3d::kZFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Point3d::Point3d()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_entities_2eproto::scc_info_Point3d.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:entities.Point3d)
}
Point3d::Point3d(const Point3d& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&z_) -
    reinterpret_cast<char*>(&x_)) + sizeof(z_));
  // @@protoc_insertion_point(copy_constructor:entities.Point3d)
}

void Point3d::SharedCtor() {
  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&z_) -
      reinterpret_cast<char*>(&x_)) + sizeof(z_));
}

Point3d::~Point3d() {
  // @@protoc_insertion_point(destructor:entities.Point3d)
  SharedDtor();
}

void Point3d::SharedDtor() {
}

void Point3d::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Point3d::descriptor() {
  ::protobuf_entities_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_entities_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Point3d& Point3d::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_entities_2eproto::scc_info_Point3d.base);
  return *internal_default_instance();
}


void Point3d::Clear() {
// @@protoc_insertion_point(message_clear_start:entities.Point3d)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&z_) -
      reinterpret_cast<char*>(&x_)) + sizeof(z_));
  _internal_metadata_.Clear();
}

bool Point3d::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:entities.Point3d)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double X = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &x_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double Y = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &y_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double Z = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &z_)));
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
  // @@protoc_insertion_point(parse_success:entities.Point3d)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:entities.Point3d)
  return false;
#undef DO_
}

void Point3d::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:entities.Point3d)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double X = 1;
  if (this->x() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->x(), output);
  }

  // double Y = 2;
  if (this->y() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->y(), output);
  }

  // double Z = 3;
  if (this->z() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->z(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:entities.Point3d)
}

::google::protobuf::uint8* Point3d::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:entities.Point3d)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double X = 1;
  if (this->x() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->x(), target);
  }

  // double Y = 2;
  if (this->y() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->y(), target);
  }

  // double Z = 3;
  if (this->z() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->z(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:entities.Point3d)
  return target;
}

size_t Point3d::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:entities.Point3d)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // double X = 1;
  if (this->x() != 0) {
    total_size += 1 + 8;
  }

  // double Y = 2;
  if (this->y() != 0) {
    total_size += 1 + 8;
  }

  // double Z = 3;
  if (this->z() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Point3d::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:entities.Point3d)
  GOOGLE_DCHECK_NE(&from, this);
  const Point3d* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Point3d>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:entities.Point3d)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:entities.Point3d)
    MergeFrom(*source);
  }
}

void Point3d::MergeFrom(const Point3d& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:entities.Point3d)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.x() != 0) {
    set_x(from.x());
  }
  if (from.y() != 0) {
    set_y(from.y());
  }
  if (from.z() != 0) {
    set_z(from.z());
  }
}

void Point3d::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:entities.Point3d)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Point3d::CopyFrom(const Point3d& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:entities.Point3d)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Point3d::IsInitialized() const {
  return true;
}

void Point3d::Swap(Point3d* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Point3d::InternalSwap(Point3d* other) {
  using std::swap;
  swap(x_, other->x_);
  swap(y_, other->y_);
  swap(z_, other->z_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Point3d::GetMetadata() const {
  protobuf_entities_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_entities_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void PointCloud::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int PointCloud::kPointsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

PointCloud::PointCloud()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_entities_2eproto::scc_info_PointCloud.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:entities.PointCloud)
}
PointCloud::PointCloud(const PointCloud& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      points_(from.points_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:entities.PointCloud)
}

void PointCloud::SharedCtor() {
}

PointCloud::~PointCloud() {
  // @@protoc_insertion_point(destructor:entities.PointCloud)
  SharedDtor();
}

void PointCloud::SharedDtor() {
}

void PointCloud::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* PointCloud::descriptor() {
  ::protobuf_entities_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_entities_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const PointCloud& PointCloud::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_entities_2eproto::scc_info_PointCloud.base);
  return *internal_default_instance();
}


void PointCloud::Clear() {
// @@protoc_insertion_point(message_clear_start:entities.PointCloud)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  points_.Clear();
  _internal_metadata_.Clear();
}

bool PointCloud::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:entities.PointCloud)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .entities.Point3d Points = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_points()));
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
  // @@protoc_insertion_point(parse_success:entities.PointCloud)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:entities.PointCloud)
  return false;
#undef DO_
}

void PointCloud::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:entities.PointCloud)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .entities.Point3d Points = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->points_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1,
      this->points(static_cast<int>(i)),
      output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:entities.PointCloud)
}

::google::protobuf::uint8* PointCloud::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:entities.PointCloud)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .entities.Point3d Points = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->points_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->points(static_cast<int>(i)), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:entities.PointCloud)
  return target;
}

size_t PointCloud::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:entities.PointCloud)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated .entities.Point3d Points = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->points_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->points(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void PointCloud::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:entities.PointCloud)
  GOOGLE_DCHECK_NE(&from, this);
  const PointCloud* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const PointCloud>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:entities.PointCloud)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:entities.PointCloud)
    MergeFrom(*source);
  }
}

void PointCloud::MergeFrom(const PointCloud& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:entities.PointCloud)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  points_.MergeFrom(from.points_);
}

void PointCloud::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:entities.PointCloud)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void PointCloud::CopyFrom(const PointCloud& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:entities.PointCloud)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool PointCloud::IsInitialized() const {
  return true;
}

void PointCloud::Swap(PointCloud* other) {
  if (other == this) return;
  InternalSwap(other);
}
void PointCloud::InternalSwap(PointCloud* other) {
  using std::swap;
  CastToBase(&points_)->InternalSwap(CastToBase(&other->points_));
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata PointCloud::GetMetadata() const {
  protobuf_entities_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_entities_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace entities
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::entities::Point3d* Arena::CreateMaybeMessage< ::entities::Point3d >(Arena* arena) {
  return Arena::CreateInternal< ::entities::Point3d >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::entities::PointCloud* Arena::CreateMaybeMessage< ::entities::PointCloud >(Arena* arena) {
  return Arena::CreateInternal< ::entities::PointCloud >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
