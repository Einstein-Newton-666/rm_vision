// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Recieve.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_Recieve_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_Recieve_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3021000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3021011 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_Recieve_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_Recieve_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_Recieve_2eproto;
namespace device_to_host {
class Frame;
struct FrameDefaultTypeInternal;
extern FrameDefaultTypeInternal _Frame_default_instance_;
}  // namespace device_to_host
PROTOBUF_NAMESPACE_OPEN
template<> ::device_to_host::Frame* Arena::CreateMaybeMessage<::device_to_host::Frame>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace device_to_host {

// ===================================================================

class Frame final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:device_to_host.Frame) */ {
 public:
  inline Frame() : Frame(nullptr) {}
  ~Frame() override;
  explicit PROTOBUF_CONSTEXPR Frame(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Frame(const Frame& from);
  Frame(Frame&& from) noexcept
    : Frame() {
    *this = ::std::move(from);
  }

  inline Frame& operator=(const Frame& from) {
    CopyFrom(from);
    return *this;
  }
  inline Frame& operator=(Frame&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Frame& default_instance() {
    return *internal_default_instance();
  }
  static inline const Frame* internal_default_instance() {
    return reinterpret_cast<const Frame*>(
               &_Frame_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Frame& a, Frame& b) {
    a.Swap(&b);
  }
  inline void Swap(Frame* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Frame* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Frame* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Frame>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Frame& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const Frame& from) {
    Frame::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena, bool is_message_owned);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Frame* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "device_to_host.Frame";
  }
  protected:
  explicit Frame(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kCurrentPitchFieldNumber = 1,
    kCurrentYawFieldNumber = 2,
    kCurrentRollFieldNumber = 3,
    kCurrentColorFieldNumber = 4,
    kBulletSpeedFieldNumber = 5,
    kModeFieldNumber = 6,
    kEnemiesBlood0FieldNumber = 7,
    kEnemiesBlood1FieldNumber = 8,
    kEnemiesBlood2FieldNumber = 9,
    kEnemiesBlood3FieldNumber = 10,
    kEnemiesBlood4FieldNumber = 11,
    kEnemiesBlood5FieldNumber = 12,
    kEnemiesOutpostFieldNumber = 13,
    kIfAttackEngineerFieldNumber = 14,
    kResetTrackerFieldNumber = 15,
  };
  // float Current_Pitch_ = 1;
  void clear_current_pitch_();
  float current_pitch_() const;
  void set_current_pitch_(float value);
  private:
  float _internal_current_pitch_() const;
  void _internal_set_current_pitch_(float value);
  public:

  // float Current_Yaw_ = 2;
  void clear_current_yaw_();
  float current_yaw_() const;
  void set_current_yaw_(float value);
  private:
  float _internal_current_yaw_() const;
  void _internal_set_current_yaw_(float value);
  public:

  // float Current_Roll_ = 3;
  void clear_current_roll_();
  float current_roll_() const;
  void set_current_roll_(float value);
  private:
  float _internal_current_roll_() const;
  void _internal_set_current_roll_(float value);
  public:

  // int32 Current_Color_ = 4;
  void clear_current_color_();
  int32_t current_color_() const;
  void set_current_color_(int32_t value);
  private:
  int32_t _internal_current_color_() const;
  void _internal_set_current_color_(int32_t value);
  public:

  // float Bullet_Speed_ = 5;
  void clear_bullet_speed_();
  float bullet_speed_() const;
  void set_bullet_speed_(float value);
  private:
  float _internal_bullet_speed_() const;
  void _internal_set_bullet_speed_(float value);
  public:

  // int32 Mode_ = 6;
  void clear_mode_();
  int32_t mode_() const;
  void set_mode_(int32_t value);
  private:
  int32_t _internal_mode_() const;
  void _internal_set_mode_(int32_t value);
  public:

  // int32 Enemies_Blood_0 = 7;
  void clear_enemies_blood_0();
  int32_t enemies_blood_0() const;
  void set_enemies_blood_0(int32_t value);
  private:
  int32_t _internal_enemies_blood_0() const;
  void _internal_set_enemies_blood_0(int32_t value);
  public:

  // int32 Enemies_Blood_1 = 8;
  void clear_enemies_blood_1();
  int32_t enemies_blood_1() const;
  void set_enemies_blood_1(int32_t value);
  private:
  int32_t _internal_enemies_blood_1() const;
  void _internal_set_enemies_blood_1(int32_t value);
  public:

  // int32 Enemies_Blood_2 = 9;
  void clear_enemies_blood_2();
  int32_t enemies_blood_2() const;
  void set_enemies_blood_2(int32_t value);
  private:
  int32_t _internal_enemies_blood_2() const;
  void _internal_set_enemies_blood_2(int32_t value);
  public:

  // int32 Enemies_Blood_3 = 10;
  void clear_enemies_blood_3();
  int32_t enemies_blood_3() const;
  void set_enemies_blood_3(int32_t value);
  private:
  int32_t _internal_enemies_blood_3() const;
  void _internal_set_enemies_blood_3(int32_t value);
  public:

  // int32 Enemies_Blood_4 = 11;
  void clear_enemies_blood_4();
  int32_t enemies_blood_4() const;
  void set_enemies_blood_4(int32_t value);
  private:
  int32_t _internal_enemies_blood_4() const;
  void _internal_set_enemies_blood_4(int32_t value);
  public:

  // int32 Enemies_Blood_5 = 12;
  void clear_enemies_blood_5();
  int32_t enemies_blood_5() const;
  void set_enemies_blood_5(int32_t value);
  private:
  int32_t _internal_enemies_blood_5() const;
  void _internal_set_enemies_blood_5(int32_t value);
  public:

  // int32 Enemies_outpost = 13;
  void clear_enemies_outpost();
  int32_t enemies_outpost() const;
  void set_enemies_outpost(int32_t value);
  private:
  int32_t _internal_enemies_outpost() const;
  void _internal_set_enemies_outpost(int32_t value);
  public:

  // int32 IfAttackEngineer = 14;
  void clear_ifattackengineer();
  int32_t ifattackengineer() const;
  void set_ifattackengineer(int32_t value);
  private:
  int32_t _internal_ifattackengineer() const;
  void _internal_set_ifattackengineer(int32_t value);
  public:

  // int32 Reset_Tracker = 15;
  void clear_reset_tracker();
  int32_t reset_tracker() const;
  void set_reset_tracker(int32_t value);
  private:
  int32_t _internal_reset_tracker() const;
  void _internal_set_reset_tracker(int32_t value);
  public:

  // @@protoc_insertion_point(class_scope:device_to_host.Frame)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    float current_pitch__;
    float current_yaw__;
    float current_roll__;
    int32_t current_color__;
    float bullet_speed__;
    int32_t mode__;
    int32_t enemies_blood_0_;
    int32_t enemies_blood_1_;
    int32_t enemies_blood_2_;
    int32_t enemies_blood_3_;
    int32_t enemies_blood_4_;
    int32_t enemies_blood_5_;
    int32_t enemies_outpost_;
    int32_t ifattackengineer_;
    int32_t reset_tracker_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_Recieve_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Frame

// float Current_Pitch_ = 1;
inline void Frame::clear_current_pitch_() {
  _impl_.current_pitch__ = 0;
}
inline float Frame::_internal_current_pitch_() const {
  return _impl_.current_pitch__;
}
inline float Frame::current_pitch_() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Current_Pitch_)
  return _internal_current_pitch_();
}
inline void Frame::_internal_set_current_pitch_(float value) {
  
  _impl_.current_pitch__ = value;
}
inline void Frame::set_current_pitch_(float value) {
  _internal_set_current_pitch_(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Current_Pitch_)
}

// float Current_Yaw_ = 2;
inline void Frame::clear_current_yaw_() {
  _impl_.current_yaw__ = 0;
}
inline float Frame::_internal_current_yaw_() const {
  return _impl_.current_yaw__;
}
inline float Frame::current_yaw_() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Current_Yaw_)
  return _internal_current_yaw_();
}
inline void Frame::_internal_set_current_yaw_(float value) {
  
  _impl_.current_yaw__ = value;
}
inline void Frame::set_current_yaw_(float value) {
  _internal_set_current_yaw_(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Current_Yaw_)
}

// float Current_Roll_ = 3;
inline void Frame::clear_current_roll_() {
  _impl_.current_roll__ = 0;
}
inline float Frame::_internal_current_roll_() const {
  return _impl_.current_roll__;
}
inline float Frame::current_roll_() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Current_Roll_)
  return _internal_current_roll_();
}
inline void Frame::_internal_set_current_roll_(float value) {
  
  _impl_.current_roll__ = value;
}
inline void Frame::set_current_roll_(float value) {
  _internal_set_current_roll_(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Current_Roll_)
}

// int32 Current_Color_ = 4;
inline void Frame::clear_current_color_() {
  _impl_.current_color__ = 0;
}
inline int32_t Frame::_internal_current_color_() const {
  return _impl_.current_color__;
}
inline int32_t Frame::current_color_() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Current_Color_)
  return _internal_current_color_();
}
inline void Frame::_internal_set_current_color_(int32_t value) {
  
  _impl_.current_color__ = value;
}
inline void Frame::set_current_color_(int32_t value) {
  _internal_set_current_color_(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Current_Color_)
}

// float Bullet_Speed_ = 5;
inline void Frame::clear_bullet_speed_() {
  _impl_.bullet_speed__ = 0;
}
inline float Frame::_internal_bullet_speed_() const {
  return _impl_.bullet_speed__;
}
inline float Frame::bullet_speed_() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Bullet_Speed_)
  return _internal_bullet_speed_();
}
inline void Frame::_internal_set_bullet_speed_(float value) {
  
  _impl_.bullet_speed__ = value;
}
inline void Frame::set_bullet_speed_(float value) {
  _internal_set_bullet_speed_(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Bullet_Speed_)
}

// int32 Mode_ = 6;
inline void Frame::clear_mode_() {
  _impl_.mode__ = 0;
}
inline int32_t Frame::_internal_mode_() const {
  return _impl_.mode__;
}
inline int32_t Frame::mode_() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Mode_)
  return _internal_mode_();
}
inline void Frame::_internal_set_mode_(int32_t value) {
  
  _impl_.mode__ = value;
}
inline void Frame::set_mode_(int32_t value) {
  _internal_set_mode_(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Mode_)
}

// int32 Enemies_Blood_0 = 7;
inline void Frame::clear_enemies_blood_0() {
  _impl_.enemies_blood_0_ = 0;
}
inline int32_t Frame::_internal_enemies_blood_0() const {
  return _impl_.enemies_blood_0_;
}
inline int32_t Frame::enemies_blood_0() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Enemies_Blood_0)
  return _internal_enemies_blood_0();
}
inline void Frame::_internal_set_enemies_blood_0(int32_t value) {
  
  _impl_.enemies_blood_0_ = value;
}
inline void Frame::set_enemies_blood_0(int32_t value) {
  _internal_set_enemies_blood_0(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Enemies_Blood_0)
}

// int32 Enemies_Blood_1 = 8;
inline void Frame::clear_enemies_blood_1() {
  _impl_.enemies_blood_1_ = 0;
}
inline int32_t Frame::_internal_enemies_blood_1() const {
  return _impl_.enemies_blood_1_;
}
inline int32_t Frame::enemies_blood_1() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Enemies_Blood_1)
  return _internal_enemies_blood_1();
}
inline void Frame::_internal_set_enemies_blood_1(int32_t value) {
  
  _impl_.enemies_blood_1_ = value;
}
inline void Frame::set_enemies_blood_1(int32_t value) {
  _internal_set_enemies_blood_1(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Enemies_Blood_1)
}

// int32 Enemies_Blood_2 = 9;
inline void Frame::clear_enemies_blood_2() {
  _impl_.enemies_blood_2_ = 0;
}
inline int32_t Frame::_internal_enemies_blood_2() const {
  return _impl_.enemies_blood_2_;
}
inline int32_t Frame::enemies_blood_2() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Enemies_Blood_2)
  return _internal_enemies_blood_2();
}
inline void Frame::_internal_set_enemies_blood_2(int32_t value) {
  
  _impl_.enemies_blood_2_ = value;
}
inline void Frame::set_enemies_blood_2(int32_t value) {
  _internal_set_enemies_blood_2(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Enemies_Blood_2)
}

// int32 Enemies_Blood_3 = 10;
inline void Frame::clear_enemies_blood_3() {
  _impl_.enemies_blood_3_ = 0;
}
inline int32_t Frame::_internal_enemies_blood_3() const {
  return _impl_.enemies_blood_3_;
}
inline int32_t Frame::enemies_blood_3() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Enemies_Blood_3)
  return _internal_enemies_blood_3();
}
inline void Frame::_internal_set_enemies_blood_3(int32_t value) {
  
  _impl_.enemies_blood_3_ = value;
}
inline void Frame::set_enemies_blood_3(int32_t value) {
  _internal_set_enemies_blood_3(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Enemies_Blood_3)
}

// int32 Enemies_Blood_4 = 11;
inline void Frame::clear_enemies_blood_4() {
  _impl_.enemies_blood_4_ = 0;
}
inline int32_t Frame::_internal_enemies_blood_4() const {
  return _impl_.enemies_blood_4_;
}
inline int32_t Frame::enemies_blood_4() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Enemies_Blood_4)
  return _internal_enemies_blood_4();
}
inline void Frame::_internal_set_enemies_blood_4(int32_t value) {
  
  _impl_.enemies_blood_4_ = value;
}
inline void Frame::set_enemies_blood_4(int32_t value) {
  _internal_set_enemies_blood_4(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Enemies_Blood_4)
}

// int32 Enemies_Blood_5 = 12;
inline void Frame::clear_enemies_blood_5() {
  _impl_.enemies_blood_5_ = 0;
}
inline int32_t Frame::_internal_enemies_blood_5() const {
  return _impl_.enemies_blood_5_;
}
inline int32_t Frame::enemies_blood_5() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Enemies_Blood_5)
  return _internal_enemies_blood_5();
}
inline void Frame::_internal_set_enemies_blood_5(int32_t value) {
  
  _impl_.enemies_blood_5_ = value;
}
inline void Frame::set_enemies_blood_5(int32_t value) {
  _internal_set_enemies_blood_5(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Enemies_Blood_5)
}

// int32 Enemies_outpost = 13;
inline void Frame::clear_enemies_outpost() {
  _impl_.enemies_outpost_ = 0;
}
inline int32_t Frame::_internal_enemies_outpost() const {
  return _impl_.enemies_outpost_;
}
inline int32_t Frame::enemies_outpost() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Enemies_outpost)
  return _internal_enemies_outpost();
}
inline void Frame::_internal_set_enemies_outpost(int32_t value) {
  
  _impl_.enemies_outpost_ = value;
}
inline void Frame::set_enemies_outpost(int32_t value) {
  _internal_set_enemies_outpost(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Enemies_outpost)
}

// int32 IfAttackEngineer = 14;
inline void Frame::clear_ifattackengineer() {
  _impl_.ifattackengineer_ = 0;
}
inline int32_t Frame::_internal_ifattackengineer() const {
  return _impl_.ifattackengineer_;
}
inline int32_t Frame::ifattackengineer() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.IfAttackEngineer)
  return _internal_ifattackengineer();
}
inline void Frame::_internal_set_ifattackengineer(int32_t value) {
  
  _impl_.ifattackengineer_ = value;
}
inline void Frame::set_ifattackengineer(int32_t value) {
  _internal_set_ifattackengineer(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.IfAttackEngineer)
}

// int32 Reset_Tracker = 15;
inline void Frame::clear_reset_tracker() {
  _impl_.reset_tracker_ = 0;
}
inline int32_t Frame::_internal_reset_tracker() const {
  return _impl_.reset_tracker_;
}
inline int32_t Frame::reset_tracker() const {
  // @@protoc_insertion_point(field_get:device_to_host.Frame.Reset_Tracker)
  return _internal_reset_tracker();
}
inline void Frame::_internal_set_reset_tracker(int32_t value) {
  
  _impl_.reset_tracker_ = value;
}
inline void Frame::set_reset_tracker(int32_t value) {
  _internal_set_reset_tracker(value);
  // @@protoc_insertion_point(field_set:device_to_host.Frame.Reset_Tracker)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace device_to_host

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_Recieve_2eproto
