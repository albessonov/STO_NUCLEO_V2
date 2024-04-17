/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_STO_TESTS_V2_NANOPB_PB_H_INCLUDED
#define PB_STO_TESTS_V2_NANOPB_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _Method {
    Method_SET = 0,
    Method_GET = 1
} Method;

/* Struct definitions */
typedef struct _MCUSettings {
    Method method;
    bool has_uid;
    char uid[13];
    bool has_firmwareVersion;
    uint32_t firmwareVersion; /* ������ �������� ����������, ������ ��� ������ */
    bool has_protobufVersion;
    uint32_t protobufVersion;
    /* ������ protobuf, ������ ��� ������. ������� 2 ����� ��������� �� ������ proto: 2 ��� 3.
������� 2 ����� �� ������, �������������� � ����� �����. */
    bool has_rtcTimestamp;
    uint32_t rtcTimestamp; /* ��������� ����� ����� ��������� ������� */
} MCUSettings;

typedef PB_BYTES_ARRAY_T(8) CanFrame_data_t;
typedef struct _CanFrame {
    uint32_t timestamp; /* unix timestamp - ����� ������� ������ */
    uint32_t id; /* can id */
    uint32_t length; /* ����� ������� � ������ */
    CanFrame_data_t data; /* ������ ������ ������� can, ������ ����� length */
} CanFrame;

typedef struct _TestData {
    Method method; /* ��������� SET ��� ������� �����, ��� ��������� ���������� ������� GET, ��������� ������ �� ���������� �� �������� */
    uint32_t testNumber; /* ����� ������������ ����� ��� �����, ��������� �������� ������ */
    bool has_timeout;
    uint32_t timeout; /* ����� �������� �����, ����� �������� �� ������� ����� � ����������� */
    bool has_accDataNumber;
    uint32_t accDataNumber; /* ����� ������������� ������ ��������� (���������, ����� ����� �� ����� ����� ��������) */
    bool has_seatbelt_number;
    uint32_t seatbelt_number; /* ����� ������������ �����(1-��������;2-�������� ��������;3,4-������ ���������) */
    bool has_vehicle_speed;
    bool vehicle_speed; /* 0-���������� ��������  15 ��/�;1-���������� �������� 40 ��/� */
    bool has_VehicleStateExtended;
    bool VehicleStateExtended; /* ��� ������ SBR: 0-Sleeping ; 1-EngineRunning */
    pb_size_t measuredValue_count;
    double measuredValue[10];
    pb_size_t frame_count;
    CanFrame frame[20];
} TestData;

typedef struct _Message {
    pb_size_t which_s;
    union {
        TestData testData; /* ������ */
        MCUSettings mcuSettings; /* ��������� ���������� */
    } s;
} Message;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _Method_MIN Method_SET
#define _Method_MAX Method_GET
#define _Method_ARRAYSIZE ((Method)(Method_GET+1))


#define MCUSettings_method_ENUMTYPE Method

#define TestData_method_ENUMTYPE Method



/* Initializer values for message structs */
#define Message_init_default                     {0, {TestData_init_default}}
#define MCUSettings_init_default                 {_Method_MIN, false, "", false, 0, false, 0, false, 0}
#define TestData_init_default                    {_Method_MIN, 0, false, 0, false, 0, false, 0, false, 0, false, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default, CanFrame_init_default}}
#define CanFrame_init_default                    {0, 0, 0, {0, {0}}}
#define Message_init_zero                        {0, {TestData_init_zero}}
#define MCUSettings_init_zero                    {_Method_MIN, false, "", false, 0, false, 0, false, 0}
#define TestData_init_zero                       {_Method_MIN, 0, false, 0, false, 0, false, 0, false, 0, false, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, {CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero, CanFrame_init_zero}}
#define CanFrame_init_zero                       {0, 0, 0, {0, {0}}}

/* Field tags (for use in manual encoding/decoding) */
#define MCUSettings_method_tag                   1
#define MCUSettings_uid_tag                      2
#define MCUSettings_firmwareVersion_tag          3
#define MCUSettings_protobufVersion_tag          4
#define MCUSettings_rtcTimestamp_tag             5
#define CanFrame_timestamp_tag                   1
#define CanFrame_id_tag                          2
#define CanFrame_length_tag                      3
#define CanFrame_data_tag                        4
#define TestData_method_tag                      1
#define TestData_testNumber_tag                  2
#define TestData_timeout_tag                     3
#define TestData_accDataNumber_tag               4
#define TestData_seatbelt_number_tag             5
#define TestData_vehicle_speed_tag               6
#define TestData_VehicleStateExtended_tag        7
#define TestData_measuredValue_tag               8
#define TestData_frame_tag                       9
#define Message_testData_tag                     1
#define Message_mcuSettings_tag                  2

/* Struct field encoding specification for nanopb */
#define Message_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    MESSAGE,  (s,testData,s.testData),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (s,mcuSettings,s.mcuSettings),   2)
#define Message_CALLBACK NULL
#define Message_DEFAULT NULL
#define Message_s_testData_MSGTYPE TestData
#define Message_s_mcuSettings_MSGTYPE MCUSettings

#define MCUSettings_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UENUM,    method,            1) \
X(a, STATIC,   OPTIONAL, STRING,   uid,               2) \
X(a, STATIC,   OPTIONAL, UINT32,   firmwareVersion,   3) \
X(a, STATIC,   OPTIONAL, UINT32,   protobufVersion,   4) \
X(a, STATIC,   OPTIONAL, UINT32,   rtcTimestamp,      5)
#define MCUSettings_CALLBACK NULL
#define MCUSettings_DEFAULT NULL

#define TestData_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UENUM,    method,            1) \
X(a, STATIC,   REQUIRED, UINT32,   testNumber,        2) \
X(a, STATIC,   OPTIONAL, UINT32,   timeout,           3) \
X(a, STATIC,   OPTIONAL, UINT32,   accDataNumber,     4) \
X(a, STATIC,   OPTIONAL, UINT32,   seatbelt_number,   5) \
X(a, STATIC,   OPTIONAL, BOOL,     vehicle_speed,     6) \
X(a, STATIC,   OPTIONAL, BOOL,     VehicleStateExtended,   7) \
X(a, STATIC,   REPEATED, DOUBLE,   measuredValue,     8) \
X(a, STATIC,   REPEATED, MESSAGE,  frame,             9)
#define TestData_CALLBACK NULL
#define TestData_DEFAULT NULL
#define TestData_frame_MSGTYPE CanFrame

#define CanFrame_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   timestamp,         1) \
X(a, STATIC,   REQUIRED, UINT32,   id,                2) \
X(a, STATIC,   REQUIRED, UINT32,   length,            3) \
X(a, STATIC,   REQUIRED, BYTES,    data,              4)
#define CanFrame_CALLBACK NULL
#define CanFrame_DEFAULT NULL

extern const pb_msgdesc_t Message_msg;
extern const pb_msgdesc_t MCUSettings_msg;
extern const pb_msgdesc_t TestData_msg;
extern const pb_msgdesc_t CanFrame_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Message_fields &Message_msg
#define MCUSettings_fields &MCUSettings_msg
#define TestData_fields &TestData_msg
#define CanFrame_fields &CanFrame_msg

/* Maximum encoded size of messages (where known) */
#define CanFrame_size                            28
#define MCUSettings_size                         34
#define Message_size                             723
#define TestData_size                            720

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
