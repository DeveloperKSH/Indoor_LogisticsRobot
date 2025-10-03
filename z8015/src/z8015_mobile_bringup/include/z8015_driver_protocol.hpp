// z8015_driver_protocol.hpp
#ifndef Z8015_DRIVER_PROTOCOL_HPP
#define Z8015_DRIVER_PROTOCOL_HPP

#include <cstdint>

namespace z8015 {

// 최대 레지스터 수 제한
  constexpr uint8_t MAX_WRITE_REGISTERS = 4;  // Write Multiple용 최대 레지스터 수
  constexpr uint8_t MAX_READ_REGISTERS = 8;   // Read Multiple용 최대 레지스터 수

// Modbus 함수 코드
  struct FunctionCodes {
    static constexpr uint8_t READ_HOLDING_REGISTERS = 0x03;
    static constexpr uint8_t WRITE_SINGLE_REGISTER = 0x06;
    static constexpr uint8_t WRITE_MULTIPLE_REGISTERS = 0x10;
  };

// ZLAC8015D 레지스터 주소 맵
  struct RegisterMap {
    // 제어 레지스터 (0x200X)
    static constexpr uint16_t CONTROL_MODE = 0x200D;      // 제어 모드 설정
    static constexpr uint16_t CONTROL_WORD = 0x200E;      // 모터 제어 명령
    static constexpr uint16_t MAX_SPEED = 0x2008;         // 최대 속도 설정

    // 속도 제어 레지스터 (0x208X)
    static constexpr uint16_t SET_L_RPM = 0x2088;         // 좌측 모터 RPM 설정
    static constexpr uint16_t SET_R_RPM = 0x2089;         // 우측 모터 RPM 설정

    // 가속도/감속도 시간 레지스터 (0x208X)
    static constexpr uint16_t SET_L_ACC_TIME = 0x2080;    // 좌측 모터 가속 시간
    static constexpr uint16_t SET_R_ACC_TIME = 0x2081;    // 우측 모터 가속 시간
    static constexpr uint16_t SET_L_DECC_TIME = 0x2082;   // 좌측 모터 감속 시간
    static constexpr uint16_t SET_R_DECC_TIME = 0x2083;   // 우측 모터 감속 시간

    // 초기 속도 레지스터
    static constexpr uint16_t INITIAL_L_VEL = 0x2043;     // 좌측 모터 초기 속도
    static constexpr uint16_t INITIAL_R_VEL = 0x2073;     // 우측 모터 초기 속도

    // 상태 읽기 레지스터 (0x20AX)
    static constexpr uint16_t GET_RPM_L = 0x20AB;         // 좌측 모터 현재 RPM
    static constexpr uint16_t GET_RPM_R = 0x20AC;         // 우측 모터 현재 RPM
    static constexpr uint16_t GET_ENCODER_L_HIGH = 0x20A7; // 좌측 엔코더 상위 16비트
    static constexpr uint16_t GET_ENCODER_L_LOW = 0x20A8;  // 좌측 엔코더 하위 16비트
    static constexpr uint16_t GET_ENCODER_R_HIGH = 0x20A9; // 우측 엔코더 상위 16비트
    static constexpr uint16_t GET_ENCODER_R_LOW = 0x20AA;  // 우측 엔코더 하위 16비트
    static constexpr uint16_t GET_CURRENT_L = 0x20AD;     // 좌측 모터 현재 전류
    static constexpr uint16_t GET_CURRENT_R = 0x20AE;     // 우측 모터 현재 전류
    static constexpr uint16_t GET_ERROR_L = 0x20A5;       // 좌측 모터 에러 코드
    static constexpr uint16_t GET_ERROR_R = 0x20A6;       // 우측 모터 에러 코드
    static constexpr uint16_t STATUS_WORD = 0x20A2;       // 드라이버 상태 워드
  };

// 제어 명령 값들
  struct ControlValues {
    static constexpr uint16_t MOTOR_ENABLE = 0x0008;      // 모터 활성화
    static constexpr uint16_t MOTOR_DISABLE = 0x0007;     // 모터 비활성화
    static constexpr uint16_t EMERGENCY_STOP = 0x0005;    // 비상 정지
    static constexpr uint16_t CLEAR_FAULT = 0x0006;       // 오류 클리어
    static constexpr uint16_t START_MOTION = 0x0010;      // 동작 시작

    // 제어 모드
    static constexpr uint16_t PROFILE_POSITION_RELATIVE = 0x0001;
    static constexpr uint16_t PROFILE_POSITION_ABSOLUTE = 0x0002;
    static constexpr uint16_t PROFILE_VELOCITY = 0x0003;
    static constexpr uint16_t PROFILE_TORQUE = 0x0004;
  };

// 에러 코드 정의
  struct ErrorCodes {
    static constexpr uint16_t NO_ERROR = 0x0000;
    static constexpr uint16_t OVER_VOLTAGE = 0x0001;
    static constexpr uint16_t UNDER_VOLTAGE = 0x0002;
    static constexpr uint16_t OVER_CURRENT = 0x0004;
    static constexpr uint16_t OVERLOAD = 0x0008;
    static constexpr uint16_t CURRENT_OUT_OF_TOLERANCE = 0x0010;
    static constexpr uint16_t ENCODER_OUT_OF_TOLERANCE = 0x0020;
    static constexpr uint16_t SPEED_OUT_OF_TOLERANCE = 0x0040;
    static constexpr uint16_t REFERENCE_VOLTAGE_ERROR = 0x0080;
    static constexpr uint16_t EEPROM_ERROR = 0x0100;
    static constexpr uint16_t HALL_ERROR = 0x0200;
    static constexpr uint16_t MOTOR_TEMPERATURE_HIGH = 0x0400;
    static constexpr uint16_t ENCODER_ERROR = 0x0800;
    static constexpr uint16_t SPEED_SETTING_ERROR = 0x2000;
  };

// 통신 파라미터
  struct CommParams {
    static constexpr uint32_t DEFAULT_BAUDRATE = 115200;
    static constexpr uint8_t DEFAULT_DEVICE_ID = 0x01;
    static constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;
    static constexpr uint8_t MIN_DEVICE_ID = 0x01;
    static constexpr uint8_t MAX_DEVICE_ID = 0x7F;  // 127
  };

// 모터 사양 제한값
  struct MotorLimits {
    static constexpr int16_t MIN_RPM = -3000;
    static constexpr int16_t MAX_RPM = 3000;
    static constexpr uint16_t MIN_ACC_TIME_MS = 0;
    static constexpr uint16_t MAX_ACC_TIME_MS = 32767;
    static constexpr uint16_t MIN_MAX_SPEED_RPM = 1;
    static constexpr uint16_t MAX_MAX_SPEED_RPM = 1000;
  };

#pragma pack(push, 1) // 구조체를 1바이트 정렬하여 Modbus RTU 프레임과 일치시키기 위함

// Write Multiple Registers Request Frame (Function Code 0x10)
// 여러 개의 레지스터에 데이터를 쓰기 위한 요청 프레임 구조
  struct ModbusWriteMultipleRegistersRequest {
    uint8_t device_addr;     // 슬레이브 주소
    uint8_t function_code;   // 함수 코드 (0x10)
    uint8_t reg_addr_hi;     // 시작 레지스터 주소 (상위 바이트)
    uint8_t reg_addr_lo;     // 시작 레지스터 주소 (하위 바이트)
    uint8_t reg_count_hi;    // 쓰려는 레지스터 개수 (상위 바이트)
    uint8_t reg_count_lo;    // 쓰려는 레지스터 개수 (하위 바이트)
    uint8_t byte_count;      // 데이터 바이트 수 (reg_count * 2)
    uint8_t data[MAX_WRITE_REGISTERS * 2]; // 실제 레지스터 데이터 (2바이트 * 개수)
    uint8_t crc_hi;          // CRC 상위 바이트
    uint8_t crc_lo;          // CRC 하위 바이트

    // 편의 함수들
    void set_register_address(uint16_t addr) {
      reg_addr_hi = (addr >> 8) & 0xFF;
      reg_addr_lo = addr & 0xFF;
    }

    uint16_t get_register_address() const {
      return (static_cast<uint16_t>(reg_addr_hi) << 8) | reg_addr_lo;
    }

    void set_register_count(uint16_t count) {
      reg_count_hi = (count >> 8) & 0xFF;
      reg_count_lo = count & 0xFF;
    }

    uint16_t get_register_count() const {
      return (static_cast<uint16_t>(reg_count_hi) << 8) | reg_count_lo;
    }

    void set_data_word(uint8_t index, uint16_t value) {
      if (index < MAX_WRITE_REGISTERS) {
        data[index * 2] = (value >> 8) & 0xFF;
        data[index * 2 + 1] = value & 0xFF;
      }
    }

    uint16_t get_data_word(uint8_t index) const {
      if (index < MAX_WRITE_REGISTERS) {
        return (static_cast<uint16_t>(data[index * 2]) << 8) | data[index * 2 + 1];
      }
      return 0;
    }
  };

// Write Multiple Registers Response Frame
// 요청 성공에 대한 응답 프레임
  struct ModbusWriteMultipleRegistersResponse {
    uint8_t device_addr;     // 슬레이브 주소
    uint8_t function_code;   // 함수 코드 (0x10)
    uint8_t reg_addr_hi;     // 시작 레지스터 주소 (상위 바이트)
    uint8_t reg_addr_lo;     // 시작 레지스터 주소 (하위 바이트)
    uint8_t reg_count_hi;    // 쓰여진 레지스터 개수 (상위 바이트)
    uint8_t reg_count_lo;    // 쓰여진 레지스터 개수 (하위 바이트)
    uint8_t crc_hi;          // CRC 상위 바이트
    uint8_t crc_lo;          // CRC 하위 바이트

    uint16_t get_register_address() const {
      return (static_cast<uint16_t>(reg_addr_hi) << 8) | reg_addr_lo;
    }

    uint16_t get_register_count() const {
      return (static_cast<uint16_t>(reg_count_hi) << 8) | reg_count_lo;
    }
  };

// Write Single Register Request Frame (Function Code 0x06)
// 하나의 레지스터에 데이터를 쓰기 위한 요청 프레임 구조
  struct ModbusWriteSingleRegisterRequest {
    uint8_t device_addr;     // 슬레이브 주소
    uint8_t function_code;   // 함수 코드 (0x06)
    uint8_t reg_addr_hi;     // 레지스터 주소 (상위 바이트)
    uint8_t reg_addr_lo;     // 레지스터 주소 (하위 바이트)
    uint8_t reg_data_hi;     // 쓰려는 데이터 (상위 바이트)
    uint8_t reg_data_lo;     // 쓰려는 데이터 (하위 바이트)
    uint8_t crc_hi;          // CRC 상위 바이트
    uint8_t crc_lo;          // CRC 하위 바이트

    // 편의 함수들
    void set_register_address(uint16_t addr) {
      reg_addr_hi = (addr >> 8) & 0xFF;
      reg_addr_lo = addr & 0xFF;
    }

    uint16_t get_register_address() const {
      return (static_cast<uint16_t>(reg_addr_hi) << 8) | reg_addr_lo;
    }

    void set_register_data(uint16_t data) {
      reg_data_hi = (data >> 8) & 0xFF;
      reg_data_lo = data & 0xFF;
    }

    uint16_t get_register_data() const {
      return (static_cast<uint16_t>(reg_data_hi) << 8) | reg_data_lo;
    }
  };

// Write Single Register Response Frame
// 요청 성공 시 동일한 형식으로 응답
  using ModbusWriteSingleRegisterResponse = ModbusWriteSingleRegisterRequest;

// Read Multiple Registers Request Frame (Function Code 0x03)
// 여러 개의 레지스터 값을 읽기 위한 요청 프레임 구조
  struct ModbusReadRegistersRequest {
    uint8_t device_addr;     // 슬레이브 주소
    uint8_t function_code;   // 함수 코드 (0x03)
    uint8_t reg_addr_hi;     // 시작 레지스터 주소 (상위 바이트)
    uint8_t reg_addr_lo;     // 시작 레지스터 주소 (하위 바이트)
    uint8_t reg_count_hi;    // 읽을 레지스터 개수 (상위 바이트)
    uint8_t reg_count_lo;    // 읽을 레지스터 개수 (하위 바이트)
    uint8_t crc_hi;          // CRC 상위 바이트
    uint8_t crc_lo;          // CRC 하위 바이트

    // 편의 함수들
    void set_register_address(uint16_t addr) {
      reg_addr_hi = (addr >> 8) & 0xFF;
      reg_addr_lo = addr & 0xFF;
    }

    uint16_t get_register_address() const {
      return (static_cast<uint16_t>(reg_addr_hi) << 8) | reg_addr_lo;
    }

    void set_register_count(uint16_t count) {
      reg_count_hi = (count >> 8) & 0xFF;
      reg_count_lo = count & 0xFF;
    }

    uint16_t get_register_count() const {
      return (static_cast<uint16_t>(reg_count_hi) << 8) | reg_count_lo;
    }
  };

// Read Multiple Registers Response Frame
// 요청 성공 시 반환되는 응답 데이터 프레임
  struct ModbusReadRegistersResponse {
    uint8_t device_addr;     // 슬레이브 주소
    uint8_t function_code;   // 함수 코드 (0x03)
    uint8_t byte_count;      // 전체 데이터 바이트 수 (reg_count * 2)
    uint8_t data[MAX_READ_REGISTERS * 2]; // 실제 읽은 데이터
    uint8_t crc_hi;          // CRC 상위 바이트
    uint8_t crc_lo;          // CRC 하위 바이트

    // 편의 함수들
    uint16_t get_data_word(uint8_t index) const {
      if (index * 2 + 1 < byte_count) {
        return (static_cast<uint16_t>(data[index * 2]) << 8) | data[index * 2 + 1];
      }
      return 0;
    }

    int16_t get_data_word_signed(uint8_t index) const {
      return static_cast<int16_t>(get_data_word(index));
    }

    uint32_t get_data_dword(uint8_t index) const {
      if (index * 4 + 3 < byte_count) {
        return (static_cast<uint32_t>(data[index * 4]) << 24) |
               (static_cast<uint32_t>(data[index * 4 + 1]) << 16) |
               (static_cast<uint32_t>(data[index * 4 + 2]) << 8) |
               static_cast<uint32_t>(data[index * 4 + 3]);
      }
      return 0;
    }

    int32_t get_data_dword_signed(uint8_t index) const {
      return static_cast<int32_t>(get_data_dword(index));
    }
  };

// 오류 응답 프레임
  struct ModbusErrorResponse {
    uint8_t device_addr;     // 슬레이브 주소
    uint8_t function_code;   // 함수 코드 | 0x80 (오류 플래그)
    uint8_t exception_code;  // 예외 코드
    uint8_t crc_hi;          // CRC 상위 바이트
    uint8_t crc_lo;          // CRC 하위 바이트

    // Modbus 예외 코드
    static constexpr uint8_t ILLEGAL_FUNCTION = 0x01;
    static constexpr uint8_t ILLEGAL_DATA_ADDRESS = 0x02;
    static constexpr uint8_t ILLEGAL_DATA_VALUE = 0x03;
    static constexpr uint8_t SLAVE_DEVICE_FAILURE = 0x04;
    static constexpr uint8_t ACKNOWLEDGE = 0x05;
    static constexpr uint8_t SLAVE_DEVICE_BUSY = 0x06;
  };

#pragma pack(pop) // 구조체 패킹 해제

// 유틸리티 함수들
  namespace utils {
    /**
     * @brief 16비트 값을 high/low 바이트로 분할
     */
    inline std::pair<uint8_t, uint8_t> split_uint16(uint16_t value) {
      return {static_cast<uint8_t>((value >> 8) & 0xFF),
              static_cast<uint8_t>(value & 0xFF)};
    }

    /**
     * @brief high/low 바이트를 16비트 값으로 결합
     */
    inline uint16_t combine_uint16(uint8_t high, uint8_t low) {
    return (static_cast<uint16_t>(high) << 8) | low;
  }

  /**
   * @brief 값이 유효한 범위 내에 있는지 확인
   */
  inline bool is_valid_rpm(int16_t rpm) {
    return rpm >= MotorLimits::MIN_RPM && rpm <= MotorLimits::MAX_RPM;
  }

  inline bool is_valid_device_id(uint8_t id) {
  return id >= CommParams::MIN_DEVICE_ID && id <= CommParams::MAX_DEVICE_ID;
}

inline bool is_valid_max_speed(uint16_t rpm) {
  return rpm >= MotorLimits::MIN_MAX_SPEED_RPM && rpm <= MotorLimits::MAX_MAX_SPEED_RPM;
}
}

} // namespace z8015

#endif // Z8015_DRIVER_PROTOCOL_HPP