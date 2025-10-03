//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <stdint.h>
#include <signal.h>
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// CppLinuxSerial
#include <CppLinuxSerial/SerialPort.hpp>

// Math
#include <cmath> // for M_PI

// crc
#include "crc_check.h"

// serial
#include <serial/serial.h>

// z8015 protocol
#include "z8015_driver_protocol.hpp"
using namespace z8015;

using namespace std::chrono_literals;
using namespace mn::CppLinuxSerial;


// 모터 데이터 구조체
typedef struct {
  int32_t encoder_L = 0;
  int32_t encoder_R = 0;
  double rpm_L = 0.0;
  double rpm_R = 0.0;
} MotorState;

#define MAX_LINEAR_VELOCITY              2.0            // m/s
#define MAX_ANGULAR_VELOCITY             2.0            // rad/s
#define WHEEL_BASE                       0.38           // m
#define WHEEL_RADIUS                     0.0535         // m
#define PLUSE_PER_ROTATION               4096           // 1회전당 펄스 수
#define CIRCUMFERENCE (2 * M_PI * WHEEL_RADIUS)         // 휠 주행거리

class Z8015Driver : public rclcpp::Node {
public:
  Z8015Driver() : Node("z8015_driver") {
    // 클래스 내부 변수 초기화
    device_id_ = 0x01;        // 기본 장치 ID
    debug_mode_ = true;      // 디버그 모드
    initial_rpm_data_ = {0, 0, 0, 0};
    initial_position_data_ = {0, 0, 0, 0};
    last_time_ = this->get_clock()->now();
    V_ = 0.0;
    W_ = 0.0;
    X_ = 0.0;
    Y_ = 0.0;
    accumulated_yaw_ = 0.0;
    last_rot_L_dst_ = 0.0;
    last_rot_R_dst_ = 0.0;
    encoder_base_X_ = 0.0;
    encoder_base_Y_ = 0.0;
    theta_ = 0.0;

    try {
      serial_ = std::make_unique<SerialPort>("/dev/driver", BaudRate::B_115200);
      serial_->SetTimeout(5); // Block for up to 5ms to receive data
      serial_->Open();
      std::this_thread::sleep_for(std::chrono::seconds(2));
      auto state = serial_->GetState();
      if (state == State::OPEN) {
        RCLCPP_INFO(this->get_logger(), "Success Open");

        // 모터 초기 설정
        motor_initialize();
      } else {
        RCLCPP_ERROR(this->get_logger(), "get state: %d", static_cast<int>(state));
      }
      /*
      wood_serial_.setPort("/dev/driver");
      wood_serial_.setBaudrate(115200);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
      wood_serial_.setTimeout(timeout);
      wood_serial_.open();
      wood_serial_.flushInput();
      RCLCPP_INFO(this->get_logger(), "Success Open");
      */
    }
    catch (mn::CppLinuxSerial::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }

    // Subscribe to the laser scan topic
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Z8015Driver::cmdVelCallback, this, std::placeholders::_1));

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);

    // 5초 후에 타이머 시작
    std::this_thread::sleep_for(std::chrono::seconds(2));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&Z8015Driver::timer_callback, this));
                                      
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu/data_raw", 10, std::bind(&Z8015Driver::imuCallback, this, std::placeholders::_1));

  }

  void serial_close() {
    auto state = serial_->GetState();
    if (state == State::OPEN) {
      serial_->Close();
    }
  }

  void motor_power_off() {
    serial_mutex_.unlock();  // 현재 잠금 상태를 해제
    multiple_rpm(0, 0);  // 정지
    motor_disable();     // 비활성화
  }

  /**
  * @brief 모터 활성화
  */
  void motor_enable() {
    if (write_single_register(0x200E, 0x0008) == 0) {
      RCLCPP_INFO(this->get_logger(), "Motor enabled successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable motor");
    }
  }

  /**
  * @brief 모터 비활성화
  */
  void motor_disable() {
    if (write_single_register(0x200E, 0x0007) == 0) {
      RCLCPP_INFO(this->get_logger(), "Motor disabled successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to disable motor");
    }
  }

  /**
  * @brief 모터 긴급정지
  */
  void motor_emergency_stop() {
    if (write_single_register(0x200E, 0x0005) == 0) {
      RCLCPP_INFO(this->get_logger(), "Motor emergency stop successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to emergency stop motor");
    }
  }

  /**
  * @brief 제어 모드 설정
  * @param mode 제어 모드
  *        1: Profile Position (Relative)
  *        2: Profile Position (Absolute)
  *        3: Profile Velocity
  *        4: Profile Torque
  */
  void control_mode(uint8_t mode) {
    uint16_t mode_value;
    const char* mode_name;

    switch(mode) {
      case 1:
        mode_value = 0x0001;
        mode_name = "Relative Position";
        break;
      case 2:
        mode_value = 0x0002;
        mode_name = "Absolute Position";
        break;
      case 3:
        mode_value = 0x0003;
        mode_name = "Velocity";
        break;
      case 4:
        mode_value = 0x0004;
        mode_name = "Torque";
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Invalid control mode: %d (valid: 1-4)", mode);
        return;
    }

    if (write_single_register(0x200D, mode_value) == 0) {
      RCLCPP_INFO(this->get_logger(), "Control mode set to %s", mode_name);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to set control mode");
    }
  }

  /**
   * @brief 시작 속도 설정
   * @param start_rpm 시작 속도 (RPM, 1-250)
   * @param motor_side 모터 선택 (0: 좌측, 1: 우측)
   */
  void starting_speed(uint16_t start_rpm, uint8_t motor_side) {
    if (start_rpm < 1 || start_rpm > 250) {
      RCLCPP_ERROR(this->get_logger(), "Invalid starting speed: %d (valid: 1-250 RPM)", start_rpm);
      return;
    }

    uint16_t reg_addr;
    if (motor_side == 0) {
      reg_addr = 0x2043;  // Left motor starting speed
    } else if (motor_side == 1) {
      reg_addr = 0x2073;  // Right motor starting speed
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid motor side: %d (0: left, 1: right)", motor_side);
      return;
    }

    write_single_register(reg_addr, start_rpm);
  }

  /**
   * @brief 최대 속도 설정
   * @param max_rpm 최대 속도 (RPM, 1-1000)
   */
  void max_speed(uint16_t max_rpm) {
    if (max_rpm < 1 || max_rpm > 1000) {
      RCLCPP_ERROR(this->get_logger(), "Invalid max speed: %d (valid: 1-1000 RPM)", max_rpm);
      return;
    }

    write_single_register(0x2008, max_rpm);
  }

  /**
   * @brief 양쪽 모터 RPM 동시 설정 (Multiple Write 사용)
   * @param left_rpm 좌측 모터 RPM (-3000 ~ +3000)
   * @param right_rpm 우측 모터 RPM (-3000 ~ +3000)
   */
  void multiple_rpm(int16_t left_rpm, int16_t right_rpm) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    // 입력값 검증
    if (left_rpm < -3000 || left_rpm > 3000 || right_rpm < -3000 || right_rpm > 3000) {
      RCLCPP_ERROR(this->get_logger(), "Invalid RPM values. Left: %d, Right: %d (valid range: -3000 to 3000)",
                   left_rpm, right_rpm);
      return;
    }

    if (debug_mode_) {
      RCLCPP_INFO(this->get_logger(), "[target] left_rpm: %d, right_rpm: %d", left_rpm, right_rpm);
    }

    // Multiple Write 요청 구조체 생성
    ModbusWriteMultipleRegistersRequest request;
    request.device_addr = device_id_;
    request.function_code = FunctionCodes::WRITE_MULTIPLE_REGISTERS;  // 0x10
    request.set_register_address(0x2088);  // 시작 주소: Left motor RPM
    request.set_register_count(2);         // 2개 레지스터
    request.byte_count = 4;                // 4바이트 데이터

    // 데이터 설정
    request.set_data_word(0, static_cast<uint16_t>(left_rpm));  // left에 right 값
    request.set_data_word(1, static_cast<uint16_t>(-right_rpm));    // right에 left 값

    // CRC 계산
    size_t frame_size = 7 + request.byte_count + 2;  // header + data + crc
    uint16_t crc = crc16(reinterpret_cast<uint8_t*>(&request), frame_size - 2);
    request.crc_hi = crc & 0xFF;
    request.crc_lo = (crc >> 8) & 0xFF;

    // 요청 데이터 상세 로깅
    /*RCLCPP_INFO(this->get_logger(), "=== 구조체 방식 데이터 ===");
    RCLCPP_INFO(this->get_logger(), "device_addr: 0x%02X", request.device_addr);
    RCLCPP_INFO(this->get_logger(), "function_code: 0x%02X", request.function_code);
    RCLCPP_INFO(this->get_logger(), "register_address: 0x%04X", request.get_register_address());
    RCLCPP_INFO(this->get_logger(), "register_count: %d", request.get_register_count());
    RCLCPP_INFO(this->get_logger(), "byte_count: %d", request.byte_count);
    
    // 데이터 워드 출력
    RCLCPP_INFO(this->get_logger(), "데이터 워드:");
    for (size_t i = 0; i < request.get_register_count(); i++) {
      RCLCPP_INFO(this->get_logger(), "워드[%zu]: 0x%04X", i, request.get_data_word(i));
    }
    
    RCLCPP_INFO(this->get_logger(), "crc_lo: 0x%02X", request.crc_lo);
    RCLCPP_INFO(this->get_logger(), "crc_hi: 0x%02X", request.crc_hi);
    RCLCPP_INFO(this->get_logger(), "전체 프레임 크기: %zu 바이트", frame_size);*/

    // 전송
    try {
      serial_->WriteBinary(to_multiple_vector(request));
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Multiple RPM failed: %s", e.what());
    }

    // 배열 방식 데이터 생성
    uint8_t hex_cmd[13] = {0};
    hex_cmd[0] = device_id_;
    hex_cmd[1] = FunctionCodes::WRITE_MULTIPLE_REGISTERS;
    hex_cmd[2] = 0x20;
    hex_cmd[3] = 0x88;
    hex_cmd[4] = 0x00; //high 8 bits of reg number
    hex_cmd[5] = 0x02; //low 8 bits of reg number
    hex_cmd[6] = 0x04; //number of bytes
    hex_cmd[7] = (left_rpm >> 8) & 0xFF;
    hex_cmd[8] = left_rpm & 0xFF;
    hex_cmd[9] = (right_rpm >> 8) & 0xFF;
    hex_cmd[10] = right_rpm & 0xFF;

    calculate_crc(hex_cmd, 13);

    // 전송
    try {
      // serial_->WriteBinary(std::vector<uint8_t>(hex_cmd, hex_cmd + 13));
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Multiple RPM failed: %s", e.what());
    }

    // 배열 방식 데이터 로깅
    /*RCLCPP_INFO(this->get_logger(), "=== 배열 방식 데이터 ===");
    RCLCPP_INFO(this->get_logger(), "device_addr: 0x%02X", hex_cmd[0]);
    RCLCPP_INFO(this->get_logger(), "function_code: 0x%02X", hex_cmd[1]);
    RCLCPP_INFO(this->get_logger(), "register_address: 0x%04X", (hex_cmd[2] << 8) | hex_cmd[3]);
    RCLCPP_INFO(this->get_logger(), "register_count: %d", (hex_cmd[4] << 8) | hex_cmd[5]);
    RCLCPP_INFO(this->get_logger(), "byte_count: %d", hex_cmd[6]);
    
    // 데이터 워드 출력
    RCLCPP_INFO(this->get_logger(), "데이터 워드:");
    RCLCPP_INFO(this->get_logger(), "워드[0]: 0x%04X", (hex_cmd[7] << 8) | hex_cmd[8]);
    RCLCPP_INFO(this->get_logger(), "워드[1]: 0x%04X", (hex_cmd[9] << 8) | hex_cmd[10]);
    
    RCLCPP_INFO(this->get_logger(), "crc_lo: 0x%02X", hex_cmd[11]);
    RCLCPP_INFO(this->get_logger(), "crc_hi: 0x%02X", hex_cmd[12]);
    RCLCPP_INFO(this->get_logger(), "전체 프레임 크기: 13 바이트");*/

    // 바이트 단위 비교
    /*RCLCPP_INFO(this->get_logger(), "=== 바이트 단위 비교 ===");
    std::vector<uint8_t> struct_data = to_multiple_vector(request);
    for(int i = 0; i < 13; i++) {
      RCLCPP_INFO(this->get_logger(), "바이트[%d]: 구조체=0x%02X, 배열=0x%02X %s", 
          i, struct_data[i], hex_cmd[i], 
          (struct_data[i] == hex_cmd[i]) ? "(일치)" : "(불일치)");
    }*/

    // response
    try {
      // 응답 읽기
      std::string readData;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      serial_->Read(readData);
      if(readData.size() != 8) {
        RCLCPP_ERROR(this->get_logger(), "write_single_register response error %ld", readData.size());
        return ;
      }

      // 응답 데이터 출력
      /*std::string hex_str;
      for (uint8_t i = 0; i < uint8_t(readData.size()); i++) {
        char hex[5];
        snprintf(hex, sizeof(hex), "%02x ", static_cast<unsigned char>(readData[i]));
        hex_str += hex;
      }
      RCLCPP_INFO(this->get_logger(), "[multiple_rpm] %s", hex_str.c_str());*/

      // CRC 체크
      if(crc16(reinterpret_cast<const unsigned char*>(readData.data()), readData.size()) != 0) {
        RCLCPP_ERROR(this->get_logger(), "write_single_register crc error");
        return ;
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Multiple RPM failed: %s", e.what());
    }
  }

  /**
  * @brief 가속 시간 설정
  * @param acc_time_ms 가속 시간 (밀리초, 0-32767)
  * @param motor_side 모터 선택 (0: 좌측, 1: 우측)
  */
  void acceleration_time(uint16_t acc_time_ms, uint8_t motor_side) {
    if (acc_time_ms > 32767) {
      RCLCPP_ERROR(this->get_logger(), "Invalid acceleration time: %d (max: 32767ms)", acc_time_ms);
      return;
    }

    uint16_t reg_addr;
    if (motor_side == 0) {
      reg_addr = 0x2080;  // Left motor acceleration time
    } else if (motor_side == 1) {
      reg_addr = 0x2081;  // Right motor acceleration time
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid motor side: %d (0: left, 1: right)", motor_side);
      return;
    }

    write_single_register(reg_addr, acc_time_ms);
  }

  /**
   * @brief 감속 시간 설정
   * @param decc_time_ms 감속 시간 (밀리초, 0-32767)
   * @param motor_side 모터 선택 (0: 좌측, 1: 우측)
   */
  void deceleration_time(uint16_t decc_time_ms, uint8_t motor_side) {
    if (decc_time_ms > 32767) {
      RCLCPP_ERROR(this->get_logger(), "Invalid deceleration time: %d (max: 32767ms)", decc_time_ms);
      return;
    }

    uint16_t reg_addr;
    if (motor_side == 0) {
      reg_addr = 0x2082;  // Left motor deceleration time
    } else if (motor_side == 1) {
      reg_addr = 0x2083;  // Right motor deceleration time
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid motor side: %d (0: left, 1: right)", motor_side);
      return;
    }

    write_single_register(reg_addr, decc_time_ms);
  }

  /**
   * @brief 현재 RPM 읽기
   * @return MOT_DATA 구조체 (rpm_L, rpm_R 필드 사용)
   */
  MotorState get_rpm() {
    MotorState result;
    std::lock_guard<std::mutex> lock(serial_mutex_);

    // Read 요청 구조체 생성
    ModbusReadRegistersRequest request;
    request.device_addr = device_id_;
    request.function_code = FunctionCodes::READ_HOLDING_REGISTERS;  // 0x03
    request.set_register_address(0x20AB);  // RPM 읽기 시작 주소
    request.set_register_count(2);         // 2개 레지스터 (Left + Right RPM)

    // CRC 계산
    uint16_t crc = crc16(reinterpret_cast<uint8_t*>(&request),
                                 sizeof(request) - 2);
    request.crc_hi = crc & 0xFF;
    request.crc_lo = (crc >> 8) & 0xFF;

    try {
      // 요청 전송
      serial_->WriteBinary(to_vector(request));

      // 응답 읽기
      std::string readData;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      serial_->Read(readData); // read num_bytes
      
      // 응답 데이터 출력
      /*std::string hex_str;
      for (uint8_t i = 0; i < uint8_t(readData.size()); i++) {
        char hex[5];
        snprintf(hex, sizeof(hex), "%02x ", static_cast<unsigned char>(readData[i]));
        hex_str += hex;
      }
      RCLCPP_INFO(this->get_logger(), "[get_rpm] %s", hex_str.c_str());*/

      if (readData.size() == 9) {
        ModbusReadRegistersResponse response;
        response.device_addr = readData[0];
        response.function_code = readData[1];
        response.byte_count = readData[2];
        response.data[0] = readData[3];
        response.data[1] = readData[4];
        response.data[2] = readData[5];
        response.data[3] = readData[6];

        // RPM 데이터 추출 (0.1 RPM 단위)
        result.rpm_L = static_cast<double>(response.get_data_word_signed(0)) / 10.0;
        result.rpm_R = -static_cast<double>(response.get_data_word_signed(1)) / 10.0;

        /*if (debug_mode_) {
          RCLCPP_INFO(this->get_logger(), "Current RPM - Left: %.1f, Right: %.1f",
                       result.rpm_L, result.rpm_R);
        }*/
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to read position - invalid response size %ld", readData.size());
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "RPM read failed: %s", e.what());
    }

    return result;
  }

  /**
   * @brief 현재 엔코더 위치 읽기
   * @return MOT_DATA 구조체 (encoder_L, encoder_R 필드 사용)
   */
  MotorState get_position() {
    MotorState result;
    std::lock_guard<std::mutex> lock(serial_mutex_);
    // Read 요청 구조체 생성 (32비트 위치값이므로 4개 레지스터 필요)
    ModbusReadRegistersRequest request;
    request.device_addr = device_id_;
    request.function_code = FunctionCodes::READ_HOLDING_REGISTERS;  // 0x03
    request.set_register_address(0x20A7);  // 엔코더 위치 시작 주소
    request.set_register_count(4);         // 4개 레지스터 (Left 32bit + Right 32bit)

    // CRC 계산
    uint16_t crc = crc16(reinterpret_cast<uint8_t*>(&request),
                                 sizeof(request) - 2);
    request.crc_hi = crc & 0xFF;
    request.crc_lo = (crc >> 8) & 0xFF;

    try {
      // 요청 전송
      serial_->WriteBinary(to_vector(request));

      // 응답 읽기
      std::string readData;
      std::this_thread::sleep_for(std::chrono::milliseconds(25));

      serial_->Read(readData); // read num_bytes
      // 응답 데이터 출력
      /*std::string hex_str;
      for (uint8_t i = 0; i < uint8_t(readData.size()); i++) {
        char hex[5];
        snprintf(hex, sizeof(hex), "%02x ", static_cast<unsigned char>(readData[i]));
        hex_str += hex;
      }
      RCLCPP_INFO(this->get_logger(), "[get_position] %s", hex_str.c_str());*/

      if (readData.size() == 13) {
        ModbusReadRegistersResponse response;
        response.device_addr = readData[0];
        response.function_code = readData[1];
        response.byte_count = readData[2];
        response.data[0] = readData[3];
        response.data[1] = readData[4];
        response.data[2] = readData[5];
        response.data[3] = readData[6];
        response.data[4] = readData[7];
        response.data[5] = readData[8];
        response.data[6] = readData[9];
        response.data[7] = readData[10];
        response.crc_hi = readData[11];
        response.crc_lo = readData[12];

        // 32비트 엔코더 값 추출
        result.encoder_L = response.get_data_dword_signed(0);   // 첫 번째 32비트
        result.encoder_R = -response.get_data_dword_signed(1);   // 두 번째 32비트

        /*if (debug_mode_) {
          RCLCPP_INFO(this->get_logger(), "Current Position - Left: %d, Right: %d",
                       result.encoder_L, result.encoder_R);
        }*/
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to read position - invalid response size %ld", readData.size());
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Position read failed: %s", e.what());
    }

    return result;
  }

  /**
   * @brief 간단한 단일 레지스터 쓰기 함수
   * @param reg_addr 레지스터 주소 (예: 0x200E)
   * @param value 쓸 값 (예: 0x0008)
   * @return 0: 성공, 1: 실패
  */
  uint8_t write_single_register(uint16_t reg_addr, uint16_t value) {
    // 1. 요청 프레임 생성
    ModbusWriteSingleRegisterRequest request;
    request.device_addr = device_id_;
    request.function_code = FunctionCodes::WRITE_SINGLE_REGISTER;  // 0x06
    request.set_register_address(reg_addr);
    request.set_register_data(value);

    // 2. CRC 계산
    uint16_t crc = crc16(reinterpret_cast<uint8_t*>(&request),
                                 sizeof(request) - 2);
    request.crc_hi = crc & 0xFF;
    request.crc_lo = (crc >> 8) & 0xFF;

    try {
      serial_->WriteBinary(to_vector(request));
      
      // 응답 읽기
      std::string readData;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      serial_->Read(readData);
      if(readData.size() != 8) {
        RCLCPP_ERROR(this->get_logger(), "write_single_register response error %ld", readData.size());
        return 1;
      }

      // 응답 데이터 출력
      std::string hex_str;
      for (uint8_t i = 0; i < uint8_t(readData.size()); i++) {
        char hex[5];
        snprintf(hex, sizeof(hex), "%02x ", static_cast<unsigned char>(readData[i]));
        hex_str += hex;
      }
      RCLCPP_INFO(this->get_logger(), "[write_single_register] %s", hex_str.c_str());

      // CRC 체크
      if(crc16(reinterpret_cast<const unsigned char*>(readData.data()), readData.size()) != 0) {
        RCLCPP_ERROR(this->get_logger(), "write_single_register crc error");
        return 1;
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
      return 1;
    }
    return 0;
  }

  std::vector<uint8_t> to_vector(const ModbusWriteSingleRegisterRequest& request) {
    std::vector<uint8_t> data;
    data.push_back(request.device_addr);
    data.push_back(request.function_code);
    data.push_back(request.reg_addr_hi);
    data.push_back(request.reg_addr_lo);
    data.push_back(request.reg_data_hi);
    data.push_back(request.reg_data_lo);
    data.push_back(request.crc_hi);
    data.push_back(request.crc_lo);
    return data;
  }

  std::vector<uint8_t> to_vector(const ModbusReadRegistersRequest& request) {
    std::vector<uint8_t> data;
    data.push_back(request.device_addr);
    data.push_back(request.function_code);
    data.push_back(request.reg_addr_hi);
    data.push_back(request.reg_addr_lo);
    data.push_back(request.reg_count_hi);
    data.push_back(request.reg_count_lo);
    data.push_back(request.crc_hi);
    data.push_back(request.crc_lo);
    return data;
  }

  template<typename T>
  std::vector<uint8_t> to_vector(const T& request) {
    if constexpr (std::is_same_v<T, ModbusWriteSingleRegisterRequest>) {
      return to_vector(static_cast<const ModbusWriteSingleRegisterRequest&>(request));
    } else if constexpr (std::is_same_v<T, ModbusReadRegistersRequest>) {
      return to_vector(static_cast<const ModbusReadRegistersRequest&>(request));
    }
    return std::vector<uint8_t>();
  }

  std::vector<uint8_t> to_multiple_vector(const ModbusWriteMultipleRegistersRequest& request) {
    std::vector<uint8_t> data;
    data.push_back(request.device_addr);
    data.push_back(request.function_code);
    data.push_back(request.reg_addr_hi);
    data.push_back(request.reg_addr_lo);
    data.push_back(request.reg_count_hi);
    data.push_back(request.reg_count_lo);
    data.push_back(request.byte_count);
    
    // 실제 사용하는 레지스터 수만큼만 데이터 추가
    for (size_t i = 0; i < request.get_register_count(); i++) {
      data.push_back(request.data[i * 2]);
      data.push_back(request.data[i * 2 + 1]);
    }
    
    data.push_back(request.crc_hi);
    data.push_back(request.crc_lo);

    // for(size_t i = 0; i < data.size(); i++) {
    //   RCLCPP_INFO(this->get_logger(), "바이트[%zu]: 0x%02X", i, data[i]);
    // }
    return data;
  }

  /**
   * @brief 모터 초기화 설정
   */
  void motor_initialize() {
    RCLCPP_INFO(this->get_logger(), "Initializing motor...");

    // 1. 속도 제어 모드 설정
    control_mode(3);  // Velocity mode
    RCLCPP_INFO(this->get_logger(), "control_mode: %d", 3);

    // 2. 최대 속도 설정
    max_speed(1000);  // 1000 RPM
    RCLCPP_INFO(this->get_logger(), "max_speed: %d", 1000);

    // 3. 가속/감속 시간 설정
    acceleration_time(1000, 0);   // 좌측 모터 500ms
    acceleration_time(1000, 1);   // 우측 모터 500ms
    deceleration_time(300, 0);   // 좌측 모터 500ms
    deceleration_time(300, 1);   // 우측 모터 500ms

    RCLCPP_INFO(this->get_logger(), "acceleration_time: %d, %d", 1000, 1000);
    RCLCPP_INFO(this->get_logger(), "deceleration_time: %d, %d", 300, 300);

    // 4. 시작 속도 설정
    starting_speed(10, 0);       // 좌측 모터 10 RPM
    starting_speed(10, 1);       // 우측 모터 10 RPM

    RCLCPP_INFO(this->get_logger(), "starting_speed: %d, %d", 10, 10);

    // 5. 모터 활성화
    motor_enable();


    multiple_rpm(0, 0);
    RCLCPP_INFO(this->get_logger(), "multiple_rpm: %d, %d", 0, 0);

    initial_rpm_data_ = get_rpm();
    initial_position_data_ = get_position();

    RCLCPP_INFO(this->get_logger(), "Motor initialization complete");
  }

  /**
   * @brief 디버그 모드 설정
   */
  void set_debug_mode(bool enable) {
    debug_mode_ = enable;
    if (enable) {
      RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
    } else {
      RCLCPP_INFO(this->get_logger(), "Debug mode disabled");
    }
  }

  void calculate_crc(uint8_t* hex_cmd, uint8_t length){
    // calculate crc and append to hex cmd
    unsigned short result = crc16(hex_cmd, length - 2);
    hex_cmd[length - 2] = result & 0xFF;
    hex_cmd[length - 1] = (result >> 8) & 0xFF;
  }

private:

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    float linear = msg->linear.x;
    linear = constrain(linear, (-1) * MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    float angular = msg->angular.z;
    angular = constrain(angular, (-1) * MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    // cmd_vel을 RPM으로 변환하여 모터 제어
    // 예시: linear.x와 angular.z를 사용하여 차등 구동
    double linear_vel = linear;   // m/s
    double angular_vel = angular; // rad/s

    // 간단한 차등 구동 계산 (로봇의 휠베이스에 따라 조정 필요)
    double wheel_base = WHEEL_BASE;  // 휠베이스 (미터)
    double wheel_radius = WHEEL_RADIUS; // 휠 반지름 (미터)

    // 좌우 휠 속도 계산 (m/s)
    double left_wheel_vel = linear_vel - (angular_vel * wheel_base / 2.0);
    double right_wheel_vel = linear_vel + (angular_vel * wheel_base / 2.0);

    // m/s를 RPM으로 변환
    double left_rpm = (left_wheel_vel / (2 * M_PI * wheel_radius)) * 60.0;
    double right_rpm = (right_wheel_vel / (2 * M_PI * wheel_radius)) * 60.0;

    // RPM 제한 (-3000 ~ 3000)
    left_rpm = std::max(-3000.0, std::min(3000.0, left_rpm));
    right_rpm = std::max(-3000.0, std::min(3000.0, right_rpm));

    // 모터 제어
    multiple_rpm(static_cast<int16_t>(left_rpm), static_cast<int16_t>(right_rpm));
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
    // 현재 시간 가져오기
    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();

    // 각속도(z)를 사용하여 회전 변화량 계산
    double delta_yaw = imu_msg->angular_velocity.z * dt; // Δyaw = ω * Δt

    // 회전 변화량 누적
    accumulated_yaw_ += delta_yaw;

    // Yaw 변화량 출력
    // RCLCPP_INFO(this->get_logger(), "Current Yaw Change: %f degrees", delta_yaw * (180.0 / M_PI));
    // RCLCPP_INFO(this->get_logger(), "Accumulated Yaw: %f degrees", accumulated_yaw_ * (180.0 / M_PI));

    // update
    double delta_translation = dt * V_;
    X_ += delta_translation * std::cos(accumulated_yaw_ + (delta_yaw / 2.0));
    Y_ += delta_translation * std::sin(accumulated_yaw_ + (delta_yaw / 2.0));

    // RCLCPP_INFO(this->get_logger(), "[imu] v: %.3f, x: %.3f, y: %.3f", V_, X_, Y_);

    auto odometry_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odometry_msg->header.frame_id = "odom";
    odometry_msg->child_frame_id = "base_link";
    odometry_msg->twist.twist.linear.x = V_;
    odometry_msg->twist.twist.angular.z = imu_msg->angular_velocity.z;
    odometry_msg->pose.pose.position.x = X_; // odom_pose.x는 로봇의 x 좌표 위치
    odometry_msg->pose.pose.position.y = Y_; // odom_pose.y는 로봇의 y 좌표 위치
    odometry_msg->pose.pose.position.z = 0.0; // 대부분의 지상 로봇의 경우 z 좌표는 0으로 설정됩니다.

    // RCLCPP_INFO(this->get_logger(), "z: %.3f W_: %.3f X_: %.3f Y_: %.3f", imu_msg->angular_velocity.z, W_, X_, Y_);

    tf2::Quaternion orientation_quat;
    orientation_quat.setRPY(0, 0, accumulated_yaw_); // 롤(Roll), 피치(Pitch), 요우(Yaw)
    odometry_msg->pose.pose.orientation = tf2::toMsg(orientation_quat);

    odometry_msg->header.stamp = this->now();
    odometry_publisher_->publish(*odometry_msg);

    // RCLCPP_INFO(this->get_logger(), "accumulated_yaw_: %.3f", accumulated_yaw_);

    // 마지막 시간 업데이트
    last_time_ = current_time;
  }

  void timer_callback() {
    
    /*static auto last_print_time = this->now();
    static int callback_count = 0;
    auto current_time = this->now();*/

    auto state = serial_->GetState();
    if (state != State::OPEN) {
      RCLCPP_ERROR(this->get_logger(), "Serial state error: %d", static_cast<int>(state));
      return;
    }
    // 주기적으로 모터 상태 읽기 및 오도메트리 발행
    [[maybe_unused]] MotorState rpm_data = get_rpm();
    [[maybe_unused]] MotorState position_data = get_position();  // imu base x,y 검증용

    // RCLCPP_INFO(this->get_logger(), "rpm_data: %.3f, %.3f", rpm_data.rpm_L, rpm_data.rpm_R);
    // RCLCPP_INFO(this->get_logger(), "position_data: %d, %d", position_data.encoder_L, position_data.encoder_R);

    V_ = ((rpm_data.rpm_L + rpm_data.rpm_R) / 2) / 60.0 * CIRCUMFERENCE;
    W_ = ((rpm_data.rpm_R - rpm_data.rpm_L) / 60.0 * CIRCUMFERENCE) / WHEEL_BASE;

    // RCLCPP_INFO(this->get_logger(), "V_: %.3f, W_: %.3f", V_, W_);


    int32_t encoder_diff_L = position_data.encoder_L - initial_position_data_.encoder_L;
    int32_t encoder_diff_R = position_data.encoder_R - initial_position_data_.encoder_R;

    // RCLCPP_INFO(this->get_logger(), "encoder_diff_L: %d, encoder_diff_R: %d", encoder_diff_L, encoder_diff_R);

    double rot_L_dst = (double(encoder_diff_L) / PLUSE_PER_ROTATION) * CIRCUMFERENCE;
    double rot_R_dst = (double(encoder_diff_R) / PLUSE_PER_ROTATION) * CIRCUMFERENCE;

    // 변화량만 추출
    double delta_L = rot_L_dst - last_rot_L_dst_;
    double delta_R = rot_R_dst - last_rot_R_dst_;

    double delta_s = (delta_L + delta_R) / 2.0;
    double delta_theta = (delta_R - delta_L) / WHEEL_BASE;

    // 누적
    theta_ += delta_theta;
    encoder_base_X_ += delta_s * std::cos(theta_);
    encoder_base_Y_ += delta_s * std::sin(theta_);

    // RCLCPP_INFO(this->get_logger(), "[encoder] x: %.3f, y: %.3f", encoder_base_X_, encoder_base_Y_);

    // 상태 갱신
    last_rot_L_dst_ = rot_L_dst;
    last_rot_R_dst_ = rot_R_dst;

    // 1초마다 한 번씩 실행 주기 출력
    /*callback_count++;
    if ((current_time - last_print_time).seconds() >= 1.0) {
        RCLCPP_INFO(this->get_logger(), "rpm & position frequency: %d Hz", callback_count);
        callback_count = 0;
        last_print_time = current_time;
    }*/
  }

  float constrain(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;

    // ignore very small velocity
    if (fabs(value) < 0.001f)
      return 0.0f;

    return value;
  }

private:
  // 클래스 내부 변수들
  uint8_t device_id_;           // 장치 ID
  bool debug_mode_;             // 디버그 모드
  std::shared_ptr<SerialPort> serial_;
  std::mutex serial_mutex_;  // 시리얼 통신을 위한 mutex

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  serial::Serial wood_serial_;

  double last_yaw_;
  rclcpp::Time last_time_;
  double V_;
  double W_;
  double theta_;
  double X_;
  double Y_;
  bool q_base_set_;
  tf2::Quaternion q_base_;
  double accumulated_yaw_;

  MotorState initial_rpm_data_;
  MotorState initial_position_data_;
  double last_rot_L_dst_;
  double last_rot_R_dst_;
  double encoder_base_X_;
  double encoder_base_Y_;

};

// on emergency calling
std::shared_ptr<Z8015Driver> node;
void signalHandler(int signum) {
  std::cout << "Interrupt signal (" << signum << ") received.\n";
  if (node) {
    node->motor_power_off();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    node->serial_close();
  }
  rclcpp::shutdown();
  exit(signum);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  node = std::make_shared<Z8015Driver>();
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  // 디버그 모드 설정 (필요시)
  node->set_debug_mode(true);

  rclcpp::spin(node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "closing...");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  if (node) {
    node->motor_power_off();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    node->serial_close();
  }
  return 0;
}