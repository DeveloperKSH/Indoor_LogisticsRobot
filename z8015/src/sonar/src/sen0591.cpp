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
#include <sensor_msgs/msg/range.hpp>

// CppLinuxSerial
#include <CppLinuxSerial/SerialPort.hpp>

// Math
#include <cmath> // for M_PI

using namespace std::chrono_literals;
using namespace mn::CppLinuxSerial;

class SEN0591Driver : public rclcpp::Node {
public:
  SEN0591Driver() : Node("sen0591_driver") {

    // ROS 퍼블리셔 생성
    range_pub_front_ = this->create_publisher<sensor_msgs::msg::Range>("sonar/left", 10);
    range_pub_rear_ = this->create_publisher<sensor_msgs::msg::Range>("sonar/right", 10);

    try {
      serial_ = std::make_unique<SerialPort>("/dev/sonar", BaudRate::B_115200);
      RCLCPP_INFO(get_logger(), "/dev/sonar, 115200");
      serial_->SetTimeout(5); // Block for up to 5ms to receive data
      serial_->Open();
      std::this_thread::sleep_for(std::chrono::seconds(2));
      auto state = serial_->GetState();
      if (state == State::OPEN) {
        RCLCPP_INFO(this->get_logger(), "Success Open");
      } else {
        RCLCPP_ERROR(this->get_logger(), "get state: %d", static_cast<int>(state));
      }
    }
    catch (mn::CppLinuxSerial::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }

    // 5초 후에 타이머 시작
    std::this_thread::sleep_for(std::chrono::seconds(2));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 10Hz로 변경 (센서 응답시간 고려)
        std::bind(&SEN0591Driver::timer_callback, this));

    current_sensor_ = 1; // 첫 번째 센서부터 시작
  }

  void serial_close() {
    auto state = serial_->GetState();
    if (state == State::OPEN) {
      serial_->Close();
    }
  }

private:
  // 클래스 내부 변수들
  std::shared_ptr<SerialPort> serial_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_front_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_rear_;
  int current_sensor_; // 현재 읽을 센서 (1 또는 2)

  // CRC16 계산 (Arduino와 동일하게)
  uint16_t calculate_crc16(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
      crc ^= byte;
      for (int i = 8; i != 0; i--) {  // Arduino와 동일하게 i != 0으로 변경
        if ((crc & 0x0001) != 0) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }

    // Arduino와 동일하게 바이트 스왑
    crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
    return crc;
  }

  // 센서에서 거리 읽기 (Arduino 방식으로 수정)
  float read_distance(uint8_t sensor_addr) {
    (void)sensor_addr;

    float distance = 0.0f;

    try {

      std::vector<uint8_t> cmd = {sensor_addr, 0x03, 0x01, 0x01, 0x00, 0x01};
      // CRC16 계산 및 추가 (Arduino 방식)
      uint16_t crc = calculate_crc16(cmd);
      cmd.push_back((crc >> 8) & 0xFF);  // CRC High byte first (Arduino 방식)
      cmd.push_back(crc & 0xFF);         // CRC Low byte
      // 명령 전송
      serial_->WriteBinary(cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(150)); // Arduino의 delay(10)

      // 데이터 읽기
      std::string byte_data;
      serial_->Read(byte_data);

      // 데이터가 있는 경우에만 처리
      if (!byte_data.empty()) {
        // 데이터 출력 (디버그용)
        std::string hex_str;
        for (uint8_t byte : byte_data) {
            char hex[5];
            snprintf(hex, sizeof(hex), "%02x ", byte);
            hex_str += hex;
        }
        // RCLCPP_INFO(this->get_logger(), "Received data: %s", hex_str.c_str());

        // 데이터가 있는 경우에만 처리
        if (!byte_data.empty()) {
          // 데이터 출력 (디버그용)
          std::string hex_str;
          for (uint8_t byte : byte_data) {
            char hex[5];
            snprintf(hex, sizeof(hex), "%02x ", byte);
            hex_str += hex;
          }
          // RCLCPP_INFO(this->get_logger(), "Received data: %s", hex_str.c_str());

          static std::vector<uint8_t> receive_buffer;
          receive_buffer.insert(receive_buffer.end(), byte_data.begin(), byte_data.end());

          const size_t frame_size = 7;
          while (receive_buffer.size() >= frame_size) {
            if (receive_buffer[0] == sensor_addr && receive_buffer[1] == 0x03 && receive_buffer[2] == 0x02) {
              uint16_t crc = calculate_crc16(std::vector<uint8_t>(receive_buffer.begin(), receive_buffer.begin() + 5));
              uint16_t received_crc = (receive_buffer[5] << 8) | receive_buffer[6];
              if (crc == received_crc) {
                int dist = (receive_buffer[3] << 8) | receive_buffer[4];
                distance = static_cast<float>(dist) / 1000.0f;
                // RCLCPP_INFO(this->get_logger(), "✔ Valid distance: %.3f m", distance);
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + frame_size);
                break;
              }
            }
            receive_buffer.erase(receive_buffer.begin()); // Sync 맞추기 위해 버림
          }
        }
      }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading distance: %s", e.what());
    }

    return distance;
  }

  // Range 메시지 생성
  sensor_msgs::msg::Range create_range_msg(float distance, const std::string& frame_id) {
    sensor_msgs::msg::Range msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id;
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = M_PI / 3.0; // 60도 (라디안)
    msg.min_range = 0.02;  // 2cm
    msg.max_range = 3.0;   // 3m
    msg.range = distance;
    return msg;
  }


  void timer_callback() {
    // 센서를 번갈아가며 읽기 (센서 간 간섭 방지)
    if (current_sensor_ == 1) {
      // 첫 번째 센서 읽기 (주소 0x01)
      float distance = read_distance(0x01);
      if (distance > 0) {
        auto msg = create_range_msg(distance, "sonar_left");
        range_pub_front_->publish(msg);
      }
      current_sensor_ = 2;
    } else {
      // 두 번째 센서 읽기 (주소 0x02)
      float distance = read_distance(0x02);
      if (distance > 0) {
        auto msg = create_range_msg(distance, "sonar_right");
        range_pub_rear_->publish(msg);
      }
      current_sensor_ = 1;
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

// on emergency calling
std::shared_ptr<SEN0591Driver> node;
void signalHandler(int signum) {
  std::cout << "Interrupt signal (" << signum << ") received.\n";
  if (node) {
    /* 마무리 코드 */
    std::this_thread::sleep_for(std::chrono::seconds(1));
    node->serial_close();
  }
  rclcpp::shutdown();
  exit(signum);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  node = std::make_shared<SEN0591Driver>();
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  rclcpp::spin(node);

  RCLCPP_INFO(rclcpp::get_logger("sen0591_driver"), "closing...");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  if (node) {
    /* 마무리 코드 */
    std::this_thread::sleep_for(std::chrono::seconds(1));
    node->serial_close();
  }
  return 0;
}