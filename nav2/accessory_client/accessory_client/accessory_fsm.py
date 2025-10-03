import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from accessory_msgs.action import AccessoryAction
from astral.sun import sun
from astral import LocationInfo
import datetime
import time
import smach
from accessory_client.utils import debug, info, warning, error, critical
import pytz  # 추가 필요
from std_msgs.msg import String  # 추가 필요
import subprocess

ACCESSORY_COMMANDS = {
    'light': {
        'on': {'target': 'L', 'command': '5', 'color': '0'},
        'off': {'target': 'L', 'command': '0', 'color': '0'}
    },
    'topLed': {
        'on': {'target': 'T', 'command': '1', 'color': 'FF0000'},
        'off': {'target': 'T', 'command': '0', 'color': '0'}
    },
    'bottomLed': {
        'on': {'target': 'B', 'command': '1', 'color': '0'},
        'off': {'target': 'B', 'command': '0', 'color': '0'}
    }
}

CHECK_INTERVAL = 60  # seconds


class AccessoryActionState(smach.State):
    def __init__(self, node: Node, accessory: str):
        smach.State.__init__(self, outcomes=['next'])
        self.node = node
        self.accessory = accessory
        self.last_mode = None  # ✅ 이전 모드 저장
        self.action_client = ActionClient(node, AccessoryAction, 'accessory_action')
        # ros2 topic pub -1 /pm2_command_topic std_msgs/msg/String "data: 'start_pm2'"
        self.pm2_publisher = self.node.create_publisher(String, 'pm2_command_topic', 10)  # ✅ publisher 추가

    def determine_mode(self) -> str:
        city = LocationInfo("Seoul", "South Korea", "Asia/Seoul", 37.5665, 126.9780)
        timezone = pytz.timezone(city.timezone)
        s = sun(city.observer, date=datetime.date.today(), tzinfo=timezone)
        now = datetime.datetime.now(timezone)
        sunset = s['sunset']
        # info(f"[{self.accessory}] 현재 시각: {now.strftime('%H:%M:%S')}, 일몰 시각: {sunset.strftime('%H:%M:%S')}")

        # 20시 이후에는 항상 off
        if now.hour >= 20:
            return 'off'

        return 'on' if now >= sunset else 'off'

    def execute(self, userdata):
        mode = self.determine_mode()

        # if self.last_mode == mode: 계속 전송
        #     info(f"[{self.accessory}] 모드 변동 없음 → '{mode}' 상태 유지 중. 전송 생략.")
        #     time.sleep(CHECK_INTERVAL)
        #     return 'next'

        command = ACCESSORY_COMMANDS[self.accessory][mode]

        goal_msg = AccessoryAction.Goal()
        goal_msg.target = command['target']
        goal_msg.command = command['command']
        goal_msg.color = command['color']

        # Wait for server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            error("액션 서버 연결 실패")
            self.pm2_publisher.publish(String(data="start_pm2"))  # ✅ pm2 재시작 명령 발행
            # try:
            #     cmd = [
            #         'pm2',
            #         'start',
            #         'accessory_client'
            #     ]
            #     subprocess.run(cmd, check=True)
            #     info("✅ PM2 서버 시작 완료")
            # except Exception as e:
            #     error(f"❌ PM2 시작 실패: {e}")
            time.sleep(CHECK_INTERVAL)
            return 'next'

        # Send goal
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            error(f"Goal 거절됨: {self.accessory} {mode}")
            return 'next'

        # Wait result
        # info(f"Goal accepted: {self.accessory} {mode} v1.0")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result

        if result.success:
            # info(f"{self.accessory} {mode} 성공 ✅")
            self.last_mode = mode  # ✅ 모드 갱신
        else:
            error(f"{self.accessory} {mode} 실패 ❌")

        time.sleep(CHECK_INTERVAL)
        return 'next'


def main():
    rclpy.init()
    node = rclpy.create_node('accessory_fsm_cycle')

    sm = smach.StateMachine(outcomes=[])

    with sm:
        smach.StateMachine.add('LIGHT',
                               AccessoryActionState(node, 'light'),
                               transitions={'next': 'TOPLED'})
        smach.StateMachine.add('TOPLED',
                               AccessoryActionState(node, 'topLed'),
                               transitions={'next': 'BOTTOMLED'})
        smach.StateMachine.add('BOTTOMLED',
                               AccessoryActionState(node, 'bottomLed'),
                               transitions={'next': 'LIGHT'})

    sm.execute()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
