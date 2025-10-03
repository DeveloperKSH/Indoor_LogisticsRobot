#ifndef __robot_h__
#define __robot_h__

#include "base.hpp"

namespace uron {
/**
 * <B>Unicyle Robot Basic Implementation</B>
 *
 * 단바퀴 로봇의 기본구현을 위해 필요한 로봇의 기능을 정의한다.
 * 유한상태기계(Finite State Machine, FSM)로 확장된 UnicycleRobotFSM 클래스도 이 단바퀴 로봇 클래스를 상속받는 클래스이다.
 */
    class UnicycleRobot : public PoseCase {
    public:
        /**
         * 생성자
         */
        UnicycleRobot() {
            robotVelocity = Polar(0, 0);
            paramMapBuilder = NULL;
            paramPathPlanner = NULL;
            paramPathFollower = NULL;
            paramMaxVelocity = Polar(0.1, uRON_DEG2RAD(10));
            paramMaxAcceleration = Polar(0.1, uRON_DEG2RAD(10));
        }

        /**
         * 현재 로봇의 속도를 주어진 목표 속도로 제어
         * @param velocity 목표 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
         * @return 속도 제어 성공시 참 값을 실패시 거짓 값을 반환
         */
        virtual bool ControlVelocity(Polar velocity) {
            (void) velocity;
            return false;
        }

        /**
         * 사용할 지도 작성기를 설정
         * @param mapper 지도 작성기의 포인터 (기본값: NULL)
         */
        virtual void SetMapBuilder(MapBuilder *mapper) {
            paramMapBuilder = mapper;
        }

        /**
         * 사용된 지도 작성기를 반환
         * @return 지도 작성기의 포인터
         */
        virtual MapBuilder *GetMapBuilder(void) {
            MapBuilder *mapper = paramMapBuilder;
            return mapper;
        }

        /**
         * 사용할 경로 계획기를 설정
         * @param planner 경로 계획기의 포인터 (기본값: NULL)
         */
        virtual void SetPathPlanner(PathPlanner *planner) {
            paramPathPlanner = planner;
        }

        /**
         * 현재 사용된 경로 계획기를 반환
         * @return 경로 계획기의 포인터
         */
        virtual PathPlanner *GetPathPlanner(void) {
            PathPlanner *planner = paramPathPlanner;
            return planner;
        }

        /**
         * 사용할 경로 추종기를 설정
         * @param follower 경로 추종기의 포인터 (기본값: NULL)
         */
        virtual void SetPathFollower(PathFollower *follower) {
            paramPathFollower = follower;
        }

        /**
         * 현재 사용된 경로 계획기를 반환
         * @return 경로 추종기의 포인터
         */
        virtual PathFollower *GetPathFollower(void) {
            PathFollower *follwer = paramPathFollower;
            return follwer;
        }

        /**
         * 거리 센서 데이터의 스펙 설정
         * @param spec 거리 센서 데이터의 스펙
         * @see RangeDataSpec
         */
        virtual void SetRangeDataSpec(const RangeDataSpec &spec) {
            if (paramMapBuilder != NULL) paramMapBuilder->SetRangeDataSpec(spec);
            if (paramPathFollower != NULL) {
                ObstacleAvoider *pAvoider = paramPathFollower->GetObstacleAvoider();
                if (pAvoider != NULL) pAvoider->SetRangeDataSpec(spec);
            }
        }

        /**
         * 현재 사용된 거리 센서 데이터의 스펙 반환
         * @return 거리 센서 데이터의 스펙
         * @see RangeDataSpec
         */
        virtual const RangeDataSpec *GetRangeDataSpec(void) {
            if (paramMapBuilder != NULL) return paramMapBuilder->GetRangeDataSpec();
            if (paramPathFollower != NULL) {
                ObstacleAvoider *pAvoider = paramPathFollower->GetObstacleAvoider();
                if (pAvoider != NULL) return pAvoider->GetRangeDataSpec();
            }
            return NULL;
        }

        /**
         * 거리 센서 데이터를 입력
         *
         * 객체가 측정되지 않은 곳의 센서 값은 센서 사양에서 설정 -1로 입력
         * @param measurement 거리 센서 데이터의 배열 (단위: [m])
         */
        virtual void SetRangeData(double *measurement) {
            if (paramMapBuilder != NULL) paramMapBuilder->SetRangeData(measurement);
            if (paramPathFollower != NULL) {
                ObstacleAvoider *pAvoider = paramPathFollower->GetObstacleAvoider();
                if (pAvoider != NULL) pAvoider->SetRangeData(measurement);
            }
        }

        /**
         * 현재 입력된 거리 센서 데이터를 반환
         *
         * 데이터를 반환받고자 하는 측에서는 거리 센서 데이터의 크기만큼 미리 메모리 할당을 하여야 함
         *
         * 메모리 할당 예: double* buffer = new double[센서 개수];
         * @param buffer 거리 센서 데이터를 반환받을 버퍼의 포인터 <B>[반환값]</B> (단위: [m])
         * @return TODO (0: ...)
         */
        virtual int GetRangeData(double *buffer) {
            if (paramMapBuilder != NULL) return paramMapBuilder->GetRangeData(buffer);
            if (paramPathFollower != NULL) {
                ObstacleAvoider *pAvoider = paramPathFollower->GetObstacleAvoider();
                if (pAvoider != NULL) pAvoider->GetRangeData(buffer);
            }
            return 0;
        }

        /**
         * 거리 센서 스펙이 제대로 설정되어 거리 센서 데이터 입력 준비 상태인지 반환
         * @return 거리 센서 데이터의 입력 준비 상태
         */
        virtual bool IsConfigured(void) {
            if (paramMapBuilder != NULL) return paramMapBuilder->IsConfigured();
            if (paramPathFollower != NULL) {
                ObstacleAvoider *pAvoider = paramPathFollower->GetObstacleAvoider();
                if (pAvoider != NULL) return pAvoider->IsConfigured();
            }
            return false;
        }

        /**
         * 자세 데이터를 입력
         * @param pose 로봇의 자세 데이터 (거리 단위: [m], 각도 단위: [rad])
         */
        virtual void SetPose(const Pose &pose) {
            PoseCase::SetPose(pose);
            if (paramMapBuilder != NULL) paramMapBuilder->SetPose(poseData);
        }

        /**
         * 로봇의 자세에 주어진 극좌표 회전 데이터 적용
         * @param delta 변화할 회전 정보 (거리 단위: [m], 각도 단위: [rad])
         * @return 적용된 회전 정보
         */
        virtual Polar AddPose(const Polar &delta) {
            Polar movement = PoseCase::AddPose(delta);
            if (paramMapBuilder != NULL) paramMapBuilder->SetPose(poseData);
            return movement;
        }

        /**
         * 로봇의 자세에 주어진 두 자세의 차이를 적용
         * @param preOdo 이동 전의 자세 (거리 단위: [m], 각도 단위: [rad])
         * @param curOdo 이동 후의 자세 (거리 단위: [m], 각도 단위: [rad])
         * @return 적용된 회전 정보
         */
        virtual Polar AddPose(const Pose &preOdo, const Pose &curOdo) {
            Polar movement = PoseCase::AddPose(preOdo, curOdo);
            if (paramMapBuilder != NULL) paramMapBuilder->SetPose(poseData);
            return movement;
        }

        /**
         * 주어진 자세 데이터로 초기화
         * @param pose 자세 데이터 (거리 단위: [m], 각도 단위: [rad])
         */
        virtual void ResetPose(const Pose &pose) {
            PoseCase::ResetPose(pose);
            if (paramMapBuilder != NULL) paramMapBuilder->SetPose(poseData);
        }

        /**
         * 빠른 이동 제어 로봇의 앞쪽 방향
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool MoveForward(void) {
            return ControlVelocity(Polar(paramMaxVelocity.linear, 0));
        }

        /**
         * 빠른 이동 제어 로봇의 뒤쪽 방향
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool MoveBackward(void) {
            return ControlVelocity(Polar(-paramMaxVelocity.linear, 0));
        }

        /**
         * 빠른 이동 제어 로봇의 왼쪽 반시계방향 회전
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool TurnLeft(void) {
            return ControlVelocity(Polar(0, paramMaxVelocity.angular));
        }

        /**
         * 빠른 이동 제어 로봇의 오른쪽 시계방향 회전
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool TurnRight(void) {
            return ControlVelocity(Polar(0, -paramMaxVelocity.angular));
        }

        /**
         * 로봇을 주어진 위치로 이동하도록 이동 (경로 계획 기반 목표 이동 제어)
         * @param target 로봇의 목적지 자세 (거리 단위: [m], 각도 단위: [rad])
         * @return 이동 제어 명령의 성공 여부
         */
        virtual bool GoToPose(Pose target) {
            return GoToPosition(Point(target.x, target.y), true, target.theta);
        }

        /**
         * 로봇 정지
         * @return 정지 명령의 성공 여부 반환
         */
        virtual bool Stop(void) {
            return ControlVelocity(Polar(0, 0));
        }

        /**
         * 이동 명령에 사용할 최대 선속도와 각속도 설정
         * @param velocity 최대 선속도 및 각속도 (선속도 기본값: 0.3, 각속도 기본값: MATH_DEG2RAD(45)) (선속도 단위: [m/s], 각속도 단위: [rad/s])
         * @see MoveForward, MoveBackward, TurnLeft, TurnRight, MoveDelta, TurnDelta
         */
        virtual void SetMaxVelocity(const Polar &velocity) {
            paramMaxVelocity = velocity;
        }

        /**
         * 이동 명령에 사용되는 최대 선속도와 각속도 반환
         * @return 최고 선속도 및 각속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
         * @see MoveForward, MoveBackward, TurnLeft, TurnRight, MoveDelta, TurnDelta
         */
        virtual Polar GetMaxVelocity(void) {
            Polar velocity = paramMaxVelocity;
            return velocity;
        }

        /**
         * 이동 명령에 사용할 최대 선가속도와 각가속도 설정
         * @param acceleration 최대 선가속도 및 각가속도 (선가속도 기본값: 0.3, 각가속도 기본값: MATH_DEG2RAD(45)) (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2])
         * @see MoveForward, MoveBackward, TurnLeft, TurnRight, MoveDelta, TurnDelta
         */
        virtual void SetMaxAcceleration(const Polar &acceleration) {
            paramMaxAcceleration = acceleration;
        }

        /**
         * 이동 명령에 사용되는 최대 선가속도 및 각가속도 반환
         * @return 최대 선가속도 및 각가속도 (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2])
         * @see MoveForward, MoveBackward, TurnLeft, TurnRight, MoveDelta, TurnDelta
         */
        virtual Polar GetMaxAcceleration(void) {
            Polar acceleration = paramMaxAcceleration;
            return acceleration;
        }

        /**
         * 로봇의 현재 속도 설정 (선속도 초기값: 0, 각속도 초기값: 0)
         * @param velocity 로봇의 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
         */
        virtual void SetVelocity(const Polar &velocity) {
            robotVelocity = velocity;
        }

        /**
         * 로봇의 현재 속도 반환
         * @return 로봇의 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
         */
        virtual Polar GetVelocity(void) {
            Polar velocity = robotVelocity;
            return velocity;
        }

        /**
         * 로봇을 현재에서 거리만큼 이동시킴
         * @param delta 이동할 거리 (거리가 양수인 경우 전진, 음수인 경우 후진) (단위: [m])
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool MoveDelta(double delta) = 0;

        /**
         * 로봇을 현재에서 각도만큼 회전
         * @param delta 회전할 각도 (각도가 양수이면 좌회전, 음수인 경우 우회전시계방향 회전) (단위: [rad])
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool TurnDelta(double delta) = 0;

        /**
         * 로봇을 주어진 위치로 이동하도록 이동 (경로 계획 기반 목표 이동 제어)
         * @param target 로봇의 목적 위치 (단위: [m])
         * @param isFinalTurn 최종 위치에서 로봇의 방향을 정해진 방향으로 바라보게 할지 여부 (기본값: false)
         * @param finalAngle 로봇의 최종 방향각 (단위: [rad], 기본값: 0)
         * @param isInitTurn 시작 위치에서 로봇이 회전하여 목적지 방향을 바라보게 할지 여부 (기본값: true)
         * @return 이동 제어 명령의 성공 여부
         */
        virtual bool
        GoToPosition(Point target, bool isFinalTurn = false, double finalAngle = 0, bool isInitTurn = true) = 0;


    protected:
        /** 로봇의 현재 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s]) */
        Polar robotVelocity;

        /** 지도 작성기 */
        MapBuilder *paramMapBuilder;

        /** 경로 계획기 */
        PathPlanner *paramPathPlanner;

        /** 경로 추종기 */
        PathFollower *paramPathFollower;

        /**
         * 이동 명령에 사용할 최대 선속도 및 각속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
         * @see MoveForward, MoveBackward, TurnLeft, TurnRight, MoveDelta, TurnDelta
         */
        Polar paramMaxVelocity;

        /**
         * 이동 명령에 사용할 최대 선가속도 및 각가속도 (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2])
         * @see MoveForward, MoveBackward, TurnLeft, TurnRight, MoveDelta, TurnDelta
         */
        Polar paramMaxAcceleration;
    };

    class UnicycleRobotFSM : public UnicycleRobot {
    public:
        enum {
            STATE_IDLE = 0,
            STATE_STOP,
            STATE_PAUSE,
            STATE_MOVE_FORWARD,
            STATE_MOVE_BACKWARD,
            STATE_TURN_LEFT,
            STATE_TURN_RIGHT,
            STATE_MOVE_DELTA,
            STATE_TURN_DELTA,
            STATE_WAIT_DELTA,
            STATE_PATH_PLANING,
            STATE_PATH_INIT_START,
            STATE_PATH_INIT_WAIT,
            STATE_PATH_INIT_EXIT,
            STATE_PATH_FOLLOWING,
            STATE_PATH_FINAL_START,
            STATE_PATH_FINAL_WAIT,
            STATE_PATH_FINAL_EXIT,
            STATE_OBSTACLE_BLOCK,
            STATE_OBSTACLE_AVOID,

            STATE_NUMBER
        };

        enum {
            ERROR_NONE = 0,
            ERROR_NOT_READY_ROBOT,
            ERROR_NOT_READY_MAP,
            ERROR_NOT_READY_PLANNER,
            ERROR_NOT_READY_FOLLOWER,
            ERROR_CANNOT_GENERATE_PATH,
            ERROR_CANNOT_LOAD_PATH,
            ERROR_CANNOT_RELEASE_PATH,
            ERROR_CANNOT_ALLOCATE_MEMORY,
            ERROR_WRONG_ARGUMENT,
            ERROR_WRONG_RETURN_VALUE,
            ERROR_TIMEOUT_OBSTACLE,
            ERROR_NUMBER
        };

        /**
         * @see SetParamAlignLookahead, SetParamObstacleTimeout
         */
        UnicycleRobotFSM() {
            ResetState();
            paramAlignLookahead = 0.5;
            paramObstacleTimeout = 15.0;
            paramRelMoveMargin.linear = 0.01;
            paramRelMoveMargin.angular = uRON_DEG2RAD(5);
            isBlockMotionComplete = false;
        }

        /**
         * 소멸자
         */
        virtual ~UnicycleRobotFSM() {
            if (IsRunning()) {
                // working...
            }
        }

        /**
         */
        virtual bool StartNavigation(double sec = 0.01) {
            (void) sec;
            ResetState();
            // code here
            return true;
        }

        /**
         * 로봇의 항법 동작 정지
         * @return 항법 정지 성공 여부
         */
        virtual bool StopNavigation(void) {
            Stop();
            // code here
            return true;
        }

        /**
         * 로봇의 항법 동작 준비상태 반환
         * @return 항법 동작 준비
         */
        virtual bool IsReady(void) {
            return false;
        }

        /**
         * 로봇의 항법 동작 상태를 반환
         * @return 로봇의 항법 상태
         * @see STATE_IDLE, STATE_STOP, STATE_PAUSE, STATE_RESUME, STATE_MOVE_FORWARD, STATE_MOVE_BACKWARD, STATE_TURN_LEFT, STATE_TURN_RIGHT, STATE_MOVE_DELTA, STATE_TURN_DELTA, STATE_WAIT_DELTA, STATE_PATH_PLANING, STATE_PATH_INIT_START, STATE_PATH_INIT_WAIT, STATE_PATH_INIT_EXIT, STATE_PATH_FOLLOWING, STATE_PATH_FINAL_START, STATE_PATH_FINAL_WAIT, STATE_PATH_FINAL_EXIT, STATE_OBSTACLE_BLOCK, STATE_MAP_BUILDING
         */
        int GetState(void) {
            int curState = stateMode;
            return curState;
        }

        /**
         * 로봇 정지
         * @return 정지 명령의 성공 여부 반환
         */
        virtual bool Stop(void) {
            LockState();
            stateMode = STATE_STOP;
            bool isSuccess = ControlStop();
            UnlockState();
            return isSuccess;
        }

        /**
         * 로봇의 동작을 일시정지
         * @return 일시 정지 성공의 성공 여부 반환
         * @see Resume
         */
        virtual bool Pause(void) {
            if (!IsRunning()) return false;
            LockState();
            bool isSuccess = (stateMode != STATE_PAUSE);
            if (isSuccess) {
                isPaused = stateMode;
                stateMode = STATE_PAUSE;
                isSuccess = ControlStop();
            }
            UnlockState();
            return isSuccess;
        }

        /**
         * 로봇의 일시 정지 상태를 해제
         * @return 일시 정지 해제 성공의 성공 여부 반환
         */
        virtual bool Resume(void) {
            if (!IsRunning()) return false;
            LockState();
            bool isSuccess = (stateMode == STATE_PAUSE);
            if (isSuccess) stateMode = isPaused;
            UnlockState();
            return isSuccess;
        }

        /**
         * 빠른 이동 제어 로봇의 앞쪽 방향
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool MoveForward(void) {
            if (!IsRunning()) return false;
            LockState();
            stateMode = STATE_MOVE_FORWARD;
            UnlockState();
            return true;
        }

        /**
         * 빠른 이동 제어 로봇의 뒤쪽 방향
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool MoveBackward(void) {
            if (!IsRunning()) return false;
            LockState();
            stateMode = STATE_MOVE_BACKWARD;
            UnlockState();
            return true;
        }

        /**
         * 빠른 이동 제어 로봇의 왼쪽 반시계방향 회전
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool TurnLeft(void) {
            if (!IsRunning()) return false;
            LockState();
            stateMode = STATE_TURN_LEFT;
            UnlockState();
            return true;
        }

        /**
         * 빠른 이동 제어 로봇의 오른쪽 시계방향 회전
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool TurnRight(void) {
            if (!IsRunning()) return false;
            LockState();
            stateMode = STATE_TURN_RIGHT;
            UnlockState();
            return true;
        }

        /**
         * 로봇을 현재에서 거리만큼 이동시킴
         * @param delta 이동할 거리 (거리가 양수인 경우 전진, 음수인 경우 후진) (단위: [m])
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool MoveDelta(double delta) {
            if (!IsRunning()) return false;
            LockState();
            statePose = GetPose();
            stateScalar = delta;
            stateFlag1 = true;
            stateMode = STATE_MOVE_DELTA;
            UnlockState();
            return true;
        }

        /**
         * 로봇을 현재에서 각도만큼 회전
         *
         * 최대 회전 범위는 [-uRON_PI, +uRON_PI)
         * @param delta 회전할 각도 (각도가 양수이면 좌회전, 음수인 경우 우회전시계방향 회전) (단위: [rad])
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool TurnDelta(double delta) {
            if (!IsRunning()) return false;
            LockState();
            Pose pose = GetPose();
            stateScalar = TrimRadianAngle(pose.theta + delta);
            stateFlag1 = true;
            stateMode = STATE_TURN_DELTA;
            UnlockState();
            return true;
        }

        /**
         * 이동 로봇을 현재에서 위치로 이동시킴 이동 (경로 계획 기반 목표 이동 제어)
         * @param target 이동할 로봇의 목적 위치 (단위: [m])
         * @param isFinalTurn 최종적으로 목적 위치에서 로봇의 방향을 방향각(orientation)으로 회전시킬지 여부 (기본값: false)
         * @param finalAngle 로봇의 방향각 (단위: [rad], 기본값: 0)
         * @param isInitTurn 시작 위치에서 로봇이 목적지 쪽을 바라보도록 회전시킬지 여부 (기본값: true)
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool
        GoToPosition(Point target, bool isFinalTurn = false, double finalAngle = 0, bool isInitTurn = true) {
            if (!IsRunning()) return false;
            if (isInitTurn && !ControlStop()) {
                CallbackError(ERROR_NOT_READY_ROBOT);
                return false;
            }

            LockState();
            statePose = Pose(target.x, target.y, finalAngle);
            stateFlag1 = isInitTurn;
            stateFlag2 = isFinalTurn;
            stateMode = STATE_PATH_PLANING;
            UnlockState();
            return true;
        }

        /**
         * 주어진 로봇의 방향으로 회전 (경로 계획 기반 목표 이동 제어)
         * @param theta 글로벌 좌표계 기준 로봇의 방향 (단위: [rad])
         * @return 이동 명령의 성공 여부 반환
         */
        virtual bool RotateToOrientation(double theta) {
            Pose pose = GetPose();
            double delta = TrimRadianAngle(theta - pose.theta);
            return TurnDelta(delta);
        }

        /**
         * 로봇을 현재에서 거리만큼 이동시킴 (blocking function, 이동 완료 후 함수 종료)
         * @param delta 이동할 거리 (거리가 양수인 경우 전진, 음수인 경우 후진) (단위: [m])
         * @return 이동 명령의 성공 여부 반환, 이동 제어 명령이 성공이면 이동 제어 완료 대기
         */
        bool MoveDeltaB(double delta) {
            if (!(this->MoveDelta(delta))) return false;
            isBlockMotionComplete = false;
            // Wait here
            return isBlockMotionComplete;
        }

        /**
         * 로봇을 현재에서 각도만큼 회전 (blocking function, 이동 완료 후 함수 종료)
         *
         * 최대 회전 범위는 [-uRON_PI, +uRON_PI)
         * @param delta 회전할 각도 (각도가 양수이면 좌회전, 음수인 경우 우회전시계방향 회전) (단위: [rad])
         * @return 이동 명령의 성공 여부 반환, 이동 제어 명령이 성공이면 이동 제어 완료 대기
         */
        bool TurnDeltaB(double delta) {
            if (!(this->TurnDelta(delta))) return false;
            isBlockMotionComplete = false;
            // Wait here
            return isBlockMotionComplete;
        }

        /**
         * 이동 로봇을 현재에서 위치로 이동시킴 이동 (경로 계획 기반 목표 이동 제어) (blocking function, 이동 완료 후 함수 종료)
         * @param target 이동할 로봇의 목적 위치 (단위: [m])
         * @param isFinalTurn 해당 위치에서 로봇의 방향을 방향각(orientation)으로 회전시킬지 여부 (기본값: false)
         * @param finalAngle 로봇의 방향각 (단위: [rad], 기본값: 0)
         * @param isInitTurn 시작 위치에서 로봇이 목적지 쪽을 바라보도록 회전시킬지 여부 (기본값: true)
         * @return 이동 명령의 성공 여부 반환, 이동 제어 명령이 성공이면 이동 제어 완료 대기
         */
        bool GoToPositionB(Point target, bool isFinalTurn = false, double finalAngle = 0, bool isInitTurn = true) {
            if (!(this->GoToPosition(target, isFinalTurn, finalAngle, isInitTurn))) return false;
            isBlockMotionComplete = false;
            // Wait here
            return isBlockMotionComplete;
        }

        /**
         * 주어진 로봇의 방향으로 회전 (경로 계획 기반 목표 이동 제어) (blocking function, 이동 완료 후 함수 종료)
         * @param theta 글로벌 좌표계 기준 로봇의 방향 (단위: [rad])
         * @return 이동 명령의 성공 여부 반환, 이동 제어 명령이 성공이면 이동 제어 완료 대기
         */
        bool RotateToOrientationB(double theta) {
            if (!(this->RotateToOrientation(theta))) return false;
            isBlockMotionComplete = false;
            // Wait here
            return isBlockMotionComplete;
        }

        /**
         * 경로 추종 시작시 로봇이 경로에 정렬하여 회전할 때의 사용되는 파라미터를 설정
         * @param lookahead 경로 추종 파라미터 (기본값: 10)
         * @see PurePursuit::SetParamLookahead
         */
        void SetParamAlignLookahead(double lookahead) {
            paramAlignLookahead = lookahead;
        }

        /**
         * 경로 추종 시작시 로봇이 경로에 정렬하여 회전할 때의 사용되는 파라미터를 반환
         * @return 경로 추종 파라미터
         */
        double GetParamAlignLookahead(void) {
            double lookahead = paramAlignLookahead;
            return lookahead;
        }

        /**
         * 장애물에 대한 대기 시간 설정
         * @param timeout 장애물 대기 시간 (기본값: 15) (단위: [sec])
         */
        void SetParamObstacleTimeout(double timeout) {
            paramObstacleTimeout = timeout;
        }

        /**
         * 장애물에 대한 대기 시간 반환
         * @return 장애물 대기 시간 (단위: [sec])
         */
        double GetParamObstacleTimeout(void) {
            double timeout = paramObstacleTimeout;
            return timeout;
        }

        /**
         * 상대 이동 명령의 정확도 설정
         * @param margin 상대적 정확도 (기본값: 0.01 [m], uRON_DEG2RAD(5) [rad]) (거리 단위: [m], 각도 단위: [rad])
         * @see MoveDelta, TurnDelta, MoveDeltaB, TurnDeltaB
         */
        void SetParamRelMoveMargin(const Polar &margin) {
            paramRelMoveMargin.linear = margin.linear;
            paramRelMoveMargin.angular = margin.angular;
        }

        /**
         * 상대 이동 명령의 정확도 반환
         * @return 상대적 정확도 (거리 단위: [m], 각도 단위: [rad])
         */
        Polar GetParamRelMoveMargin(void) {
            Polar margin;
            margin.linear = paramRelMoveMargin.linear;
            margin.angular = paramRelMoveMargin.angular;
            return margin;
        }

    public:
        /**
         * @param elapse  [sec])
         */
        virtual void PeriodicTask(double elapse) {
            // Execute command
            bool isSuccess = true;
            bool isGoal = false;
            int errorCode = ERROR_NONE;
            LockState();
            switch (stateMode) {
                case STATE_STOP:
                    if (IsStop()) {
                        ResetState();
                    } else isSuccess = ControlStop();
                    break;
                case STATE_MOVE_FORWARD:
                    isSuccess = UnicycleRobot::MoveForward();
                    break;
                case STATE_MOVE_BACKWARD:
                    isSuccess = UnicycleRobot::MoveBackward();
                    break;
                case STATE_TURN_LEFT:
                    isSuccess = UnicycleRobot::TurnLeft();
                    break;
                case STATE_TURN_RIGHT:
                    isSuccess = UnicycleRobot::TurnRight();
                    break;
                case STATE_MOVE_DELTA:
                    if (CheckTargetDistance(elapse, errorCode)) {
                        if (stateFlag1) isSuccess = ControlStop();
                        stateMode = STATE_WAIT_DELTA;
                    }
                    break;
                case STATE_TURN_DELTA:
                    if (CheckTargetOrientation(elapse, errorCode)) {
                        if (stateFlag1) isSuccess = ControlStop();
                        stateMode = STATE_WAIT_DELTA;
                    }
                    break;
                case STATE_WAIT_DELTA:
                    if (IsStop()) {
                        ResetState();
                        isBlockMotionComplete = true;
                        isGoal = true;
                    }
                    break;
                case STATE_PATH_PLANING: {
                    if (paramPathPlanner == NULL) {
                        isSuccess = ControlStop();
                        stateMode = STATE_IDLE;
                        errorCode = ERROR_NOT_READY_PLANNER;
                        break;
                    }
                    int length;
                    Point *path = paramPathPlanner->GeneratePath(GetPose(), statePose, length);
                    if (path == NULL || length <= 0) {
                        isSuccess = ControlStop();
                        stateMode = STATE_IDLE;
                        errorCode = ERROR_CANNOT_GENERATE_PATH;
                        break;
                    }
                    if (paramPathFollower == NULL) {
                        isSuccess = ControlStop();
                        stateMode = STATE_IDLE;
                        errorCode = ERROR_NOT_READY_FOLLOWER;
                        break;
                    }
                    int size = paramPathFollower->LoadPath(path, length);
                    if (size != length) {
                        paramPathPlanner->ReleasePath(path);
                        isSuccess = ControlStop();
                        stateMode = STATE_IDLE;
                        errorCode = ERROR_CANNOT_LOAD_PATH;
                        break;
                    }
                    if (stateFlag1) {
                        isSuccess = ControlStop();
                        stateScalar = CalculateAlignAngle(path, length);
                        stateMode = STATE_PATH_INIT_START;
                    } else stateMode = STATE_PATH_FOLLOWING;
                    if (!paramPathPlanner->ReleasePath(path))
                        errorCode = ERROR_CANNOT_RELEASE_PATH;
                }
                    break;
                case STATE_PATH_INIT_START:
                    if (IsStop()) isSuccess = AlignAtPath();
                    break;
                case STATE_PATH_INIT_WAIT:
                    if (CheckTargetOrientation(elapse, errorCode)) {
                        if (stateFlag1) isSuccess = ControlStop();
                        stateMode = STATE_PATH_INIT_EXIT;
                    }
                    break;
                case STATE_PATH_INIT_EXIT:
                    if (IsStop()) stateMode = STATE_PATH_FOLLOWING;
                    break;
                case STATE_PATH_FOLLOWING:
                case STATE_OBSTACLE_BLOCK:
                case STATE_OBSTACLE_AVOID:
                    if (CheckTargetPosition(elapse, errorCode)) {
                        isSuccess = ControlStop();
                        if (stateFlag2) stateMode = STATE_PATH_FINAL_START;
                        else stateMode = STATE_PATH_FINAL_EXIT;
                    }
                    break;
                case STATE_PATH_FINAL_START:
                    if (IsStop()) isSuccess = AlignAtGoal();
                    break;
                case STATE_PATH_FINAL_WAIT:
                    if (CheckTargetOrientation(elapse, errorCode)) {
                        if (stateFlag1) isSuccess = ControlStop();
                        stateMode = STATE_PATH_FINAL_EXIT;
                    }
                    break;
                case STATE_PATH_FINAL_EXIT:
                    if (IsStop()) {
                        ResetState();
                        isBlockMotionComplete = true;
                        isGoal = true;
                    }
            }
            UnlockState();
            if (!isSuccess) CallbackError(ERROR_NOT_READY_ROBOT);
            if (errorCode != ERROR_NONE) CallbackError(errorCode);
            if (isGoal) CallbackGoalComplete();
        }

        /**
         * @param elapse [sec])
         * @return
         * @see MoveDelta
         */
        bool CheckTargetDistance(double elapse, int &errorCode) {

            (void) elapse;
            Pose pose = GetPose();
            Pose start = statePose;
            double deltaX = pose.x - start.x;
            double deltaY = pose.y - start.y;
            double delta = fabs(stateScalar) - sqrt(deltaX * deltaX + deltaY * deltaY);
            Polar currentVelocity = GetVelocity();
            double velocity = uRON_MIN(fabs(currentVelocity.linear), paramMaxVelocity.linear);
            double distStop = 0.5 * velocity * velocity / paramMaxAcceleration.linear;
            if (delta < uRON_MAX(distStop, paramRelMoveMargin.linear)) {
                RCLCPP_INFO(rclcpp::get_logger("CheckTargetDistance #1"), "%.3f", elapse);
                return true; // Near the target distance
            } else {
                // Wait until the target distance
                if (IsStop())
                {
                    // Move again if the robot does not turn
                    delta = delta * uRON_SIGN(stateScalar);
                    if (!ControlMoveDelta(delta)) errorCode = ERROR_NOT_READY_ROBOT;
                }
            }

            return false;
        }

        /**
         * @param elapse [sec])
         * @return
         * @see TurnDelta
         */
        bool CheckTargetOrientation(double elapse, int &errorCode) {
            (void) elapse;
            Pose pose = GetPose();
            double delta = TrimRadianAngle(stateScalar - pose.theta);
            Polar currentVelocity = GetVelocity();
            double velocity = uRON_MIN(fabs(currentVelocity.angular), paramMaxVelocity.angular);
            double angStop = 0.5 * velocity * velocity / paramMaxAcceleration.angular;
            if (fabs(delta) < uRON_MAX(angStop, paramRelMoveMargin.angular)) return true; // Near the target direction
            else {
                // Wait until the target direction
                if (IsStop())
                {
                    // Turn again if the robot does not turn
                    if (!ControlTurnDelta(delta)) errorCode = ERROR_NOT_READY_ROBOT;
                }
            }
            return false;
        }

        /**
         * @see GoToPosition, GoAlongViaPoints
         */
        bool CheckTargetPosition(double elapse, int &errorCode) {
            static int blockingRepetition = 0;

            // 1. Calculate velocity to track the path
            if (paramPathFollower == NULL) {
                errorCode = ERROR_NOT_READY_FOLLOWER;
                return false;
            }
            Pose currentPose = GetPose();
            Polar currentVelocity = GetVelocity();
            Polar controlVelocity;
            int ret = paramPathFollower->FollowPath(currentPose, currentVelocity, elapse, controlVelocity);

            // 2. Control velocity according to situations
            bool isGoal = false;
            if (ret == -3)         // [CASE 3] The robot cannot track the path due to obstacles
            {
                if (!IsStop()) {
                    if (!ControlStop()) errorCode = ERROR_NOT_READY_ROBOT;
                    stateMode = STATE_OBSTACLE_BLOCK;
                    blockingRepetition++;
                    if (blockingRepetition > uRON_FLOOR_POS(paramObstacleTimeout / elapse)) {
                        errorCode = ERROR_TIMEOUT_OBSTACLE;
                        blockingRepetition = 0;
                    }
                }
            } else {
                blockingRepetition = 0;
                if (ret >= 0)       // [CASE 0] The robot follows the path
                {
                    if (!ControlVelocity(controlVelocity)) errorCode = ERROR_NOT_READY_ROBOT;
                    stateMode = STATE_PATH_FOLLOWING;
                } else if (ret == -1) // [CASE 1] The path follower or obstacle avoider are not ready
                {
                    if (!IsStop())
                        if (!ControlStop()) errorCode = ERROR_NOT_READY_ROBOT;
                    errorCode = ERROR_NOT_READY_FOLLOWER;
                } else if (ret == -2) // [CASE 2] The robot arrives the goal
                {
                    if (!IsStop())
                        if (!ControlStop()) errorCode = ERROR_NOT_READY_ROBOT;
                    isGoal = true;
                } else if (ret == -4) // [CASE 4] The robot avoids obstacles
                {
                    if (!ControlVelocity(controlVelocity)) errorCode = ERROR_NOT_READY_ROBOT;
                    stateMode = STATE_OBSTACLE_AVOID;
                } else                // [CASE 5] Unknown return value
                {
                    if (!IsStop())
                        if (!ControlStop()) errorCode = ERROR_NOT_READY_ROBOT;
                    errorCode = ERROR_WRONG_RETURN_VALUE;
                }
            }
            return isGoal;
        }

        void LockState(void) {
            isLocked = stateMode;
        }

        void UnlockState(void) {
            bool isTransit = (isLocked != stateMode);
            if (isTransit) CallbackStateChange(isLocked, stateMode);
        }

        void ResetState(void) {
            stateMode = UnicycleRobotFSM::STATE_IDLE;
            isLocked = UnicycleRobotFSM::STATE_IDLE;
            statePose = Pose(0, 0, 0);
            stateScalar = 0;
            stateFlag1 = false;
            stateFlag2 = false;
        }

        /**
         * @see ERROR_NONE, ERROR_NOT_READY_ROBOT, ERROR_NOT_READY_MAP, ERROR_NOT_READY_PLANNER, ERROR_NOT_READY_FOLLOWER, ERROR_CANNOT_GENERATE_PATH, ERROR_CANNOT_LOAD_PATH, ERROR_CANNOT_RELEASE_PATH, ERROR_CANNOT_ALLOCATE_MEMORY, ERROR_WRONG_ARGUMENT, ERROR_WRONG_RETURN_VALUE, ERROR_TIMEOUT_OBSTACLE
         */
        virtual void CallbackError(int errCode) {
            (void) errCode;
        }

        /**
         */
        virtual void CallbackGoalComplete(void) {

        }

        /**
         * @see STATE_IDLE, STATE_STOP, STATE_MOVE_FORWARD, STATE_MOVE_BACKWARD, STATE_TURN_LEFT, STATE_TURN_RIGHT, STATE_MOVE_DELTA, STATE_TURN_DELTA, STATE_WAIT_DELTA, STATE_PATH_PLANING, STATE_PATH_INIT_START, STATE_PATH_INIT_WAIT, STATE_PATH_INIT_EXIT, STATE_PATH_FOLLOWING, STATE_PATH_FINAL_START, STATE_PATH_FINAL_WAIT, STATE_PATH_FINAL_EXIT, STATE_OBSTACLE_BLOCK, STATE_MAP_BUILDING
         */
        virtual void CallbackStateChange(int preState, int curState) {
            (void) preState;
            (void) curState;
        }

        /**
         * @see ControlVelocity, ControlStop, ControlMoveDelta, ControlTurnDelta
         */
        bool ControlStop(void) {
            return ControlVelocity(Polar(0, 0));
        }

        /**
         * @return
         * @see ControlVelocity, ControlStop, ControlMoveDelta, ControlTurnDelta
         */
        bool ControlMoveDelta(double delta) {
            bool isSuccess = true;
            if (fabs(delta) > uRON_EPSILON_LEN) {
                double linearVel = paramMaxVelocity.linear;
                if (uRON_SIGN(delta) != uRON_SIGN(linearVel)) linearVel *= -1.0;
                isSuccess = ControlVelocity(Polar(linearVel, 0));
                // RCLCPP_INFO(rclcpp::get_logger("ControlMoveDelta"), "%.3f", linearVel);
            }
            return isSuccess;
        }

        /**
         * @see ControlVelocity, ControlStop, ControlMoveDelta, ControlTurnDelta
         */
        bool ControlTurnDelta(double delta) {
            bool isSuccess = true;
            delta = TrimRadianAngle(delta);
            if (fabs(delta) > uRON_EPSILON_ANG) {
                double angularVel = paramMaxVelocity.angular;
                if (uRON_SIGN(delta) != uRON_SIGN(angularVel)) angularVel *= -1.0;
                isSuccess = ControlVelocity(Polar(0, angularVel));
            }
            return isSuccess;
        }

        /**
         */
        bool IsStop(void) {
            Polar velocity = GetVelocity();
            return (fabs(velocity.linear) < uRON_EPSILON_LEN &&
                    fabs(velocity.angular) < uRON_EPSILON_ANG);
        }

        /**
         */
        double CalculateAlignAngle(Point *path, int size) {
            if (size < 2) return 0;
            double deltaX = path[1].x - path[0].x;
            double deltaY = path[1].y - path[0].y;
            double step = sqrt(deltaX * deltaX + deltaY * deltaY);
            int lookahead = uRON_MAX(uRON_FLOOR_POS(paramAlignLookahead / step), 1);
            lookahead = uRON_MIN(lookahead, size - 1);
            return atan2(path[lookahead].y - path[0].y, path[lookahead].x - path[0].x);
        }

        /**
         */
        bool AlignAtPath(void) {
            Pose pose = GetPose();
            stateFlag1 = true;
            stateMode = STATE_PATH_INIT_WAIT;
            return ControlTurnDelta(TrimRadianAngle(stateScalar - pose.theta));
        }

        /**
         */
        bool AlignAtGoal(void) {
            Pose pose = GetPose();
            stateScalar = statePose.theta;
            stateFlag1 = true;
            stateMode = STATE_PATH_FINAL_WAIT;
            return ControlTurnDelta(TrimRadianAngle(stateScalar - pose.theta));
        }

        bool IsRunning() {
            // code here
            return true;    // 이것 자체가 thread 임
        }

        /**
         * 로봇의 항법 동작 상태
         * @see STATE_IDLE, STATE_STOP, STATE_PAUSE, STATE_MOVE_FORWARD, STATE_MOVE_BACKWARD, STATE_TURN_LEFT, STATE_TURN_RIGHT, STATE_MOVE_DELTA, STATE_TURN_DELTA, STATE_WAIT_DELTA, STATE_PATH_PLANING, STATE_PATH_INIT_START, STATE_PATH_INIT_WAIT, STATE_PATH_INIT_EXIT, STATE_PATH_FOLLOWING, STATE_PATH_FINAL_START, STATE_PATH_FINAL_WAIT, STATE_PATH_FINAL_EXIT, STATE_OBSTACLE_BLOCK, STATE_MAP_BUILDING
         */
        int stateMode;

        /** 로봇의 항법 동작 수행에 필요한 정보, 자세 데이터 */
        Pose statePose;

        /** 로봇의 항법 동작 수행에 필요한 정보, 스칼라 데이터 */
        double stateScalar;

        /** 로봇의 항법 동작 수행에 필요한 정보, 부울 플래그 데이터 */
        bool stateFlag1;

        /** 로봇의 항법 동작 수행에 필요한 정보, 부울 플래그 데이터 */
        bool stateFlag2;

        /** 경로 추종 파라미터 */
        double paramAlignLookahead;

        /** 장애물 대기 시간 (단위: [sec]) */
        double paramObstacleTimeout;

        /** 로봇 제어의 정확도 (거리 단위: [m], 각도 단위: [rad]) */
        Polar paramRelMoveMargin;

    private:
        /** 로봇의 상태 변경 중 플래그 */
        int isLocked;

        /** 일시 정지 이전 상태 */
        int isPaused;

        /** Blocking function 완료를 위한 상태 */
        bool isBlockMotionComplete;

    };
}    // end namespace uron

#endif