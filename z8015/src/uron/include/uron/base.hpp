
#ifndef __base_hpp__
#define __base_hpp__

#include <Eigen/Eigen>
	
/** \addtogroup uRON_Math */
/** @{ */

/** 원주율 */
#define uRON_PI                         (3.14159265358979323846)

/**
 * 주어진 라디안 각도를 도 단위로 변환하는 매크로 함수
 * @param x 변환하고자 하는 라디안 각도 (단위: [rad])
 * @return 도 단위 각도 (단위: [도])
 * @see MATH_DEG2RAD
 */
#define uRON_RAD2DEG(x)                 ((x) * 180.0 / uRON_PI)

/**
 * 주어진 도 단위 각도를 라디언 단위로 변환하는 매크로 함수
 * @param x 변환하고자 하는 도 각도 (단위: [도])
 * @return 라디언 단위 각도 (단위: [rad])
 * @see MATH_DEG2RAD
 */
#define uRON_DEG2RAD(x)                 ((x) * uRON_PI / 180.0)

/**
 * 주어진 두 값에서 최대값을 반환하는 매크로 함수
 * @param a 첫 번째 값
 * @param b 두 번째 값
 * @return 두 값 중 최대값
 * @see MATH_MIN
 */
#define uRON_MAX(a, b)                  ( ((a) > (b)) ? (a) : (b) )

/**
 * 주어진 두 값에서 최소값을 반환하는 매크로 함수
 * @param a 첫 번째 값
 * @param b 두 번째 값
 * @return 두 값 중 최소값
 * @see MATH_MAX
 */
#define uRON_MIN(a, b)                  ( ((a) < (b)) ? (a) : (b) )

/**
 * 주어진 값의 기호를 반환하는 매크로 함수
 * @param x 기호를 검사할 값
 * @return 양수인 경우 +1, 0인 경우 0, 음수인 경우 -1
 */
#define uRON_SIGN(x)                    ( ((x) > 0) ? +1 : (((x) < 0) ? -1 : 0) )

/**
 * 주어진 값이 양수 또는 0인 경우의 반올림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값 (x >= 0)
 * @return 반올림된 값 (타입: int)
 */
#define uRON_ROUND_POS(x)               (static_cast<int>((x) + 0.5))

/**
 * 주어진 값이 음수인 경우의 반올림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값 (x < 0)
 * @return 반올림된 값 (타입: int)
 */
#define uRON_ROUND_NEG(x)               (static_cast<int>((x) - 0.5))

/**
 * 주어진 값의 반올림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값
 * @return 반올림된 값 (타입: int)
 */
#define uRON_ROUND(x)                   ( (x) > 0 ? uRON_ROUND_POS(x) : uRON_ROUND_NEG(x) )

/**
 * 주어진 값의 올림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값
 * @return 반올림된 값 (타입: int)
 */
#define uRON_CEIL_POS(x)                ( (x - static_cast<int>(x)) > 0 ? static_cast<int>((x) + 1) : static_cast<int>(x) )

/**
 * 주어진 값이 음수 또는 0인 경우 올림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값 (x <= 0)
 * @return 반올림된 값 (타입: int)
 */
#define uRON_CEIL_NEG(x)                (static_cast<int>(x))

/**
 * 주어진 값의 올림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값
 * @return 반올림된 값 (타입: int)
 */
#define uRON_CEIL(x)                    uRON_CEIL_POS(x)

/**
 * 주어진 값이 양수 또는 0인 경우 올림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값 (x >= 0)
 * @return 반올림된 값 (타입: int)
 */
#define uRON_FLOOR_POS(x)               (static_cast<int>(x))

/**
 * 주어진 값의 내림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값
 * @return 반올림된 값 (타입: int)
 */
#define uRON_FLOOR_NEG(x)               ( (x - static_cast<int>(x)) < 0 ? static_cast<int>((x) - 1) : static_cast<int>(x) )

/**
 * 주어진 값의 내림 값을 반환하는 매크로 함수
 * @param x 반올림 할 값
 * @return 반올림된 값 (타입: int)
 */
#define uRON_FLOOR(x)                   uRON_FLOOR_NEG(x)
/** @} */

/** 거리 및 선속도의 최소값의 크기 (단위: [m]) */
#define uRON_EPSILON_LEN                (0.003)

/** 각도 및 각속도의 최소값의 크기 (단위: [rad]) */
#define uRON_EPSILON_ANG                (uRON_DEG2RAD(3.0))

namespace uron {
/**
 * <B>Matrix</B>
 *
 * uRON에서 사용하는 <B>행렬</B>의 구현이다.
 * TODO
 */
typedef Eigen::MatrixXd Matrix;


// Global Functions
/**
 * 주어진 라디언 각도를 [-MATH_PI, +MATH_PI)의 범위로 변환
 * @param rad 주어진 라디언 각도 (단위: [rad])
 * @return 변환된 라디언 각도 (단위: [rad])
 */
static double TrimRadianAngle(double rad){
	rad = rad - static_cast<int>(rad / (2 * uRON_PI)) * (2 * uRON_PI);
	if (rad >= + uRON_PI) rad = rad - 2 * uRON_PI;
	if (rad <  -uRON_PI) rad = rad + 2 * uRON_PI;
	return rad;
}

/**
 * <B>2D Vector(Point) in Polar Coordinates</B>
 *
 * 이차원 극좌표계(polar coordinates)에서의 벡터를 표현한다.
 * 선형 성분과 각 성분의 값을 가진다.
 * 이차원 평면에서의 로봇 좌표계에서의 <B>속도(velocity)</B>와 <B>가속도(acceleration)</B>를 표현하는데 주로 사용된다.
 *
 * @see Point 이차원 직교좌표계(rectangular coordinates)에서의 벡터
 */
class Polar
{
public:
    /**
     * 생성자 (기본값: 0)
     */
    Polar() { linear = angular = 0; }

    /**
     * 선형 성분과 각 성분의 값을 초기화하는 생성자
     * @param linear 선형 성분의 값
     * @param angular 각 성분의 값
     */
    Polar(double linear, double angular)
    {
        this->linear = linear;
        this->angular = angular;
    }

    /**
     * 등가 비교
     * @param rhs 등가 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 같음 여부
     */
    bool operator==(const Polar& rhs) const { return (linear == rhs.linear) && (angular == rhs.angular); }

    /**
     * 상등 비교
     * @param rhs 상등 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 다름 여부
     */
    bool operator!=(const Polar& rhs) const { return (linear != rhs.linear) || (angular != rhs.angular); }

    /** 선형 성분의 값 */
    double linear;

    /** 각 성분의 값 */
    double angular;
};

/**
 * <B>2D Vector(Point) in Rectangular Coordinates</B>
 *
 * 이차원 직교좌표계(rectangular coordinates)에서의 벡터를 표현한다.
 * X 축과 Y 축의 값을 가진다.
 * 이차원 평면에서 <B>위치(position)</B>를 표현하는데 주로 사용된다.
 *
 * @see Polar 이차원 극좌표계(polar coordinates)에서의 벡터
 */
class Point
{
public:
    /**
     * 생성자 (기본값: 0)
     */
    Point() { x = y = 0; }

    /**
     * X 축과 Y 축 값을 초기화하는 생성자
     * @param x X 축의 값
     * @param y Y 축의 값
     */
    Point(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    /**
     * 등가 비교
     * @param rhs 등가 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 같음 여부
     */
    bool operator==(const Point& rhs) const { return (x == rhs.x) && (y == rhs.y); }

    /**
     * 상등 비교
     * @param rhs 상등 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 다름 여부
     */
    bool operator!=(const Point& rhs) const { return (x != rhs.x) || (y != rhs.y); }

    /** X 축 값 */
    double x;

    /** Y 축 값 */
    double y;
};

/**
 * <B>2D Pose in Rectangular Coordinates</B>
 *
 * 이차원 평면 위에 있는 이차원 강체(rigid body)의 자세(pose)를 표현한다.
 * X 축과 Y 축의 값과 Z 축 방향의 회전에 대한 값을 가진다.
 * 이차원 평면에서 <B>자세(pose)</B>, 즉 X 축, Y 축 위치(position)와 강체의 방향각(orientation)을 표현하는데 주로 사용된다.
 * Z 축 방향의 회전에 대한 부호는 아래 그림과 같은 오른손 좌표계를 따른다.
 *
 * \image html righthand_coordinate.png "오른손 좌표계"
 * \image latex righthand_coordinate.png "오른손 좌표계" width=0.5\textwidth
 */
class Pose : public Point
{
public:
    /**
     * 생성자 (기본값: 0)
     */
    Pose() { x = y = theta = 0; }

    /**
     * X축, Y축 값와 방향각을 초기화하는 생성자
     * @param x X 축 값
     * @param y Y 축 값
     * @param theta Z 축 회전에 대한 값 (오른손 좌표계)
     */
    Pose(double x, double y, double theta)
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    /**
     * 등가 비교
     * @param rhs 등가 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 같음 여부
     */
    bool operator==(const Pose& rhs) const { return (x == rhs.x) && (y == rhs.y) && (theta == rhs.theta); }

    /**
     * 상등 비교
     * @param rhs 상등 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 다름 여부
     */
    bool operator!=(const Pose& rhs) const { return (x != rhs.x) || (y != rhs.y) || (theta != rhs.theta); }

    /** Z 축에 대한 회전 값 */
    double theta;
	
};

/**
 * <B>Extended Kalman Filter (EKF)</B>
 *
 * uRON에서 사용하는 <B>확장 칼만 필터(이하 EKF)</B>의 기본 구현이다.
 *
 * EKF의 바탕이되는 칼만 필터(이하 KF)는 가우시안 노이즈를 갖는 선형 시스템의 상태를 최적으로 추정하는 필터이다.
 * EKF은 KF를 아래와 같은 비선형 시스템의 상태변수 \f$ x_k \f$를 추정할 수 있도록 확장한 것이다.
 * \f$ u_k \f$은 시스템에 주어진 제어입력, \f$ z_k \f$은 상태변수 \f$ x_k \f$의 관측값,
 * \f$ w_{k-1} \f$, \f$ v_k \f$은 각각 상태천이와 상태관측에서 발생하는 가우시안 노이즈이다.
 *
 * - 제어 입력값에 대한 <B>상태천이함수 (State Transition Function)</B>: \f$ x_k = f(x_{k-1}, u_k, w_{k-1}) \f$
 * - 상태 관측값에 대한 <B>상태관측함수 (State Measurement Function)</B>: \f$ \hat{z}_k = h(\hat{x}_k, v_k) \f$
 *
 * EKF는 아래 그림과 같이 추측 단계(prediction)과 수정 단계(correction)의 반복으로 동작한다.
 * 추즉 단계는 시스템에 주어진 제어입력을 이용하여 상태변수를 갱신하고,
 * 수정 단계는 상태변수의 관측값을 이용하여 상태변수를 갱신한다.
 *
 * \image html ekf.png "EKF의 동작 (from Welch and Bishop)"
 * \image latex ekf.png "EKF의 동작 (from Welch and Bishop)" width=0.7\textwidth
 *
 * 사용자는 EKF 클래스를 상속받아 EKF를 적용할 시스템의 상태천이함수(\f$ f \f$, EKF::TransitModel 함수)와 그 자코비안(\f$ A_k \f$, EKF::TransitJacobian 함수), 그 노이즈의 공분산(\f$ W_k Q_{k-1} W_k^T \f$, EKF::TransitNoiseCov 함수),
 * 그리고 상태관측함수(\f$ h \f$, EKF::MeasureModel 함수)과 그 자코비안(\f$ H_k \f$, EKF::MeasureJacobian 함수), 그 노이즈의 공분산(\f$ V_k R_k V_k^T \f$, EKF::MeasureNoiseCov 함수)을 구현하여
 * EKF의 EKF::Predict 함수와 EKF::Correct 함수를 호출하여 시스템의 상태를 추정한다.
 * uRON::EKFLocalizer 클래스의 구현을 참고하여 사용자의 시스템에 알맞은 클래스를 구현한다.
 *
 * @see Greg Welch and Gary Bishop, <I>An Introduction to the Kalman Filter</I>, UNC-Chapel Hill, TR95-041, 2006, http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html
 * @see The Kalman Filter, Welch, http://www.cs.unc.edu/~welch/kalman/
 */
class EKF
{
public:
    /**
     * 상태변수 \f$ x_k \f$와 관측변수 \f$ z_k \f$의 크기에 맞게 내부 행렬들을 초기화
     * @param dimX 상태변수의 크기
     * @param dimZ 관측변수의 크기
     */
    void Initialize(int dimX, int dimZ){
		state.resize(dimX, 1);
		state.setZero();
		stateCov.setIdentity(dimX, dimX);
		tempG.resize(dimX, dimX);
		tempH.resize(dimZ, dimX);
		kalmanGain.resize(dimX, dimZ);
		tempK.resize(dimZ, dimZ);
	}

    /**
     * 주어진 제어입력 \f$ u_k \f$을 이용하여 상태 추측 (내부적으로 상태변수 업데이트)
     * @param control 주어진 제어입력
     * @return 추정된 상태변수
     */
    const Matrix& Predict(const Matrix& control){
		// Predict the state
		state = TransitModel(state, control);
		tempG = TransitJacobian(state, control);
		stateCov = tempG * stateCov * tempG.transpose() + TransitNoiseCov(control);
		return state;
	}	

    /**
     * 주어진 관측값 \f$ z_k \f$을 이용하여 상태 수정 (내부적으로 상태변수 업데이트)
     *
     * 칼만 증폭 값(Kalman gain)을 계산하기 위한 역행렬이 유효하지 않은 경우 상태변수를 업데이트하지 않음
     * @param measurement 주어진 관측입력
     * @return 수정된 상태변수
     * @see Matrix::IsValid
     */
    const Matrix& Correct(const Matrix& measurement){
		// Calculate Kalman gain
		tempH = MeasureJacobian(state);
		tempK = tempH * stateCov * tempH.transpose() + MeasureNoiseCov();
		Eigen::FullPivLU<Matrix> lu(tempK);
		if (!lu.isInvertible()) return state;
		kalmanGain = stateCov * tempH.transpose() * lu.inverse();

		// Correct the state
		state = state + kalmanGain * (measurement - MeasureModel(state));
		tempH = MeasureJacobian(state);
		stateCov = stateCov - kalmanGain * tempH * stateCov;
		return state;
	}

    /**
     * 시스템의 상태변수 \f$ x_k \f$을 설정
     * @param state 상태변수
     */
    void SetState(const Matrix& state) { this->state = state; }

    /**
     * 현재 상태변수 \f$ x_k \f$을 반환
     * @return 상태변수
     */
    const Matrix& GetState(void) { return state; }

    /**
     * 시스템의 상태변수의 공분산 \f$ P_k \f$을 설정
     * @param covariance 상태변수의 공분산
     */
    void SetStateCov(const Matrix& covariance) { stateCov = covariance; }

    /**
     * 현재 상태변수의 공분산 \f$ P_k \f$을 반환
     * @return 상태변수의 공분산
     */
    const Matrix& GetStateCov(void) { return stateCov; }

protected:
    /**
     * 시스템의 상태천이함수 (state transition function), \f$ f(x_{k-1}, u_{k-1}, 0) \f$
     * @param state 시스템의 상태변수
     * @param control 시스템의 제어입력
     * @return 상태천이 결과 행렬
     */
    virtual Matrix& TransitModel(const Matrix& state, const Matrix& control) = 0;

    /**
     * 시스템의 상태천이함수의 자코비안 (Jacobian), \f$ A_k \f$
     * @param state 시스템의 상태변수
     * @param control 시스템의 제어입력
     * @return 상태천이 자코비안 행렬
     */
    virtual Matrix& TransitJacobian(const Matrix& state, const Matrix& control) = 0;

    /**
     * 현재 설정된 상태천이 노이즈의 공분산 \f$ W_k Q_{k-1} W_k^T \f$을 반환
     * @param control 시스템의 제어입력
     * @return 상태천이 노이즈의 공분산
     */
    virtual Matrix& TransitNoiseCov(const Matrix& control) = 0;

    /**
     * 시스템의 상태관측함수 (state measurement function), \f$ h(x_k, 0) \f$
     * @param state 시스템의 상태변수
     * @return 상태관측 결과 행렬
     */
    virtual Matrix& MeasureModel(const Matrix& state) = 0;

    /**
     * 시스템의 상태관측함수의 자코비안 (Jacobian), \f$ H_k \f$
     * @param state 시스템의 상태변수
     * @return 상태관측 자코비안 행렬
     */
    virtual Matrix& MeasureJacobian(const Matrix& state) = 0;

    /**
     * 현재 설정된 상태관측 노이즈의 공분산 \f$ V_k R_k V_k^T \f$을 반환
     * @return 상태관측 노이즈의 공분산
     */
    virtual Matrix& MeasureNoiseCov(void) = 0;

    /** 시스템의 상태변수 (state variable), \f$ x_k \f$ */
    Matrix state;

    /** 시스템의 상태변수의 공분산 (state covariance), \f$ P_k \f$ */
    Matrix stateCov;

    /** 칼만 증폭 값 (Kalman gain) \f$ K_k \f$ */
    Matrix kalmanGain;

private:
    /** 상태천이함수의 자코비안 계산 행렬 */
    Matrix tempG;

    /** 상태관측함수의 자코비안 계산 행렬 */
    Matrix tempH;

    /** 칼만 증폭 값 계산을 위한 임시 행렬 */
    Matrix tempK;
};

/**
 * <B>Random Number Generator with Uniform Distribution</B>
 *
 * 0과 1 사이의 연속 균일 확률 분포(uniform distribution)를 갖는 난수 발생기이다.
 */
class RandomUniform
{
public:
	/**
     * 난수 발생기(random number generator)를 초기화
     */
    RandomUniform(){ srand(static_cast<unsigned int>(time(NULL))); }
    /**
     * 균일 분포를 갖는 난수 발생
     */
    virtual double Generate(void){ return (static_cast<double>(rand()) / RAND_MAX); }
};

/**
 * <B>Random Number Generator with Normal Distribution</B>
 *
 * 평균 0, 표준편차 1의 정규 확률 분포(normal distribution)를 갖는 난수 발생기
 *
 * Box-Muller 알고리즘을 사용하여 구현하였다.
 *
 * @see Pseudo-random Numbers, Kaplan, http://www.bearcave.com/misl/misl_tech/wavelets/hurst/random.html
 */
class RandomNormal
{
public:
	/**
     * 난수 발생기(random number generator)를 초기화
     */
    RandomNormal(){ srand(static_cast<unsigned int>(time(NULL))); }
    /**
     * 정규 분포를 갖는 난수 발생
     */
    virtual double Generate(void){
		// Box-Muller (Polar) algorithm
		// Ref. http://www.bearcave.com/misl/misl_tech/wavelets/hurst/random.html
		// Ref. http://www.mathworks.com/moler/random.pdf
		double u1, u2, s;
		do
		{
			u1 = 2.0 * rand() / RAND_MAX - 1.0;
			u2 = 2.0 * rand() / RAND_MAX - 1.0;
			s = u1 * u1 + u2 * u2;
		}
		while (s > 1.0);
		double value = sqrt(-2.0 * log(s) / s) * u1;
		return value;
	}
};

template<typename T> class NodeT;
template<typename T> class EdgeT;
template<typename T> class Graph;

/**
 * <B>Edge of Sparse Graph</B>
 *
 * 그래프 자료구조의 노드(node)와 노드를 연결하는 <B>에지(edge)</B>이다.
 * 연결 비용을 나타내는 weight 변수를 가진다.
 *
 * @see Node 그래프의 노드
 * @see SparseGraph 그래프 자료 구조
 */
template<typename T>
class EdgeT
{
public:
    /**
     * 생성자
     */
    EdgeT() { to = NULL; }

    /**
     * 에지의 방향과 연결 비용을 설정할 수 있는 생성자
     * @param to 에지가 가리키는 노드의 포인터
     * @param weight 열결 비용
     */
    EdgeT(NodeT<T>* to, double weight) { this->to = to; this->weight = weight; }

    /** 에지의 연결 비용 */
    double weight;

    /** 에지가 가르키는 노드의 포인터 */
    NodeT<T>* to;
};



/**
 * <B>Node of Sparse Graph</B>
 *
 * 그래프 자료구조의 <B>노드</B>로, 데이터를 나타내는 data 변수를 가진다.
 * 그래프 상의 다른 노드들과 에지(edge)로 연결된다.
 *
 * @see Edge 그래프의 에지
 * @see SparseGraph 그래프 자료 구조
 */
template<typename T>
class NodeT
{
public:
    /**
     * 생성자
     */
    NodeT() { }

    /**
     * 노드의 데이터를 설정할 수 있는 생성자
     * @param data 노드의 데이터
     */
    NodeT(T data) { this->data = data; }

    /**
     * 노드 데이터의 등가 비교 연산
     * @param rhs 등가 연산의 오른쪽 항
     * @return 데이터의 같음 여부
     */
    bool operator==(const NodeT<T>& rhs) const { return (data == rhs.data); }

    /**
     * 노드 데이터의 상등 비교 연산
     * @param rhs 상등 연산의 오른쪽 항
     * @return 데이터의 상등 여부
     */
    bool operator!=(const NodeT<T>& rhs) const { return (data != rhs.data); }

    /** 노드의 데이터 */
    T data;

    friend class Graph<T>;

protected:
    /** 노드에서 시작된 에지들의 리스트 */
    std::list<EdgeT<T> > listEdge;
};



/**
 * <B>Sparse Graph</B>
 *
 * uRON에서 사용하는 <B>그래프</B> 자료구조의 구현이다.
 *
 * 노드와 에지의 밀도가 낮은 그래프(sparse graph)를 위한 구현으로,
 * 밀도가 높은 그래프(dense graph)는 일반적으로 행렬로 표현한다.
 * SparseGraph 클래스는 아래 그림과 같이 방향성이 있는 에지만을 가진다 (directed graph).
 * 그러나 그림의 7번 노드와 11번 노드 사이의 에지와 같이 두 개의 에지를 쌍으로 이용하여 방향성이 없는 그래프를 구현할 수 있다 (undireted graph).
 *
 * \image html directed_graph.png "Directed Graph (from Wikipedia)"
 * \image latex directed_graph.png "Directed Graph (from Wikipedia)" width=0.3\textwidth
 *
 * SparseGraph 클래스는 C++ 언어의 템플릿(template)을 이용하여 구현하였기 때문에, 객체 생성시 노드의 데이터 타입을 설정할 수 있다.
 *
 * @see <I>Directed Graph</I>, Wikipedia, http://en.wikipedia.org/wiki/Directed_graph
 */
template<class T>
class Graph
{
public:
    /**
     * 타입 T의 그래프 안에 있는 노드들의 순환 연산을 위한 반복자 타입
     */
    typedef typename std::list< NodeT<T> >::iterator NodeItr;

    /**
     * 타입 T의 노드가 가리키는 에지들의 순환 연산을 위한 반복자 타입
     */
    typedef typename std::list< EdgeT<T> >::iterator EdgeItr;

    /**
     * 타입 T의 그래프의 노드 타입
     */
    typedef class NodeT<T> Node;

    /**
     * 타입 T의 그래프의 에지 타입
     */
    typedef class EdgeT<T> Edge;

    /**
     * 소멸자
     *
     * 그래프에 할당된 모든 메모리를 해제
     */
    virtual ~Graph() { RemoveAll(); }

    /**
     * 노드 추가 (시간 복잡도: O(1))
     * @param data 추가할 노드의 데이터
     * @return 추가한 노드의 포인터
     */
    Node* AddNode(const T& data) { listNode.push_back(data); return &(listNode.back()); }

    /**
     * 에지 추가 (시간 복잡도: O(1))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @param weight 연결 비용 (기본값: 1.0)
     * @return 추가한 에지의 포인터
     */
    Edge* AddEdge(Node* from, Node* to, double weight = 1.0)
    {
        assert(from);
        assert(to);
        from->listEdge.push_back(Edge(to, weight));
        return &(from->listEdge.back());
    }

    /**
     * 주어진 데이터를 가진 노드를 반환 (시간 복잡도: O(N))
     * @param data 검색할 노드의 데이터
     * @return 검색된 노드의 포인터 (주어진 데이터를 갖는 노드가 없는 경우 NULL 반환)
     */
    Node* GetNode(const T& data)
    {
        Node* node = NULL;
        NodeItr itrN = listNode.begin();
        for (; itrN != listNode.end(); itrN++)
            if (itrN->data == data) { node = &(*itrN); break; }
        return node;
    }

    /**
     * 주어진 시작 노드의 데이터와 목적 노드의 데이터를 가리키는 에지를 반환 (시간 복잡도: O(N + E))
     * @param from 시작 노드의 데이터
     * @param to 목적 노드의 데이터
     * @return 검색된 에지의 포인터 (주어진 두 노드를 연결하는 에지가 없는 경우 NULL 반환)
     */
    Edge* GetEdge(const T& from, const T& to)
    {
        Node* fnode = GetNode(from);
        Node* tnode = GetNode(to);
        if (fnode == NULL || tnode == NULL) return NULL;
        return GetEdge(fnode, tnode);
    }

    /**
     * 주어진 시작 노드와 목적 노드를 가리키는 에지를 반환 (시간 복잡도: O(E))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @return 검색된 에지의 포인터 (주어진 두 노드를 연결하는 에지가 없는 경우 NULL 반환)
     */
    Edge* GetEdge(Node* from, Node* to)
    {
        assert(from);
        assert(to);
        Edge* edge = NULL;
        EdgeItr itrE = from->listEdge.begin();
        for (; itrE != from->listEdge.end(); itrE++)
            if (itrE->to->data == to->data) { edge = &(*itrE); break; }
        return edge;
    }

    /**
     * 주어진 시작 노드와 목적 노드를 가리키는 에지를 반환 (시간 복잡도: O(E))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @return 검색된 에지의 포인터 (주어진 두 노드를 연결하는 에지가 없는 경우 NULL 반환)
     */
    Edge* GetEdge(NodeItr from, NodeItr to)
    {
        Edge* edge = NULL;
        EdgeItr itrE = from->listEdge.begin();
        for (; itrE != from->listEdge.end(); itrE++)
            if (itrE->to->data == to->data) { edge = &(*itrE); break; }
        return edge;
    }

    /**
     * 주어진 시작 노드와 목적 노드를 가리키는 에지의 연결 비용을 반환 (시간 복잡도: O(E))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @return 에지의 연결 비용 (주어진 두 노드를 연결하는 에지가 없는 경우 -1 반환)
     */
    double GetEdgeWeight(Node* from, Node* to)
    {
        Edge* edge = GetEdge(from, to);
        if (edge == NULL) return -1.0;
        return edge->weight;
    }

    /**
     * 주어진 시작 노드와 목적 노드를 가리키는 에지의 연결 비용을 반환 (시간 복잡도: O(E))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @return 에지의 연결 비용 (주어진 두 노드를 연결하는 에지가 없는 경우 -1 반환)
     */
    double GetEdgeWeight(NodeItr from, NodeItr to)
    {
        Edge* edge = GetEdge(from, to);
        if (edge == NULL) return -1.0;
        return edge->weight;
    }

    /**
     * 주어진 시작 노드와 목적 노드 사이의 연결 여부 반환 (시간 복잡도: O(E))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @return 두 노드의 연결 여부
     */
    bool IsConnected(Node* from, Node* to) { return (GetEdgeWeight(from, to) >= 0); }

    /**
     * 주어진 시작 노드와 목적 노드를 가리키는 에지의 연결 비용을 반환 (시간 복잡도: O(E))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @return 에지의 연결 비용 (주어진 두 노드를 연결하는 에지가 없는 경우 -1 반환)
     */
    bool IsConnected(NodeItr from, NodeItr to) { return (GetEdgeWeight(from, to) >= 0); }

    /**
     * 노드 삭제 (시간 복잡도: O(VE))
     *
     * 노드에 연결된 모든 에지를 함께 삭제
     * @param node 삭제할 노드의 포인터
     * @return 노드의 삭제 성공 여부
     */
    bool RemoveNode(Node* node)
    {
        assert(node);
        NodeItr itrFound = listNode.end();
        NodeItr itrN = listNode.begin();
        for (; itrN != listNode.end(); itrN++)
        {
            RemoveEdge(&(*itrN), node);
            if (itrN->data == node->data) itrFound = itrN;
        }
        bool isFound = (itrFound != listNode.end());
        if (isFound) listNode.erase(itrFound);
        return isFound;
    }

    /**
     * 노드 삭제 (시간 복잡도: O(VE))
     *
     * 노드에 연결된 모든 에지를 함께 삭제
     * @param node 삭제할 노드의 포인터
     * @return 노드의 삭제 성공 여부
     */
    bool RemoveNode(NodeItr node)
    {
        NodeItr itrFound = listNode.end();
        NodeItr itrN = listNode.begin();
        for (; itrN != listNode.end(); itrN++)
        {
            RemoveEdge(itrN, node);
            if (itrN->data == node->data) itrFound = itrN;
        }
        bool isFound = (itrFound != listNode.end());
        if (isFound) listNode.erase(itrFound);
        return isFound;
    }

    /**
     * 에지 삭제 (시간 복잡도: O(E))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @return 에지의 삭제 성공 여부
     */
    bool RemoveEdge(Node* from, Node* to)
    {
        assert(from);
        assert(to);
        bool isFound = false;
        EdgeItr itrE = from->listEdge.begin();
        while (itrE != from->listEdge.end())
        {
            if (itrE->to->data == to->data)
            {
                itrE = from->listEdge.erase(itrE);
                isFound = true;
                continue;
            }
            itrE++;
        }
        return isFound;
    }

    /**
     * 에지 삭제 (시간 복잡도: O(E))
     * @param from 시작 노드의 포인터
     * @param to 목적 노드의 포인터
     * @return 에지의 삭제 성공 여부
     */
    bool RemoveEdge(NodeItr from, NodeItr to)
    {
        bool isFound = false;
        EdgeItr itrE = from->listEdge.begin();
        while (itrE != from->listEdge.end())
        {
            if (itrE->to->data == to->data)
            {
                itrE = from->listEdge.erase(itrE);
                isFound = true;
                continue;
            }
            itrE++;
        }
        return isFound;
    }

    /**
     * 모든 노드와 에지를 삭제
     * @return 노드와 에지의 삭제 성공 여부
     */
    bool RemoveAll(void) { listNode.clear(); return true; }

    /**
     * 그래프 상에 있는 노드의 개수를 반환 (시간 복잡도: O(1))
     * @return 노드의 개수
     */
    int CountNodes(void) { return listNode.size(); }

    /**
     * 주어진 노드에서 시작하는 에지의 개수를 반환 (시간 복잡도: O(1))
     * @param node 시작 노드의 포인터
     * @return 에지의 개수
     */
    int CountEdges(Node* node)
    {
        assert(node);
        return node->listEdge.size();
    }

    /**
     * 주어진 노드에서 시작하는 에지의 개수를 반환 (시간 복잡도: O(1))
     * @param node 시작 노드의 포인터
     * @return 에지의 개수
     */
    int CountEdges(NodeItr node) { return node->listEdge.size(); }

    /**
     * 그래프의 노드 순환 연산을 위한 첫 번째 노드의 반복자 반환 (시간 복잡도: O(1))
     * @return 첫 번째 노드의 반복자
     * @see GetTailNode
     */
    NodeItr GetHeadNode(void) { return listNode.begin(); }

    /**
     * 그래프의 노드 순환 연산을 위한 마지막 노드의 반복자 반환 (시간 복잡도: O(1))
     * @return 마지막 노드의 반복자
     * @see GetHeadNode
     */
    NodeItr GetTailNode(void) { return listNode.end(); }

    /**
     * 주어진 노드에서 시작하는 첫 번째 에지의 반복자 반환 (시간 복잡도: O(1))
     * @param node 에지가 시작되는 노드의 포인터
     * @return 첫 번째 에지의 반복자
     * @see GetTailEdge
     */
    EdgeItr GetHeadEdge(Node* node)
    {
        assert(node);
        return node->listEdge.begin();
    }

    /**
     * 주어진 노드에서 시작하는 첫 번째 에지의 반복자 반환 (시간 복잡도: O(1))
     * @param node 에지가 시작되는 노드의 포인터
     * @return 첫 번째 에지의 반복자
     * @see GetTailEdge
     */
    EdgeItr GetHeadEdge(NodeItr node) { return node->listEdge.begin(); }

    /**
     * 주어진 노드에서 시작하는 마지막 에지의 반복자 반환 (시간 복잡도: O(1))
     * @param node 에지가 시작되는 노드의 포인터
     * @return 마지막 에지의 반복자
     * @see GetHeadEdge
     */
    EdgeItr GetTailEdge(Node* node)
    {
        assert(node);
        return node->listEdge.end();
    }

    /**
     * 주어진 노드에서 시작하는 마지막 에지의 반복자 반환 (시간 복잡도: O(1))
     * @param node 에지가 시작되는 노드의 포인터
     * @return 마지막 에지의 반복자
     * @see GetHeadEdge
     */
    EdgeItr GetTailEdge(NodeItr node) { return node->listEdge.end(); }

private:
    /** 그래프 내의 노드의 리스트 */
    std::list<Node> listNode;
}; // End of 'SparseGraph'


/**
 * <B>State of Differential Drive</B>
 *
 * 차륜 구동기(differential drive)의 두 바퀴의 값을 표현한다.
 * 두 바퀴의 변위나 속도, 가속도를 표현하는데 주로 사용된다.
 *
 * @see DiffDriveKinematics 차륜 구동기의 기구학
 */
class DiffDriveWheel
{
public:
    /**
     * 생성자 (기본값: 0)
     */
    DiffDriveWheel() { left = right = 0; }

    /**
     * 두 바퀴의 값을 초기화하는 생성자
     * @param left 구동기의 왼쪽 바퀴의 값
     * @param right 구동기의 오른쪽 바퀴의 값
     */
    DiffDriveWheel(double left, double right)
    {
        this->left = left;
        this->right = right;
    }

    /**
     * 등가 비교
     * @param rhs 등치 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 같음 여부
     */
    bool operator==(const DiffDriveWheel& rhs) const { return (left == rhs.left) && (right == rhs.right); }

    /**
     * 상등 비교
     * @param rhs 상등 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 다름 여부
     */
    bool operator!=(const DiffDriveWheel& rhs) const { return (left != rhs.left) || (right != rhs.right); }

    /** 차륜 구동기의 왼쪽 바퀴의 값 */
    double left;

    /** 차륜 구동기의 오른쪽 바퀴의 값 */
    double right;
};

/**
 * <B>Kinematics of Differential Drive</B>
 *
 * 차륜 구동기(differential drive) 정기구학(forward kinematics)과 역기구학(inverse kinematics)을 정의한다.
 * 기구학에 사용되는 파라미터는 아래 그림과 같다.
 *
 * \image html differential_drive.png "차륜 구동기의 기구학"
 * \image latex differential_drive.png "차륜 구동기의 기구학" width=0.5\textwidth
 *
 * @see FourMecanumKinematics 네 개의 메카늄 바퀴(Mecanum wheel)을 가진 구동기의 기구학
 */
class DiffDriveKinematics
{
public:
    /**
     * 차륜 구동기의 정기구학 (두 바퀴의 속도가 주어진 경우, 로봇의 선속도와 각속도로 반환)
     * @param wheel 두 바퀴의 속도 (단위: [m/s])
     * @param wheelBase 두 바퀴 사이의 길이 (단위: [m])
     * @param robot 로봇의 속도 <B>[반환값]</B> (선속도 단위: [m/s], 각속도 단위: [rad/s])
     */
    static void Forward(const DiffDriveWheel& wheel, double wheelBase, Polar& robot)
    {
        robot.linear = (wheel.right + wheel.left) / 2.0;
        robot.angular = (wheel.right - wheel.left) / wheelBase;
    }

    /**
     * 차륜 구동기의 역기구학 (로봇의 선속도와 각속도가 주어진 경우, 두 바퀴의 속도로 반환)
     * @param robot 로봇의 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
     * @param wheelBase 두 바퀴 사이의 길이 (단위: [m])
     * @param wheel 두 바퀴의 속도 <B>[반환값]</B> (단위: [m/s])
     */
    static void Inverse(const Polar& robot, double wheelBase, DiffDriveWheel& wheel)
    {
        wheel.left  = robot.linear - robot.angular * wheelBase / 2.0;
        wheel.right = robot.linear + robot.angular * wheelBase / 2.0;
    }
};

/**
 * <B>State of Four-Mecanum-Wheel Drive</B>
 *
 * 네 개의 메카늄 바퀴(Mecanum wheel)을 가진 구동기의 네 바퀴의 값을 표현한다.
 * 네 바퀴의 변위나 속도, 가속도를 표현하는데 주로 사용된다.
 *
 * @see FourMecanumKinematics 네 개의 메카늄 바퀴를 가진 구동기의 기구학
 */
class FourMecanumWheel
{
public:
    /**
     * 생성자 (기본값: 0)
     */
    FourMecanumWheel() { frontLeft = frontRight = rearLeft = rearRight = 0; }

    /**
     * 네 바퀴의 값을 초기화하는 생성자
     * @param frontLeft 구동기의 전방 왼쪽 바퀴의 값
     * @param frontRight 구동기의 전방 오른쪽 바퀴의 값
     * @param rearLeft 구동기의 후방 왼쪽 바퀴의 값
     * @param rearRight 구동기의 후방 오른쪽 바퀴의 값
     */
    FourMecanumWheel(double frontLeft, double frontRight, double rearLeft, double rearRight)
    {
        this->frontLeft = frontLeft;
        this->frontRight = frontRight;
        this->rearLeft = rearLeft;
        this->rearRight = rearRight;
    }

    /**
     * 등가 비교
     * @param rhs 등치 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 같음 여부
     */
    bool operator==(const FourMecanumWheel& rhs) const
    {
        return (frontLeft == rhs.frontLeft) && (frontRight == rhs.frontRight) &&
               (rearLeft == rhs.rearLeft) && (rearRight == rhs.rearRight);
    }

    /**
     * 상등 비교기
     * @param rhs 상등 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 다름 여부
     */
    bool operator!=(const FourMecanumWheel& rhs) const
    {
        return (frontLeft != rhs.frontLeft) || (frontRight != rhs.frontRight) ||
               (rearLeft != rhs.rearLeft) || (rearRight != rhs.rearRight);
    }

    /** 구동기의 전방 왼쪽 바퀴의 값 */
    double frontLeft;

    /** 구동기의 전방 오른쪽 바퀴의 값 */
    double frontRight;

    /** 구동기의 후방 왼쪽 바퀴의 값 */
    double rearLeft;

    /** 구동기의 후방 오른쪽 바퀴의 값 */
    double rearRight;
};

/**
 * <B>2D Pose of Four-Mecanum-Wheel Drive</B>
 *
 * 네 개의 메카늄 바퀴(Mecanum wheel)을 가진 구동기의 자세를 표현한다.
 *
 * @see FourMecanumKinematics 네 개의 메카늄 바퀴를 가진 구동기의 기구학
 */
class FourMecanumPose : public Pose
{
public:
    /**
     * 생성자 (기본값: 0)
     */
    FourMecanumPose() { error = 0; }

    /**
     * 네 개의 메카늄 바퀴를 가진 구동기의 자세 값을 초기화하는 생성자
     * @param x X 축 값
     * @param y Y 축 값
     * @param theta Z 축 회전에 대한 값 (오른손 좌표계)
     * @param error 에러 보상 값
     */
    FourMecanumPose(double x, double y, double theta, double error)
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
        this->error = error;
    }

    /**
     * 등가 비교
     * @param rhs 등치 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 같음 여부
     */
    bool operator==(const FourMecanumPose rhs) const
    {
        return (x == rhs.x) && (y == rhs.y) && (theta == rhs.theta) && (error == rhs.error);
    }

    /**
     * 상등 비교기
     * @param rhs 상등 연산자의 오른쪽 항
     * @return 두 피연산자(operand)의 다름 여부
     */
    bool operator!=(const FourMecanumPose rhs) const
    {
        return (x != rhs.x) || (y != rhs.y) || (theta != rhs.theta) || (error != rhs.error);
    }

    /** 에러 보상 값 */
    double error;
};

/**
 * <B>Kinematics of Four-Mecanum-Wheel Drive</B>
 *
 * 네 개의 메카늄 바퀴(Mecanum wheel)을 가진 구동기의 정기구학(forward kinematics)과 역기구학(inverse kinematics)을 정의한다.
 * 기구학에 사용되는 파라미터는 아래 그림과 같다.
 *
 * \image html four_mecanum_drive.png "네 개의 메카늄 바퀴 구동기의 기구학"
 * \image latex four_mecanum_drive.png "네 개의 메카늄 바퀴 구동기의 기구학" width=0.8\textwidth
 *
 * @see DiffDriveKinematics 차륜 구동기(differential drive)의 기구
 */
class FourMecanumKinematics
{
public:
    /**
     * 네 개의 메카늄 바퀴 구동기의 정기구학 (네 바퀴의 속도가 주어진 경우, 로봇의 X, Y 방향 속도 및 Z 방향 회전 속도로 반환)
     * @param wheel 네 바퀴의 속도 (단위: [m/s])
     * @param wheelBase 두 바퀴 사이의 길이 (단위: [m])
     * @param robot 로봇의 속도와 에러 보상값 <B>[반환값]</B> (선속도 단위: [m/s], 각속도 단위: [rad/s])
     */
    static void Forward(const FourMecanumWheel& wheel, const double wheelBase, FourMecanumPose& robot)
    {
        robot.x = (wheel.frontLeft + wheel.frontRight + wheel.rearLeft + wheel.rearRight) / 4.0;
        robot.y = (-wheel.frontLeft + wheel.frontRight + wheel.rearLeft - wheel.rearRight) / 4.0;
        robot.theta = (-wheel.frontLeft + wheel.frontRight - wheel.rearLeft + wheel.rearRight) / 4.0 / (wheelBase / 2.0);
        robot.error = (wheel.frontLeft + wheel.frontRight - wheel.rearLeft - wheel.rearRight) / 4.0;
    }

    /**
     * 네 개의 메카늄 바퀴 구동기의 역기구학 (로봇의 X, Y 방향 속도 및 Z 방향 회전 속도가 주어진 경우, 네 바퀴의 속도로 반환)
     * @param robot 로봇의 속도와 에러 보상값 (선속도 단위: [m/s], 각속도 단위: [rad/s])
     * @param wheelBase 두 바퀴 사이의 길이 (단위: [m])
     * @param wheel 네 바퀴의 속도 <B>[반환값]</B> (단위: [m/s])
     */
    static void Inverse(const FourMecanumPose& robot, const double wheelBase, FourMecanumWheel& wheel)
    {
        double angular = robot.theta * (wheelBase / 2.0);
        wheel.frontLeft  = robot.x - robot.y - angular + robot.error;
        wheel.frontRight = robot.x + robot.y + angular + robot.error;
        wheel.rearLeft   = robot.x + robot.y - angular - robot.error;
        wheel.rearRight  = robot.x - robot.y + angular - robot.error;
    }
};

/**
 * <B>2D Pose Container Interface</B>
 *
 * 자세(pose) 데이터를 입출력으로 하는 클래스의 기본적으로 필요한 부분을 구현한 것으로
 * UnicycleRobotFSM 등의 작업 제어기의 부모 클래스로 사용된다.
 */
class PoseCase
{
public:
    /**
     * 생성자 (기본값: (0, 0, 0))
     */
    PoseCase(){
		poseData = Pose(0, 0, 0);
	}

    /**
     * 자세 데이터를 입력
     * @param pose 자세 데이터 (거리 단위: [m], 각도 단위: [rad])
     */
    void SetPose(const Pose& pose){
		poseData = pose;
		poseData.theta = TrimRadianAngle(poseData.theta);
	}

    /**
     * 현재 자세를 반환
     * @return 자세 (거리 단위: [m], 각도 단위: [rad])
     */
    Pose GetPose(void){
		Pose pose = poseData;
		return pose;
	}

    /**
     * 현재의 자세에 주어진 병진과 회전 변위를 더함
     * @param delta 병진 및 회전 변위 (거리 단위: [m], 각도 단위: [rad])
     */
    Polar AddPose(const Polar& delta){
		double theta = poseData.theta + delta.angular / 2;
		poseData.x += delta.linear * cos(theta);
		poseData.y += delta.linear * sin(theta);
		poseData.theta = TrimRadianAngle(poseData.theta + delta.angular);
		return delta;
	}

    /**
     * 현재의 자세에 주어진 두 자세의 변위를 더함
     * @param preOdo 이동 전의 자세 (거리 단위: [m], 각도 단위: [rad])
     * @param curOdo 이동 후의 자세 (거리 단위: [m], 각도 단위: [rad])
     */
    Polar AddPose(const Pose& preOdo, const Pose& curOdo){
		Pose deltaPose;
		deltaPose.x = curOdo.x - preOdo.x;
		deltaPose.y = curOdo.y - preOdo.y;
		deltaPose.theta = TrimRadianAngle(curOdo.theta - preOdo.theta);

		double rho = sqrt(deltaPose.x * deltaPose.x + deltaPose.y * deltaPose.y);
		double phi = TrimRadianAngle(atan2(deltaPose.y, deltaPose.x) - preOdo.theta);
		// Consider the backward motion
		if (phi >= uRON_PI / 2)
		{
			rho *= -1;
			phi -= uRON_PI;
		}
		else if (phi < -uRON_PI / 2)
		{
			rho *= -1;
			phi += uRON_PI;
		}

		double theta = poseData.theta + phi;
		poseData.x += rho * cos(theta);
		poseData.y += rho * sin(theta);
		poseData.theta = TrimRadianAngle(poseData.theta + deltaPose.theta);
		return Polar(rho, deltaPose.theta);
	}

    /**
     * 주어진 자세 데이터로 초기화
     * @param pose 자세 데이터 (거리 단위: [m], 각도 단위: [rad])
     */
    void ResetPose(const Pose& pose){
		SetPose(pose);
	}

protected:
    /** 로봇의 현재 자세 (거리 단위: [m], 각도 단위: [rad]) */
    Pose poseData;
};

/**
 * <B>Range Data Specification</B>
 *
 * 거리 감지 데이터(range data)의 명세를 기술한다.
 *
 * uRON은 빔(beam) 단위로 거리 감지 데이터를 다룬다.
 * 하나의 빔 데이터는 센서의 중심에서 주어진 빔의 방향각으로 뻗었을 때 검출된 장애물과의 거리이다.
 * 센서의 설치 위치와 빔의 특성을 기술하는 각 파라미터는 아래 그림과 같다.
 *
 * \image html range_data_spec.png "거리 감지 데이터의 명세"
 * \image latex range_data_spec.png "거리 감지 데이터의 명세" width=0.5\textwidth
 *
 * @see RangeDataCase 거리 감지 데이터의 입출력 인터페이스
 */
class RangeDataSpec
{
public:
    /**
     * 생성자 (기본값: 0)
     */
    RangeDataSpec()
    {
        beamNumber = 0;
        beamDirection = NULL;
        beamMaxRange = 0;
        beamMinRange = 0;
        beamWidth = 0;
        offset = Pose(0, 0, 0);
    }

    /** 빔의 개수 */
    int beamNumber;

    /** 빔의 방향 (단위: [rad]) */
    double* beamDirection;

    /** 빔의 최대 감지 거리 (단위: [m]) */
    double beamMaxRange;

    /** 빔의 최소 감지 거리 (단위: [m]) */
    double beamMinRange;

    /** 빔의 폭 (단위: [rad]) */
    double beamWidth;

    /** 로봇 좌표계에서의 센서의 위치와 방향 (거리 단위: [m], 각도 단위: [rad]) */
    Pose offset;
};

/**
 * <B>Range Data Container Basic Interface</B>
 *
 * 거리 감지 데이터(range data)를 입출력으로 하는 클래스의 기본적인 부분을 구현한 것으로
 * OccupancyGridMapper와 StopAndGo 등의 지도 작성기와 장애물 회피기의 부모 클래스로 사용된다.
 */
class RangeDataCase
{
public:
    /**
     * 생성자
     */
    RangeDataCase(){
		rangeData = NULL;
		rangeSpec.beamDirection = NULL;
	}

    /**
     * 소멸자
     */
    ~RangeDataCase(){
		ReleaseRangeBuffer();
	}

    /**
     * 거리 감지 데이터의 명세를 설정
     * @param spec 거리 감지 데이터의 명세
     * @see RangeDataSpec
     */
    void SetRangeDataSpec(const RangeDataSpec& spec){
		// Deallocate 'rangeData' and 'rangeSpec.beamDirection'
		ReleaseRangeBuffer();
		// Copy specification of range finder,
		// and allocate 'rangeData' and 'rangeSpec.beamDirection'
		rangeSpec = spec;
		if (spec.beamNumber != 0 && spec.beamDirection != NULL){
			rangeData = new double[rangeSpec.beamNumber];
			for (int i = 0; i < rangeSpec.beamNumber; i++)
				rangeData[i] = -1.0;
			rangeSpec.beamDirection = new double[rangeSpec.beamNumber];
			memcpy(rangeSpec.beamDirection, spec.beamDirection, rangeSpec.beamNumber * sizeof(double));
		}
	}

    /**
     * 현재 설정된 거리 감지 데이터의 명세를 반환
     * @return 거리 감지 데이터의 명세
     * @see RangeDataSpec
     */
    const RangeDataSpec* GetRangeDataSpec(void){
		return &rangeSpec;
	}

    /**
     * 거리 감지 데이터를 입력
     *
     * 빔에 감지되는 물체가 없는 경우, 감지 데이터의 값은 -1으로 입력
     * @param measurement 거리 감지 데이터의 포인터 (단위: [m])
     */
    void SetRangeData(double* measurement){ 
		if (rangeData != NULL && measurement != NULL)
			memcpy(rangeData, measurement, rangeSpec.beamNumber * sizeof(double));
	}

    /**
     * 현재 입력된 거리 감지 데이터를 반환
     *
     * 데이터를 반환받을 버퍼는 거리 감지 데이터의 크기만큼 미리 메모리 할당을 하여야 함
     *
     * 메모리 할당 예: double* buffer = new double[빔의 개수];
     * @param buffer 거리 감지 데이터를 반환받을 버퍼의 포인터 <B>[반환값]</B> (단위: [m])
     * @return 반환된 거리 감지 데이터의 빔의 개수
     */
    int GetRangeData(double* buffer){
		int num = 0;
		if (rangeData != NULL && buffer != NULL){
			memcpy(buffer, rangeData, rangeSpec.beamNumber * sizeof(double));
			num = rangeSpec.beamNumber;
		}
		return num;
	}

    /**
     * 거리 감지 데이터 명세가 설정되어 거리 감지 데이터 입력 가능 유무를 반환
     * @return 거리 감지 데이터의 입력 가능 유무
     */
    bool IsConfigured(void){
		return (rangeData != NULL) && (rangeSpec.beamDirection != NULL);
	}

protected:
    /**
     * 거리 감지 데이터와 관련된 메모리 할당을 해제
     */
    void ReleaseRangeBuffer(void){
		if (rangeSpec.beamDirection != NULL){
			delete [] rangeSpec.beamDirection;
			rangeSpec.beamDirection = NULL;
		}
		if (rangeData != NULL){
			delete [] rangeData;
			rangeData = NULL;
		}
	}

    /** 거리 감지 데이터의 포인터 */
    double* rangeData;

    /** 거리 감지 데이터의 명세 */
    RangeDataSpec rangeSpec;

};
/**
 * <B>Obstacle Avoider Basic Implementation</B>
 *
 * 장애물 회피기에 기본적으로 필요한 부분을 구현한 것으로
 * StopAndGo, VFHPlus 등의 장애물 회피기의 부모 클래스로 사용된다.
 */
class ObstacleAvoider : public RangeDataCase
{
public:
    /**
     * 생성자
     */
    ObstacleAvoider(){
		paramMaxVelocity = Polar(0.3, uRON_DEG2RAD(45));
		paramMaxAcceleration = Polar(0.3, uRON_DEG2RAD(45));
		paramGoal = Pose(0, 0, 0);
	}
	/**
     * 장애물 회피를 수행
     *
     * 거리 감지 데이터를 이용하여 로봇이 행동을 결정 (예: 추종 위치 또는 추종 속도 등을 변경)
     * @param currentPose 로봇의 현재 위치와 방향 (거리 단위: [m], 각도 단위: [rad])
     * @param currentVelocity 로봇의 현재 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
     * @param elapse 직전에 AvoidObstacle 함수가 호출되고 현재 함수가 호출되기 전까지 경과된 시간 (단위: [sec])
     * @param targetPose 로봇이 현재 추종하려고 했던 위치와 방향을 인자로 넣고, 변경된 추종 위치와 방향이 반환 <B>[반환값]</B> (거리 단위: [m], 각도 단위: [rad])
     * @param targetVelocity 로봇이 현재 추종하려고 했던 속도를 인자로 넣고, 변경된 추종 속도가 반환 <B>[반환값]</B> (선속도 단위: [m/s], 각속도 단위: [rad/s])
     * @return 장애물 회피 결과 (0: 주변에 장애물이 없거나 회피 가능한 경우 / -1: 장애물 회피기가 준비되지 않은 경우 / -3: 장애물로 인한 이동할 수 없는 경우)
     */
	virtual int AvoidObstacle(const Pose& currentPose, const Polar& currentVelocity, double elapse, Pose& targetPose, Polar& targetVelocity){
        (void)currentPose;
        (void)currentVelocity;
        (void)elapse;
        (void)targetPose;
        (void)targetVelocity;
        return 0;
    }
    /**
     * 장애물 회피기의 준비 여부를 반환
     * @return 장애물 회피기의 준비 여부
     */
    bool IsReady(void){	return IsConfigured(); }

    /**
     * 장애물 회피에 사용되는 최고 운행 속도를 설정
     * @param velocity 최고 운행 속도 (선속도 기본값: 0.3, 각속도 기본값: MATH_DEG2RAD(45)) (선속도 단위: [m/s], 각속도 단위: [rad/s])
     */
    void SetMaxVelocity(const Polar& velocity){	paramMaxVelocity = velocity; }

    /**
     * 장애물 회피에 사용되는 최고 운행 속도를 반환
     * @return 최고 운행 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
     */
    Polar GetMaxVelocity(void){
		Polar velocity = paramMaxVelocity;
		return velocity;
	}

    /**
     * 장애물 회피에 사용되는 최고 운행 가속도를 설정
     * @param acceleration 최고 운행 가속도 (선속도 기본값: 0.3, 각속도 기본값: MATH_DEG2RAD(45)) (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2])
     */
    void SetMaxAcceleration(const Polar& acceleration){	paramMaxAcceleration = acceleration; }

    /**
     * 장애물 회피에 사용되는 최고 운행 가속도를 반환
     * @return 최고 운행 가속도 (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2])
     */
    Polar GetMaxAcceleration(void){
		Polar acceleration = paramMaxAcceleration;
		return acceleration;
	}

    /**
     * 목적지의 위치와 방향각을 설정
     * @param 위치와 방향각 (기본값: (0, 0, 0)) (거리 단위: [m], 각도 단위: [rad])
     */
    void SetGoalPose(const Pose& goal){	
		paramGoal = goal; 
	}

    /**
     * 목적지의 위치와 방향각을 반환
     * @param 위치와 방향각 (거리 단위: [m], 각도 단위: [rad])
     */
    Pose GetGoalPose(void){
		Pose goal = paramGoal;
		return goal;
	}

protected:
    /** 최고 운행 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s]) */
    Polar paramMaxVelocity;

    /** 최고 운행 가속도 (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2]) */
    Polar paramMaxAcceleration;

    /** 목적지의 위치와 방향각 (거리 단위: [m], 각도 단위: [rad]) */
    Pose paramGoal;

};


/**
 * <B>Path Planner Interface</B>
 *
 * <B>경로 계획기</B>에 필요한 기본 함수를 정의한 인터페이스이다.
 *
 * 경로 계획기는 로봇의 시작 지점(예: 로봇의 위치와 방향)에서 목적 지점(예: 로봇이 이동하고자 하는 위치와 방향)까지 충돌 없이 이동 가능한 경로를 생성한다.
 * 생성한 경로는 공간 상의 연속된 점들, 즉 Point 클래스의 배열로 표현된다.
 * 경로 계획기에 의해 생성된 경로는 사용 후 PathPlanner::ReleasePath 함수를 통해 메모리 할당을 해제하여야 한다.
 */
class PathPlanner
{
public:
    /**
     * 주어진 시작 위치에서 목적 위치까지의 경로 생성
     * @param start 시작 위치 (거리 단위: [m])
     * @param goal 목적 위치 (단위: [m])
     * @param length 경로의 길이 <B>[반환값]</B>
     * @return 경로를 담고 있는 포인터 (NULL: 경로가 없는 경우)
     */
    virtual Point* GeneratePath(const Point& start, const Point& goal, int& length){
        (void)start;
        (void)goal;
        (void)length;
		return NULL;
	}

    /**
     * PathPlanner::GeneratePath 함수를 통해 반환된 경로의 메모리를 해제
     * @param path 경로를 담고 있는 포인터
     * @return 메모리 해제의 성공 여부
     */
    virtual bool ReleasePath(Point* path){
        (void)path;
		return false;
	}
};

/**
 * <B>Path Follower Basic Implementation</B>
 *
 * 경로 추종기에 기본적으로 필요한 부분을 구현한 것으로
 * PurePursuit, LandingPathTracker 등의 경로 추종기들의 부모 클래스로 사용된다.
 */
class PathFollower
{
public:
    /**
     * 생성자
     */
    PathFollower(){
		pathData = NULL;
		pathSize = 0;

		paramMaxVelocity = Polar(0.3, uRON_DEG2RAD(45));
		paramMaxAcceleration = Polar(0.3, uRON_DEG2RAD(45));
		paramAvoider = NULL;
	}

	/**
     * 경로 추종을 수행
     *
     * 주어진 경로과 로봇의 현재 위치와 속도를 고려하여 로봇의 동작 속도를 결정
     * @param currentPose 로봇의 현재 위치와 방향 (거리 단위: [m], 각도 단위: [rad])
     * @param currentVelocity 로봇의 현재 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
     * @param elapse 직전에 FollowPath 함수가 호출되고 현재 함수가 호출되기 전까지 경과된 시간 (단위: [sec])
     * @param controlVelocity 경로를 추종하기 위한 로봇의 동작 속도 <B>[반환값]</B> (선속도 단위: [m/s], 각속도 단위: [rad/s])
     * @return 0을 포함한 양수인 경우 현재 로봇의 위치와 가장 가까운 경로의 인덱스 (-1: 경로 추종기가 준비되지 않은 경우 / -2: 목적 지점에 도착한 경우 / -3: 장애물로 인해 이동할 수 없는 경우)
     */
    virtual int FollowPath(const Pose& currentPose, const Polar& currentVelocity, double elapse, Polar& controlVelocity){
        (void)currentPose;
        (void)currentVelocity;
        (void)elapse;
        (void)controlVelocity;
        return -1;
	}

    /**
     * 소멸자
     *
     * 경로와 관련된 메모리 할당을 해제
     * @see ResetPath
     */
    ~PathFollower(){ ResetPath(); }

    /**
     * 추종할 경로를 입력
     * @param path 추종할 경로의 포인터 (단위: [m])
     * @param length 경로의 길이
     * @return 경로 추종기에 입력된 경로의 길이
     */
    virtual int LoadPath(const Point* path, int length){
		ResetPath();
		pathData = new Point[length];
		if (pathData == NULL)
		{
			return 0;
		}
		memcpy(pathData, path, length * sizeof(Point));
		pathSize = length;
		if (paramAvoider != NULL)
			paramAvoider->SetGoalPose(Pose(pathData[pathSize - 1].x, pathData[pathSize - 1].y, 0)); // TODO
		return length;
	}

    /**
     * 입력된 추종 경로를 반환
     *
     * 경로를 반환받을 버퍼는 경로의 길이만큼 미리 메모리 할당을 하여야 함
     *
     * 메모리 할당 예: uRON::Point* buffer = new uRON::Point[경로의 길이];
     *
     * @param buffer 경로를 반환받을 버퍼의 포인터 <B>[반환값]</B> (단위: [m])
     * @param bufferSize 경로를 반환받을 버퍼의 최대 크기
     * @return 실제 반환된 경로의 길이
     */
    int GetPath(Point* buffer, int bufferSize){
		int length = 0;
		if (pathData != NULL)
		{
			length = uRON_MIN(pathSize, bufferSize);
			memcpy(buffer, pathData, length * sizeof(Point));
		}
		return length;
	}

    /**
     * 입력된 추종 경로의 길이를 반환
     * @return 추종 경로의 길이
     */
    int GetPathSize(void){
		int length = 0;
		if (pathData != NULL) length = pathSize;
		return length;
	}

    /**
     * 경로와 관련된 메모리를 해제
     */
    void ResetPath(void){
		if (pathData != NULL)
		{
			delete [] pathData;
			pathData = NULL;
		}
		pathSize = 0;
	}

    /**
     * 경로 추종기의 준비 여부를 반환
     * @return 경로 추종기의 준비 여부
     */
    bool IsReady(void){ return (pathData != NULL) && (pathSize > 0); }

    /**
     * 장애물 회피기를 설정 (장애물 회피를 하지 않을 경우 NULL) (기본값: NULL)
     * @param avoider 장애물 회피기의 포인터
     */
    void SetObstacleAvoider(ObstacleAvoider* avoider){ paramAvoider = avoider; }

    /**
     * 현재의 설정된 장애물 회피기를 반환
     * @return 장애물 회피기의 포인터
     */
    ObstacleAvoider* GetObstacleAvoider(void){
		ObstacleAvoider* avoider = paramAvoider;
		return avoider;
	}

    /**
     * 경로 추종에 사용되는 최고 운행 속도를 설정
     * @param velocity 최고 운행 속도 (선속도 기본값: 0.3, 각속도 기본값: MATH_DEG2RAD(45)) (선속도 단위: [m/s], 각속도 단위: [rad/s])
     */
    void SetMaxVelocity(const Polar& velocity){ paramMaxVelocity = velocity; }

    /**
     * 경로 추종에 사용되는 최고 운행 속도를 반환
     * @return 최고 운행 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s])
     */
    Polar GetMaxVelocity(void){
		Polar velocity = paramMaxVelocity;
		return velocity;
	}

    /**
     * 경로 추종에 사용되는 최고 운행 가속도를 설정
     * @param acceleration 최고 운행 가속도 (선속도 기본값: 0.3, 각속도 기본값: MATH_DEG2RAD(45)) (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2])
     */
    void SetMaxAcceleration(const Polar& acceleration){ paramMaxAcceleration = acceleration; }

    /**
     * 경로 추종에 사용되는 최고 운행 가속도를 반환
     * @return 최고 운행 가속도 (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2])
     */
    Polar GetMaxAcceleration(void){	
		Polar acceleration = paramMaxAcceleration;
		return acceleration;
	}

protected:
    /** 경로를 담고 있는 버퍼의 포인터 */
    Point* pathData;

    /** 경로의 길이 */
    int pathSize;

    /** 장애물 회피기 */
    ObstacleAvoider* paramAvoider;

    /** 최고 운행 속도 (선속도 단위: [m/s], 각속도 단위: [rad/s]) */
    Polar paramMaxVelocity;

    /** 최고 운행 가속도 (선가속도 단위: [m/s^2], 각가속도 단위: [rad/s^2]) */
    Polar paramMaxAcceleration;

};


/**
 * <B>Map Builder Interface</B>
 *
 * <B>지도 작성기</B>에 필요한 기본 함수를 정의한 인터페이스이다.
 *
 * 지도 작성기는 거리 감지 데이터와 로봇의 자세 데이터를 이용하여 로봇이 동작하는 환경의 지도를 작성한다.
 * 거리 감지 데이터는 RangeDataCase::SetRangeData 함수를 통해 입력 받고, 자세 데이터는 Mapper::SetPose 함수를 통해 입력 받는다.
 */
class MapBuilder : public RangeDataCase
{
public:
    /**
     * 자세 데이터를 입력
     * @param pose 자세 데이터 (거리 단위: [m], 각도 단위: [rad])
     */
	virtual void SetPose(const Pose& pose){
        (void)pose;
		return ;
	}
};


/**
 * <B>Task Controller Interface</B>
 *
 * <B>작업 제어기</B>에 필요한 기본 함수를 정의한 인터페이스이다.
 *
 * 로봇 주행의 기본 목적은 로봇을 지도 상의 목적 위치와 방향각을 갖도록 계획, 제어하는 것이다.
 * 작업 제어기는 자세 데이터와 거리 감지 데이터를 입력 받고, 지도 작성기, 경로 계획기, 경로 추종기 등을 이용하여 이러한 목적을 달성한다.
 */
class TaskController : public PoseCase
{
public:   

};

}	// end namespace uron;
#endif