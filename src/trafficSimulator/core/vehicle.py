import uuid  # uuid 라이브러리를 임포트, 고유 식별자 생성에 사용
import numpy as np  # numpy 라이브러리를 임포트, 수학적 계산에 사용

class Vehicle:
    def __init__(self, config={}):
        # 디폴트 설정을 지정하는 메소드 호출
        self.set_default_config()

        # 제공된 config 딕셔너리를 통해 인스턴스 속성 업데이트
        for attr, val in config.items():
            setattr(self, attr, val)

        # 차량의 주요 속성 계산 초기화
        self.init_properties()

    def set_default_config(self):
        # 각 차량에 대한 고유 ID 생성
        self.id = uuid.uuid4()

        # 차량의 기본 물리적 특성 설정
        self.l = 4  # 차량 길이 (미터)
        self.s0 = 4  # 안전 거리 (미터)
        self.T = 1  # 시간 헤드웨이 (초)
        self.v_max = 16.6  # 최대 속도 (m/s)
        self.a_max = 1.44  # 최대 가속도 (m/s^2)
        self.b_max = 4.61  # 최대 감속도 (m/s^2)

        # 차량의 운행 경로 및 상태 관련 설정
        self.path = []  # 차량 경로 (도로 세그먼트 목록)
        self.current_road_index = 0  # 현재 도로의 인덱스

        # 동적인 차량의 위치, 속도, 가속도
        self.x = 0  # 초기 위치
        self.v = 0  # 초기 속도
        self.a = 0  # 초기 가속도
        self.stopped = False  # 차량 정지 상태 표시

    def init_properties(self):
        # 가속도와 감속도의 기하 평균 계산, 추후 계산에 사용
        self.sqrt_ab = 2 * np.sqrt(self.a_max * self.b_max)
        self._v_max = self.v_max  # 내부 사용을 위한 최대 속도 변수 설정

    def update(self, lead, dt):
        # 차량의 위치와 속도 업데이트
        if self.v + self.a * dt < 0:
            # 속도가 0 이하가 되는 경우 위치 업데이트 및 속도를 0으로 설정
            self.x -= 1 / 2 * self.v * self.v / self.a 
            self.v = 0
        else:
            # 일반적인 위치 및 속도 업데이트
            self.v += self.a * dt
            self.x += self.v * dt + self.a * dt * dt / 2

        # 가속도 업데이트
        alpha = 0
        if lead:
            # 앞차와의 거리 및 속도 차이 계산
            delta_x = lead.x - self.x - lead.l
            delta_v = self.v - lead.v

            # 안전 거리를 고려한 가속도 계산
            alpha = (self.s0\
                     + max(0, self.T * self.v\
                           + delta_v * self.v / self.sqrt_ab)) / delta_x

        # 새로운 가속도 계산
        self.a = self.a_max * (1 - (self.v / self.v_max)**4 - alpha**2)

        # 정지명령을 받았을 때 서서히 감속하도록 함 
        if self.stopped:
            self.a = -self.b_max * self.v / self.v_max


    # # PPT에 있는 IDM을 그대로 구현한 코드 
    # def update(self, lead, dt):
    #     if lead:
    #         # 앞차와의 거리 및 속도 차이 계산
    #         delta_x = lead.x - self.x - lead.l
    #         delta_v = self.v - lead.v

    #         # 바람직한 거리 s* 계산
    #         s_star = self.s0 + self.v * self.T + (self.v * delta_v) / (2 * np.sqrt(self.a_max * self.b_max))

    #         # 새로운 가속도 계산
    #         self.a = self.a_max * (1 - (self.v / self.v_max)**self.delta - (s_star / delta_x)**2)

    #     else:
    #         # 앞차가 없는 경우 최대 가속
    #         self.a = self.a_max * (1 - (self.v / self.v_max)**self.delta)