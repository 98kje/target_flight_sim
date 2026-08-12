# target_flight_sim

표적 기체 비행 시뮬레이터 (ROS 2 Humble). 실행하면 즉시 위치/속도 데이터가
토픽으로 발행됩니다. GCS나 별도 트리거는 필요 없습니다.

## 빌드 & 실행

```bash
cd target_flight_sim_ws
colcon build --packages-select target_flight_sim
source install/setup.bash
ros2 launch target_flight_sim target_sim.launch.py
```

## 설정

`config/target_sim_params.yaml` 에서 표적의 출발점/목표점/속도/도착 후 동작을
바꿀 수 있습니다. 재빌드는 필요 없고, 파일을 수정한 뒤 다시 `ros2 launch`만
실행하면 됩니다.

| 파라미터 | 설명 |
|---|---|
| `start_lat_deg` / `start_lon_deg` / `start_alt_m` | 출발점 위경도(deg)·고도(m) |
| `goal_lat_deg` / `goal_lon_deg` / `goal_alt_m` | 목표점 위경도(deg)·고도(m) |
| `speed_mps` | 출발→목표 등속 비행 속도 [m/s] |
| `update_hz` | 위치/속도 발행 주기 [Hz] |
| `on_arrival` | 목표 도달 후 동작: `hold`(정지) / `loop`(대기 후 재출발) / `reverse`(왕복) |
| `loop_pause_s` | `on_arrival: loop` 일 때 재출발 전 대기 시간 [s] |

launch 인자로 config 파일 경로를 바꿀 수도 있습니다:

```bash
ros2 launch target_flight_sim target_sim.launch.py params_file:=/path/to/your.yaml
```

## 발행 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/target/global_position` | `sensor_msgs/NavSatFix` | 표적 현재 위치 (lat/lon/alt) |
| `/target/velocity` | `geometry_msgs/TwistStamped` | 표적 현재 속도, ENU(East-North-Up) [m/s] |
| `/target/status` | `std_msgs/String` | 상태 전이 시 1회: `EN_ROUTE` / `ARRIVED` |

표준 ROS 2 메시지만 사용하므로 별도 메시지 패키지 의존성 없이 바로 구독해서
쓸 수 있습니다.
