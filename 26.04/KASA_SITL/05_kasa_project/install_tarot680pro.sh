#!/usr/bin/env bash
#
# TAROT Iron Man 680 Pro - PX4 SITL (Gazebo Classic) 모델 설치 스크립트
#
#   사용법:  bash install_tarot680pro.sh [PX4-Autopilot 경로]
#   기본값:  ~/PX4-Autopilot
#
#   하는 일:
#     1) models/tarot680pro/{tarot680pro.sdf, model.config} 생성
#     2) init.d-posix/airframes/6021_gazebo-classic_tarot680pro 생성
#     3) airframes/CMakeLists.txt 에 에어프레임 등록
#     4) 빌드 타겟 목록(cmake)에 tarot680pro 등록
#   모두 멱등(idempotent) — 여러 번 돌려도 중복 안 됨.
#
set -euo pipefail

PX4_DIR="${1:-$HOME/PX4-Autopilot}"
MODEL="tarot680pro"
AIRFRAME_ID="6021"
AIRFRAME_FILE="${AIRFRAME_ID}_gazebo-classic_${MODEL}"

say()  { printf '\033[1;32m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[!]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[x]\033[0m %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- 0. 경로 확인
[ -d "$PX4_DIR" ] || die "PX4 디렉토리 없음: $PX4_DIR   (인자로 경로를 넘기세요)"
[ -f "$PX4_DIR/Tools/setup/ubuntu.sh" ] || warn "PX4-Autopilot 루트가 맞는지 확인하세요: $PX4_DIR"

# 모델 디렉토리 (PX4 버전마다 위치가 다름)
for c in \
  "Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" \
  "Tools/simulation/gazebo/sitl_gazebo/models" \
  "Tools/sitl_gazebo/models"
do
  if [ -d "$PX4_DIR/$c" ]; then MODELS_DIR="$PX4_DIR/$c"; break; fi
done
[ -n "${MODELS_DIR:-}" ] || die "Gazebo Classic 모델 디렉토리를 못 찾음. PX4 버전 확인 필요."

AIRFRAMES_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
[ -d "$AIRFRAMES_DIR" ] || die "에어프레임 디렉토리 없음: $AIRFRAMES_DIR"

say "PX4      : $PX4_DIR"
say "모델 경로 : $MODELS_DIR"

MODEL_DIR="$MODELS_DIR/$MODEL"
mkdir -p "$MODEL_DIR"

# ---------------------------------------------------------- 1. model.config
say "model.config 생성"
cat > "$MODEL_DIR/model.config" <<'MODELCONFIG'
<?xml version="1.0" ?>
<model>
  <name>tarot680pro</name>
  <version>1.0</version>
  <sdf version="1.5">tarot680pro.sdf</sdf>
  <author>
    <name></name>
    <email></email>
  </author>
  <description>
    TAROT Iron Man 680 Pro hexacopter (Hexa X) for PX4 SITL / Gazebo Classic.
    695 mm motor-to-motor folding CF frame, 6x Tarot 4006-620KV motors on 4S,
    13x5.5 CF props, ~2.8 kg all-up weight.
  </description>
</model>
MODELCONFIG

# ------------------------------------------------------------- 2. SDF 모델
say "tarot680pro.sdf 생성"
cat > "$MODEL_DIR/tarot680pro.sdf" <<'SDFEOF'
<?xml version="1.0" ?>
<!--
  TAROT Iron Man 680 Pro Hexacopter (Hexa X) - PX4 SITL / Gazebo Classic

  Published data this model is built from:
    Frame TL68P00        : 780-810 g, motor-to-motor 685-695 mm, 16 mm CF booms
    Motor TL68P02        : 4006 620KV, 82 g, 126 mOhm, 0.8 A idle @14.8 V
    Prop  TL2829 13x5.5  : max thrust 1580 g @ 17.4 A per motor on 4S
    ESC                  : 30 A class, ~40 g each
    Battery              : 4S 5000 mAh, ~475 g
    Vendor hover test    : 2.76 kg AUW, 14.5 min

  Derived:
    AUW             = 2.80 kg   (base_link 2.70 + 6 rotor links @ 0.017)
    arm radius      = 0.345 m
    max thrust/motor= 15.5 N    -> 6 motors = 93 N, T/W = 3.4
    hover throttle  ~ 0.54      (matches real-world 50-55% stick)
-->
<sdf version='1.5'>
  <model name='tarot680pro'>

    <link name='base_link'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>2.70</mass>
        <inertia>
          <ixx>0.062</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.062</iyy><iyz>0</iyz><izz>0.122</izz>
        </inertia>
      </inertial>

      <collision name='base_collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>0.195 0.195 0.06</size></box></geometry>
      </collision>
      <visual name='base_visual'>
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>0.195 0.195 0.06</size></box></geometry>
        <material><script><name>Gazebo/DarkGrey</name></script></material>
      </visual>

      <visual name='arm_30'>
        <pose>0.1494 -0.08625 0 0 1.5708 -0.5236</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.345</length></cylinder></geometry>
        <material><script><name>Gazebo/Orange</name></script></material>
      </visual>
      <visual name='arm_90'>
        <pose>0 -0.1725 0 0 1.5708 -1.5708</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.345</length></cylinder></geometry>
        <material><script><name>Gazebo/Orange</name></script></material>
      </visual>
      <visual name='arm_150'>
        <pose>-0.1494 -0.08625 0 0 1.5708 -2.6180</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.345</length></cylinder></geometry>
        <material><script><name>Gazebo/Black</name></script></material>
      </visual>
      <visual name='arm_210'>
        <pose>-0.1494 0.08625 0 0 1.5708 -3.6652</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.345</length></cylinder></geometry>
        <material><script><name>Gazebo/Black</name></script></material>
      </visual>
      <visual name='arm_270'>
        <pose>0 0.1725 0 0 1.5708 -4.7124</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.345</length></cylinder></geometry>
        <material><script><name>Gazebo/Black</name></script></material>
      </visual>
      <visual name='arm_330'>
        <pose>0.1494 0.08625 0 0 1.5708 -5.7596</pose>
        <geometry><cylinder><radius>0.008</radius><length>0.345</length></cylinder></geometry>
        <material><script><name>Gazebo/Orange</name></script></material>
      </visual>

      <collision name='skid_left'>
        <pose>0 0.13 -0.18 0 0 0</pose>
        <geometry><box><size>0.30 0.02 0.02</size></box></geometry>
      </collision>
      <visual name='skid_left_visual'>
        <pose>0 0.13 -0.18 0 0 0</pose>
        <geometry><box><size>0.30 0.02 0.02</size></box></geometry>
        <material><script><name>Gazebo/Black</name></script></material>
      </visual>
      <collision name='skid_right'>
        <pose>0 -0.13 -0.18 0 0 0</pose>
        <geometry><box><size>0.30 0.02 0.02</size></box></geometry>
      </collision>
      <visual name='skid_right_visual'>
        <pose>0 -0.13 -0.18 0 0 0</pose>
        <geometry><box><size>0.30 0.02 0.02</size></box></geometry>
        <material><script><name>Gazebo/Black</name></script></material>
      </visual>
    </link>

    <link name='imu_link'>
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.015</mass>
        <inertia>
          <ixx>1e-05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-05</iyy><iyz>0</iyz><izz>1e-05</izz>
        </inertia>
      </inertial>
      <sensor name='imu_sensor' type='imu'>
        <always_on>1</always_on>
        <update_rate>250</update_rate>
      </sensor>
    </link>
    <joint name='imu_joint' type='revolute'>
      <child>imu_link</child>
      <parent>base_link</parent>
      <axis>
        <xyz>1 0 0</xyz>
        <limit><upper>0</upper><lower>0</lower></limit>
      </axis>
    </joint>

    <!--
      Hexa X, alternating CCW/CW around the circle.
        idx 0  front_right  30 deg  CCW
        idx 1  rear_left   210 deg  CW
        idx 2  front_left  330 deg  CW
        idx 3  rear_right  150 deg  CCW
        idx 4  right        90 deg  CW
        idx 5  left        270 deg  CCW
      Gazebo frame is x-forward / y-LEFT (y negated vs PX4 FRD).
    -->

    <link name='rotor_0'>
      <pose>0.2988 -0.1725 0.06 0 0 0</pose>
      <inertial><mass>0.017</mass>
        <inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>1.55e-04</iyy><iyz>0</iyz><izz>1.55e-04</izz></inertia>
      </inertial>
      <collision name='rotor_0_collision'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
      </collision>
      <visual name='rotor_0_visual'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
        <material><script><name>Gazebo/Orange</name></script></material>
      </visual>
    </link>
    <joint name='rotor_0_joint' type='revolute'>
      <parent>base_link</parent><child>rotor_0</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>

    <link name='rotor_1'>
      <pose>-0.2988 0.1725 0.06 0 0 0</pose>
      <inertial><mass>0.017</mass>
        <inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>1.55e-04</iyy><iyz>0</iyz><izz>1.55e-04</izz></inertia>
      </inertial>
      <collision name='rotor_1_collision'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
      </collision>
      <visual name='rotor_1_visual'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
        <material><script><name>Gazebo/Black</name></script></material>
      </visual>
    </link>
    <joint name='rotor_1_joint' type='revolute'>
      <parent>base_link</parent><child>rotor_1</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>

    <link name='rotor_2'>
      <pose>0.2988 0.1725 0.06 0 0 0</pose>
      <inertial><mass>0.017</mass>
        <inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>1.55e-04</iyy><iyz>0</iyz><izz>1.55e-04</izz></inertia>
      </inertial>
      <collision name='rotor_2_collision'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
      </collision>
      <visual name='rotor_2_visual'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
        <material><script><name>Gazebo/Orange</name></script></material>
      </visual>
    </link>
    <joint name='rotor_2_joint' type='revolute'>
      <parent>base_link</parent><child>rotor_2</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>

    <link name='rotor_3'>
      <pose>-0.2988 -0.1725 0.06 0 0 0</pose>
      <inertial><mass>0.017</mass>
        <inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>1.55e-04</iyy><iyz>0</iyz><izz>1.55e-04</izz></inertia>
      </inertial>
      <collision name='rotor_3_collision'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
      </collision>
      <visual name='rotor_3_visual'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
        <material><script><name>Gazebo/Black</name></script></material>
      </visual>
    </link>
    <joint name='rotor_3_joint' type='revolute'>
      <parent>base_link</parent><child>rotor_3</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>

    <link name='rotor_4'>
      <pose>0 -0.345 0.06 0 0 0</pose>
      <inertial><mass>0.017</mass>
        <inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>1.55e-04</iyy><iyz>0</iyz><izz>1.55e-04</izz></inertia>
      </inertial>
      <collision name='rotor_4_collision'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
      </collision>
      <visual name='rotor_4_visual'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
        <material><script><name>Gazebo/Orange</name></script></material>
      </visual>
    </link>
    <joint name='rotor_4_joint' type='revolute'>
      <parent>base_link</parent><child>rotor_4</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>

    <link name='rotor_5'>
      <pose>0 0.345 0.06 0 0 0</pose>
      <inertial><mass>0.017</mass>
        <inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>1.55e-04</iyy><iyz>0</iyz><izz>1.55e-04</izz></inertia>
      </inertial>
      <collision name='rotor_5_collision'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
      </collision>
      <visual name='rotor_5_visual'>
        <geometry><cylinder><length>0.005</length><radius>0.165</radius></cylinder></geometry>
        <material><script><name>Gazebo/Black</name></script></material>
      </visual>
    </link>
    <joint name='rotor_5_joint' type='revolute'>
      <parent>base_link</parent><child>rotor_5</child>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>

    <!--
      motorConstant  4.05e-05  thrust[N] = k*w^2 -> 15.5 N @ 618 rad/s
      maxRotVelocity 620 rad/s (~5900 rpm loaded; 620KV x 14.8V = 9176 rpm no-load)
      momentConstant 0.018     (Cq/Ct * D for a 13 in prop)
      timeConstant   0.02/0.04 (4006 + 13x5.5 is heavy, slower than a 10 in setup)
    -->
    <plugin name='rotor_0_motor_model' filename='libgazebo_motor_model.so'>
      <robotNamespace/><jointName>rotor_0_joint</jointName><linkName>rotor_0</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.02</timeConstantUp><timeConstantDown>0.04</timeConstantDown>
      <maxRotVelocity>620</maxRotVelocity>
      <motorConstant>4.05e-05</motorConstant><momentConstant>0.018</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>1.0e-04</rotorDragCoefficient>
      <rollingMomentCoefficient>1.0e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/0</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
    <plugin name='rotor_1_motor_model' filename='libgazebo_motor_model.so'>
      <robotNamespace/><jointName>rotor_1_joint</jointName><linkName>rotor_1</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.02</timeConstantUp><timeConstantDown>0.04</timeConstantDown>
      <maxRotVelocity>620</maxRotVelocity>
      <motorConstant>4.05e-05</motorConstant><momentConstant>0.018</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>1</motorNumber>
      <rotorDragCoefficient>1.0e-04</rotorDragCoefficient>
      <rollingMomentCoefficient>1.0e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/1</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
    <plugin name='rotor_2_motor_model' filename='libgazebo_motor_model.so'>
      <robotNamespace/><jointName>rotor_2_joint</jointName><linkName>rotor_2</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.02</timeConstantUp><timeConstantDown>0.04</timeConstantDown>
      <maxRotVelocity>620</maxRotVelocity>
      <motorConstant>4.05e-05</motorConstant><momentConstant>0.018</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>2</motorNumber>
      <rotorDragCoefficient>1.0e-04</rotorDragCoefficient>
      <rollingMomentCoefficient>1.0e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/2</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
    <plugin name='rotor_3_motor_model' filename='libgazebo_motor_model.so'>
      <robotNamespace/><jointName>rotor_3_joint</jointName><linkName>rotor_3</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.02</timeConstantUp><timeConstantDown>0.04</timeConstantDown>
      <maxRotVelocity>620</maxRotVelocity>
      <motorConstant>4.05e-05</motorConstant><momentConstant>0.018</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>3</motorNumber>
      <rotorDragCoefficient>1.0e-04</rotorDragCoefficient>
      <rollingMomentCoefficient>1.0e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/3</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
    <plugin name='rotor_4_motor_model' filename='libgazebo_motor_model.so'>
      <robotNamespace/><jointName>rotor_4_joint</jointName><linkName>rotor_4</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.02</timeConstantUp><timeConstantDown>0.04</timeConstantDown>
      <maxRotVelocity>620</maxRotVelocity>
      <motorConstant>4.05e-05</motorConstant><momentConstant>0.018</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>4</motorNumber>
      <rotorDragCoefficient>1.0e-04</rotorDragCoefficient>
      <rollingMomentCoefficient>1.0e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/4</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>
    <plugin name='rotor_5_motor_model' filename='libgazebo_motor_model.so'>
      <robotNamespace/><jointName>rotor_5_joint</jointName><linkName>rotor_5</linkName>
      <turningDirection>ccw</turningDirection>
      <timeConstantUp>0.02</timeConstantUp><timeConstantDown>0.04</timeConstantDown>
      <maxRotVelocity>620</maxRotVelocity>
      <motorConstant>4.05e-05</motorConstant><momentConstant>0.018</momentConstant>
      <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
      <motorNumber>5</motorNumber>
      <rotorDragCoefficient>1.0e-04</rotorDragCoefficient>
      <rollingMomentCoefficient>1.0e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>/motor_speed/5</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
    </plugin>

    <plugin name='magnetometer_plugin' filename='libgazebo_magnetometer_plugin.so'>
      <robotNamespace/><pubRate>100</pubRate>
      <noiseDensity>0.0004</noiseDensity><randomWalk>6.4e-06</randomWalk>
      <biasCorrelationTime>600</biasCorrelationTime><magTopic>/mag</magTopic>
    </plugin>
    <plugin name='barometer_plugin' filename='libgazebo_barometer_plugin.so'>
      <robotNamespace/><pubRate>50</pubRate><baroTopic>/baro</baroTopic>
      <baroDriftPaPerSec>0</baroDriftPaPerSec>
    </plugin>
    <plugin name='gps_plugin' filename='libgazebo_gps_plugin.so'>
      <robotNamespace/><gpsNoise>true</gpsNoise>
    </plugin>

    <plugin name='mavlink_interface' filename='libgazebo_mavlink_interface.so'>
      <robotNamespace/>
      <imuSubTopic>/imu</imuSubTopic>
      <magSubTopic>/mag</magSubTopic>
      <baroSubTopic>/baro</baroSubTopic>
      <mavlink_addr>INADDR_ANY</mavlink_addr>
      <mavlink_tcp_port>4560</mavlink_tcp_port>
      <mavlink_udp_port>14560</mavlink_udp_port>
      <serialEnabled>0</serialEnabled>
      <serialDevice>/dev/ttyACM0</serialDevice>
      <baudRate>921600</baudRate>
      <qgc_addr>INADDR_ANY</qgc_addr>
      <qgc_udp_port>14550</qgc_udp_port>
      <sdk_addr>INADDR_ANY</sdk_addr>
      <sdk_udp_port>14540</sdk_udp_port>
      <hil_mode>0</hil_mode>
      <hil_state_level>0</hil_state_level>
      <enable_lockstep>1</enable_lockstep>
      <use_tcp>1</use_tcp>
      <motorSpeedCommandPubTopic>/gazebo/command/motor_speed</motorSpeedCommandPubTopic>
      <control_channels>
        <channel name='rotor_0'><input_index>0</input_index><input_offset>0</input_offset>
          <input_scaling>620</input_scaling><zero_position_disarmed>0</zero_position_disarmed>
          <zero_position_armed>50</zero_position_armed>
          <joint_control_type>velocity</joint_control_type></channel>
        <channel name='rotor_1'><input_index>1</input_index><input_offset>0</input_offset>
          <input_scaling>620</input_scaling><zero_position_disarmed>0</zero_position_disarmed>
          <zero_position_armed>50</zero_position_armed>
          <joint_control_type>velocity</joint_control_type></channel>
        <channel name='rotor_2'><input_index>2</input_index><input_offset>0</input_offset>
          <input_scaling>620</input_scaling><zero_position_disarmed>0</zero_position_disarmed>
          <zero_position_armed>50</zero_position_armed>
          <joint_control_type>velocity</joint_control_type></channel>
        <channel name='rotor_3'><input_index>3</input_index><input_offset>0</input_offset>
          <input_scaling>620</input_scaling><zero_position_disarmed>0</zero_position_disarmed>
          <zero_position_armed>50</zero_position_armed>
          <joint_control_type>velocity</joint_control_type></channel>
        <channel name='rotor_4'><input_index>4</input_index><input_offset>0</input_offset>
          <input_scaling>620</input_scaling><zero_position_disarmed>0</zero_position_disarmed>
          <zero_position_armed>50</zero_position_armed>
          <joint_control_type>velocity</joint_control_type></channel>
        <channel name='rotor_5'><input_index>5</input_index><input_offset>0</input_offset>
          <input_scaling>620</input_scaling><zero_position_disarmed>0</zero_position_disarmed>
          <zero_position_armed>50</zero_position_armed>
          <joint_control_type>velocity</joint_control_type></channel>
      </control_channels>
    </plugin>

    <static>0</static>
  </model>
</sdf>
SDFEOF

# --------------------------------------------------------- 3. 에어프레임 파일
say "에어프레임 $AIRFRAME_FILE 생성"
cat > "$AIRFRAMES_DIR/$AIRFRAME_FILE" <<'AFEOF'
#!/bin/sh
#
# @name TAROT Iron Man 680 Pro Hexacopter (SITL)
#
# @type Hexarotor x
# @class Copter
#

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gazebo-classic}
PX4_GZ_MODEL=${PX4_GZ_MODEL:=tarot680pro}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=gazebo-classic_tarot680pro}

param set-default SYS_CTRL_ALLOC 1

# ---- Hexarotor X geometry, arm radius 0.345 m, body frame FRD ----
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 6

param set-default CA_ROTOR0_PX   0.2988
param set-default CA_ROTOR0_PY   0.1725
param set-default CA_ROTOR0_KM   0.05

param set-default CA_ROTOR1_PX  -0.2988
param set-default CA_ROTOR1_PY  -0.1725
param set-default CA_ROTOR1_KM  -0.05

param set-default CA_ROTOR2_PX   0.2988
param set-default CA_ROTOR2_PY  -0.1725
param set-default CA_ROTOR2_KM  -0.05

param set-default CA_ROTOR3_PX  -0.2988
param set-default CA_ROTOR3_PY   0.1725
param set-default CA_ROTOR3_KM   0.05

param set-default CA_ROTOR4_PX   0.0
param set-default CA_ROTOR4_PY   0.345
param set-default CA_ROTOR4_KM  -0.05

param set-default CA_ROTOR5_PX   0.0
param set-default CA_ROTOR5_PY  -0.345
param set-default CA_ROTOR5_KM   0.05

param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104
param set-default SIM_GZ_EC_FUNC5 105
param set-default SIM_GZ_EC_FUNC6 106

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150
param set-default SIM_GZ_EC_MIN5 150
param set-default SIM_GZ_EC_MIN6 150

param set-default SIM_GZ_EC_MAX1 1000
param set-default SIM_GZ_EC_MAX2 1000
param set-default SIM_GZ_EC_MAX3 1000
param set-default SIM_GZ_EC_MAX4 1000
param set-default SIM_GZ_EC_MAX5 1000
param set-default SIM_GZ_EC_MAX6 1000

# ---- Hover: AUW 2.8 kg vs 6 x 1.58 kg max thrust -> T/W 3.4 ----
param set-default MPC_THR_HOVER 0.54

# ---- Starting rate gains for a 2.8 kg / 695 mm hexa ----
param set-default MC_ROLLRATE_P  0.11
param set-default MC_ROLLRATE_I  0.22
param set-default MC_ROLLRATE_D  0.004
param set-default MC_PITCHRATE_P 0.11
param set-default MC_PITCHRATE_I 0.22
param set-default MC_PITCHRATE_D 0.004
param set-default MC_YAWRATE_P   0.18
param set-default MC_YAWRATE_I   0.10

param set-default MC_ROLL_P      6.0
param set-default MC_PITCH_P     6.0
param set-default MC_YAW_P       2.6

# ---- Camera-platform limits, not a racer ----
param set-default MPC_XY_VEL_MAX 8.0
param set-default MPC_Z_VEL_MAX_UP 3.0
param set-default MPC_Z_VEL_MAX_DN 1.5
param set-default MPC_TILTMAX_AIR 35.0
param set-default MPC_LAND_SPEED 0.7
AFEOF
chmod +x "$AIRFRAMES_DIR/$AIRFRAME_FILE" 2>/dev/null || true

# ------------------------------------------ 4. airframes/CMakeLists.txt 등록
AF_CMAKE="$AIRFRAMES_DIR/CMakeLists.txt"
if [ -f "$AF_CMAKE" ]; then
  if grep -q "$AIRFRAME_FILE" "$AF_CMAKE"; then
    say "airframes/CMakeLists.txt — 이미 등록됨"
  else
    # typhoon_h480 줄 바로 뒤에 삽입 (없으면 px4_add_romfs_files( 다음 줄)
    if grep -q "gazebo-classic_typhoon_h480" "$AF_CMAKE"; then
      sed -i "/gazebo-classic_typhoon_h480/a\\\t$AIRFRAME_FILE" "$AF_CMAKE"
    else
      sed -i "0,/px4_add_romfs_files(/s//px4_add_romfs_files(\n\t$AIRFRAME_FILE/" "$AF_CMAKE"
    fi
    say "airframes/CMakeLists.txt — 등록 완료"
  fi
else
  warn "airframes/CMakeLists.txt 없음 — 수동 등록 필요"
fi

# ------------------------------------------- 5. 빌드 타겟 목록(cmake) 등록
# PX4 버전마다 파일이 다르므로 typhoon_h480 이 들어있는 cmake 를 찾아서 붙인다.
TARGET_CMAKE=""
for f in \
  "$PX4_DIR/src/modules/simulation/gazebo-classic/CMakeLists.txt" \
  "$PX4_DIR/platforms/posix/cmake/sitl_target.cmake" \
  "$PX4_DIR/src/modules/simulation/simulator_mavlink/CMakeLists.txt"
do
  if [ -f "$f" ] && grep -q "typhoon_h480" "$f"; then TARGET_CMAKE="$f"; break; fi
done
if [ -z "$TARGET_CMAKE" ]; then
  TARGET_CMAKE="$(grep -rl --include='*.cmake' --include='CMakeLists.txt' \
                   'typhoon_h480' "$PX4_DIR/src" "$PX4_DIR/platforms" 2>/dev/null | head -n1 || true)"
fi

if [ -n "$TARGET_CMAKE" ]; then
  if grep -q "$MODEL" "$TARGET_CMAKE"; then
    say "빌드 타겟 — 이미 등록됨 ($(basename "$TARGET_CMAKE"))"
  else
    sed -i "0,/typhoon_h480/s//typhoon_h480\n\t$MODEL/" "$TARGET_CMAKE"
    say "빌드 타겟 등록 완료 → $TARGET_CMAKE"
  fi
else
  warn "빌드 타겟 cmake 를 못 찾음. 아래 명령으로 직접 확인 후 'tarot680pro' 를 추가하세요:"
  warn "  grep -rn typhoon_h480 $PX4_DIR/src $PX4_DIR/platforms --include=*.cmake --include=CMakeLists.txt"
fi

# --------------------------------------------------------------- 6. 요약 출력
cat <<EOM

────────────────────────────────────────────────────────────
 설치 완료

   SDF        : $MODEL_DIR/tarot680pro.sdf
   model.cfg  : $MODEL_DIR/model.config
   airframe   : $AIRFRAMES_DIR/$AIRFRAME_FILE

 실행:

   cd $PX4_DIR
   make px4_sitl gazebo-classic_$MODEL

 기체 제원 (실제 스펙 기반)
   AUW            2.80 kg        모터간 거리  695 mm
   모터           4006 620KV x6  프롭        13x5.5 CF
   모터당 최대추력 1.58 kg        T/W         3.4
   호버 스로틀    ~0.54          암 반경      0.345 m

 ⚠ 첫 비행 체크: arm 후 기체가 한쪽으로 계속 회전(yaw spin)하면
   로터 회전방향 매핑이 PX4 hexa_x 와 어긋난 것입니다.
   tarot680pro.sdf 의 turningDirection 에서 ccw <-> cw 를
   전부 뒤집고 다시 빌드하세요. 롤/피치는 정상인데 yaw 만
   이상할 때가 바로 이 경우입니다.
────────────────────────────────────────────────────────────
EOM
