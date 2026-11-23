#!/usr/bin/env python3
"""
iris.sdf 에 LiDAR 센서(link+joint)를 직접 병합해서
새 모델 iris_lidar 를 만드는 스크립트.

카메라 때 겪었던 문제(<include>+<joint>로 다른 모델을 iris에 갖다붙이면
libgazebo_ros_* 플러그인이 조용히 초기화를 건너뛰는 버그)를 피하기 위해,
iris.sdf 안에 lidar_link/lidar_joint를 텍스트 레벨에서 직접 삽입한다.

사용법:
  python3 merge_iris_lidar.py /path/to/PX4-Autopilot

동작:
  1) <PX4-Autopilot>/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf 를 읽음
  2) </model> 태그 바로 앞에 lidar_link + lidar_joint 블록을 삽입
  3) 모델명을 iris_lidar 로 변경
  4) models/iris_lidar/iris_lidar.sdf 와 model.config 로 새로 저장
"""

import sys
import os
import shutil

LIDAR_BLOCK = """
    <link name="lidar_link">
      <pose>0 0 -0.05 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <sensor name="lidar_sensor" type="gpu_ray">
        <pose>0 0 0 0 1.5708 0</pose>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <resolution>1</resolution>
              <min_angle>-0.2618</min_angle>
              <max_angle>0.2618</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>100.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace>/lidar</namespace>
            <remapping>~/out:=points</remapping>
          </ros>
          <output_type>sensor_msgs/PointCloud2</output_type>
          <frame_name>lidar_link</frame_name>
        </plugin>
      </sensor>
    </link>
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>
"""


def main():
    if len(sys.argv) != 2:
        print("사용법: python3 merge_iris_lidar.py /path/to/PX4-Autopilot")
        sys.exit(1)

    px4_root = sys.argv[1]
    models_dir = os.path.join(
        px4_root,
        "Tools", "simulation", "gazebo-classic",
        "sitl_gazebo-classic", "models",
    )
    iris_dir = os.path.join(models_dir, "iris")
    iris_sdf_path = os.path.join(iris_dir, "iris.sdf")

    if not os.path.isfile(iris_sdf_path):
        print(f"iris.sdf를 찾을 수 없습니다: {iris_sdf_path}")
        print("PX4-Autopilot 경로가 맞는지 확인해주세요.")
        sys.exit(1)

    with open(iris_sdf_path, "r") as f:
        content = f.read()

    if "lidar_link" in content:
        print("이미 lidar_link가 포함된 것 같습니다. 원본 iris.sdf가 맞는지 확인해주세요.")
        sys.exit(1)

    # </model> 태그 직전에 라이다 블록 삽입 (마지막 </model>이 최상위 모델을 닫는 태그라고 가정)
    last_model_close = content.rfind("</model>")
    if last_model_close == -1:
        print("iris.sdf에서 </model> 태그를 찾을 수 없습니다. 파일 형식을 확인해주세요.")
        sys.exit(1)

    new_content = (
        content[:last_model_close]
        + LIDAR_BLOCK
        + content[last_model_close:]
    )

    # 모델명 iris -> iris_lidar 로 변경 (최상위 <model name='iris'>만 대상)
    new_content = new_content.replace(
        "<model name='iris'>", "<model name='iris_lidar'>", 1
    )
    new_content = new_content.replace(
        '<model name="iris">', '<model name="iris_lidar">', 1
    )

    # 새 모델 디렉토리 생성
    new_model_dir = os.path.join(models_dir, "iris_lidar")
    os.makedirs(new_model_dir, exist_ok=True)

    new_sdf_path = os.path.join(new_model_dir, "iris_lidar.sdf")
    with open(new_sdf_path, "w") as f:
        f.write(new_content)

    # model.config 복사 및 수정 (있는 경우)
    src_config = os.path.join(iris_dir, "model.config")
    dst_config = os.path.join(new_model_dir, "model.config")
    if os.path.isfile(src_config):
        with open(src_config, "r") as f:
            cfg = f.read()
        cfg = cfg.replace(">iris<", ">iris_lidar<")
        cfg = cfg.replace("iris.sdf", "iris_lidar.sdf")
        with open(dst_config, "w") as f:
            f.write(cfg)
    else:
        # model.config가 없으면 최소한의 버전을 새로 만듦
        with open(dst_config, "w") as f:
            f.write(
                "<?xml version=\"1.0\"?>\n"
                "<model>\n"
                "  <name>iris_lidar</name>\n"
                "  <version>1.0</version>\n"
                "  <sdf version='1.6'>iris_lidar.sdf</sdf>\n"
                "  <description>iris + lidar merged model</description>\n"
                "</model>\n"
            )

    # iris 모델이 참조하는 mesh/texture 등 기타 파일이 있으면 함께 복사 (sdf/config 제외)
    for fname in os.listdir(iris_dir):
        if fname in ("iris.sdf", "model.config"):
            continue
        src = os.path.join(iris_dir, fname)
        dst = os.path.join(new_model_dir, fname)
        if os.path.isdir(src):
            if not os.path.exists(dst):
                shutil.copytree(src, dst)
        else:
            if not os.path.exists(dst):
                shutil.copy2(src, dst)

    print(f"완료: {new_model_dir}")
    print("다음 명령으로 SITL 실행해서 확인하세요:")
    print(
        f'  ./Tools/simulation/gazebo-classic/sitl_run.sh '
        f'$(pwd)/build/px4_sitl_default/bin/px4 none iris_lidar none '
        f'$(pwd) $(pwd)/build/px4_sitl_default'
    )
    print("그 다음:")
    print("  ros2 topic list | grep points")
    print("  ros2 topic echo /lidar/points --no-arr")


if __name__ == "__main__":
    main()
