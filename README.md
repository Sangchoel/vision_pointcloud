# 🌐 Depth to PointCloud ROS Processor

> RGB-D 카메라의 Depth 이미지를 이용해 3D 포인트 클라우드를 생성하고,  
> 카메라 보정 정보(`/camera_info`)를 반영해 정확한 좌표계를 복원하는  
> **ROS + PCL 기반 실시간 포인트 클라우드 처리 노드**입니다.

---

## 📷 주요 기능

- `depth/image_raw` + `camera_info` → 3D 포인트 계산
- Depth 스케일 변환 (mm → meter)
- 카메라 내파라미터 (fx, fy, cx, cy) 기반 좌표 복원
- `PCL` 기반 다운샘플링 (VoxelGrid)
- `PCLVisualizer`로 실시간 3D 렌더링

---

## 📁 코드 구성

| 파일 | 설명 |
|------|------|
| `pointcloud_processor.cpp` | 핵심 노드 코드 (PointCloud 생성 및 시각화) |

---

## ⚙️ 핵심 수식

```cpp
float z = depth(i, j) / 1000.0f; // [mm → m]
x = -(j - cx) * z / fx;
y = -(i - cy) * z / fy;
