"""Projeção world (metros) → pixel normalizado usando parâmetros reais da câmera."""

import numpy as np


def world_to_pixel_with_camera(
    world_x: float,
    world_y: float,
    camera_height: float,
    camera_info,
) -> tuple:
    """
    Projeta coordenada do mapa para pixel normalizado YOLO usando intrínsecos reais.

    A câmera está posicionada em z=camera_height apontando para baixo (-Z).
    O robô está no plano z=0.

    Args:
        world_x, world_y: posição do robô no mapa (metros)
        camera_height: altura da câmera em relação ao chão (metros)
        camera_info: sensor_msgs/CameraInfo com fx, fy, cx, cy

    Returns:
        (cx_norm, cy_norm): centro normalizado [0,1] para YOLO, ou None se fora do frame
    """
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]
    img_w = camera_info.width
    img_h = camera_info.height

    # Câmera em pose <0 0 3 0 1.5708 0> (pitch=π/2, yaw=0):
    # Após R_y(π/2): X_cam=-Y_world, Y_cam=-X_world, Z_cam=-Z_world (aponta para baixo)
    # Vetor câmera→robô no world: (world_x, world_y, -camera_height)
    # Projetado nos eixos da câmera:
    cam_x = world_y       # X_cam = +Y_world (espelho corrigido)
    cam_y = -world_x      # Y_cam = -X_world
    cam_z = camera_height

    if cam_z <= 0:
        return None

    # Projeção pinhole
    u = fx * (cam_x / cam_z) + cx
    v = fy * (cam_y / cam_z) + cy

    # Normaliza para [0, 1]
    cx_norm = u / img_w
    cy_norm = v / img_h

    if not (0.0 <= cx_norm <= 1.0 and 0.0 <= cy_norm <= 1.0):
        return None

    return cx_norm, cy_norm


def pixel_to_world_with_camera(
    cx_norm: float,
    cy_norm: float,
    camera_height: float,
    camera_info,
) -> tuple:
    """
    Projeta pixel normalizado YOLO para coordenada do mapa, usando
    intrínsecos reais (fx, fy, cx, cy de camera_info, tipicamente vindos de
    camera_calibration.npz).

    Convenção de eixos validada empiricamente contra pixel_to_world_simple +
    world_x,world_y=raw_y,-raw_x (yolo_detector.py, produção): NÃO é a mesma
    convenção documentada em world_to_pixel_with_camera (aquela função, usada
    só por dataset_collector.py — não usado em produção — está espelhada em
    X quando comparada empiricamente contra a heurística validada; não
    corrigida aqui para não alterar esse consumidor legado sem necessidade).
    """
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]
    img_w = camera_info.width
    img_h = camera_info.height

    u = cx_norm * img_w
    v = cy_norm * img_h

    cam_x = (u - cx) / fx * camera_height
    cam_y = (v - cy) / fy * camera_height

    world_y = -cam_x
    world_x = -cam_y

    return world_x, world_y


def robot_bbox_normalized(camera_height: float, camera_info, robot_radius: float = 0.17) -> tuple:
    """
    Calcula w/h do bounding box do robô em coordenadas YOLO normalizadas.

    Usa o tamanho real do TurtleBot3 Waffle (raio ~0.17m) e a altura
    da câmera para projetar um bbox proporcional ao tamanho aparente.

    Returns:
        (w_norm, h_norm): tamanho normalizado do bbox
    """
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    img_w = camera_info.width
    img_h = camera_info.height

    diameter = robot_radius * 2.0
    w_pixels = fx * (diameter / camera_height)
    h_pixels = fy * (diameter / camera_height)

    return w_pixels / img_w, h_pixels / img_h


# Fallback simples para testes sem CameraInfo
def world_to_pixel_simple(
    pose_x: float,
    pose_y: float,
    camera_height: float = 3.0,
    horizontal_fov: float = 1.047,
    img_width: int = 640,
    img_height: int = 480,
) -> tuple:
    """
    Projeção simplificada para testes sem CameraInfo disponível.
    Usa FOV da câmera para calcular escala correta.
    """
    half_fov = horizontal_fov / 2.0
    map_half_width = camera_height * np.tan(half_fov)
    aspect = img_height / img_width
    map_half_height = map_half_width * aspect

    norm_x = (pose_x + map_half_width) / (2.0 * map_half_width)
    # Y invertido: +Y mapa = cima = menor valor de pixel
    norm_y = (-pose_y + map_half_height) / (2.0 * map_half_height)

    norm_x = max(0.0, min(1.0, norm_x))
    norm_y = max(0.0, min(1.0, norm_y))

    return norm_x, norm_y


def pixel_to_world_simple(
    norm_x: float,
    norm_y: float,
    camera_height: float = 3.0,
    horizontal_fov: float = 1.047,
    img_width: int = 640,
    img_height: int = 480,
) -> tuple:
    """Inverso de world_to_pixel_simple."""
    half_fov = horizontal_fov / 2.0
    map_half_width = camera_height * np.tan(half_fov)
    aspect = img_height / img_width
    map_half_height = map_half_width * aspect

    pose_x = norm_x * (2.0 * map_half_width) - map_half_width
    pose_y = -(norm_y * (2.0 * map_half_height) - map_half_height)

    return pose_x, pose_y


if __name__ == '__main__':
    # Câmera em z=3m, FOV=1.047rad (60°) → map_half_width = 3*tan(30°) ≈ 1.73m
    # Robô em (0, 0) → pixel centro (0.5, 0.5)
    cx, cy = world_to_pixel_simple(0.0, 0.0)
    assert abs(cx - 0.5) < 0.01 and abs(cy - 0.5) < 0.01, f'Center fail: {cx:.3f}, {cy:.3f}'

    # Robô em (+1.73, 0) → borda direita (norm_x ≈ 1.0)
    cx, cy = world_to_pixel_simple(1.73, 0.0)
    assert cx > 0.95, f'Right edge fail: {cx:.3f}'

    # Robô em (0, +1.3) → borda superior (norm_y ≈ 0.0)
    cx, cy = world_to_pixel_simple(0.0, 1.3)
    assert cy < 0.05, f'Top edge fail: {cy:.3f}'

    print('OK: projecao world->pixel validada com FOV real')
