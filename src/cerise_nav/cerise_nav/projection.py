"""Projeção mapa (metros) → pixel para câmera overhead."""

def world_to_pixel(pose_x: float, pose_y: float, 
                   map_width: float = 10.0, map_height: float = 10.0,
                   img_width: int = 640, img_height: int = 480) -> tuple:
    """
    Transforma pose do mapa (metros, centro em 0,0) → pixel normalizado YOLO (0-1).
    
    Args:
        pose_x, pose_y: coordenadas no mapa (metros)
        map_width, map_height: dimensões do mapa (metros)
        img_width, img_height: resolução câmera (pixels)
    
    Returns:
        (cx, cy): centro normalizado (0-1) para YOLO format
    """
    # Normaliza mapa [−width/2, +width/2] → [0, 1]
    norm_x = (pose_x + map_width / 2.0) / map_width
    norm_y = (pose_y + map_height / 2.0) / map_height
    
    # Clamp para [0, 1]
    norm_x = max(0.0, min(1.0, norm_x))
    norm_y = max(0.0, min(1.0, norm_y))
    
    return norm_x, norm_y


def pixel_to_world(norm_x: float, norm_y: float,
                   map_width: float = 10.0, map_height: float = 10.0) -> tuple:
    """Inverso: pixel normalizado → coordenada mapa."""
    pose_x = norm_x * map_width - map_width / 2.0
    pose_y = norm_y * map_height - map_height / 2.0
    return pose_x, pose_y


# Testa
if __name__ == '__main__':
    # robot1 no centro (0, 0) → pixel (0.5, 0.5)
    cx, cy = world_to_pixel(0.0, 0.0)
    assert abs(cx - 0.5) < 0.01 and abs(cy - 0.5) < 0.01, f'Center fail: {cx}, {cy}'
    
    # robot2 em (-2, -2) → pixel (0.3, 0.3)
    cx, cy = world_to_pixel(-2.0, -2.0)
    assert abs(cx - 0.3) < 0.01 and abs(cy - 0.3) < 0.01, f'Corner fail: {cx}, {cy}'
    
    print('✅ Projeção OK: mapa→pixel validada')
