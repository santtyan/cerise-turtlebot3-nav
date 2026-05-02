#!/usr/bin/env python3
import cv2
import numpy as np
from pathlib import Path
import sys

class BBoxViewer:
    def __init__(self):
        self.img_dir = Path("dataset/images/train")
        self.label_dir = Path("dataset/labels/train")
        self.images = sorted(list(self.img_dir.glob("*.jpg")))
        self.current_idx = 0
        self.window_name = "BBox Verification - Use ARROWS to navigate, Q to quit"

        if not self.images:
            print("❌ Nenhuma imagem encontrada em dataset/images/train/")
            sys.exit(1)

        print(f"✅ {len(self.images)} imagens carregadas")
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1000, 750)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        self.show_current()

    def mouse_callback(self, event, x, y, flags, param):
        """Callback para mouse (futuro)"""
        pass

    def draw_image(self):
        img_path = self.images[self.current_idx]
        img = cv2.imread(str(img_path))

        if img is None:
            print(f"❌ Erro ao ler: {img_path}")
            return

        h, w = img.shape[:2]
        label_path = self.label_dir / (img_path.stem + ".txt")

        # Contar robôs anotados
        robot_count = 0
        if label_path.exists():
            with open(label_path) as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) < 5:
                        continue

                    robot_count += 1
                    class_id = int(parts[0])
                    x_center = float(parts[1]) * w
                    y_center = float(parts[2]) * h
                    box_w = float(parts[3]) * w
                    box_h = float(parts[4]) * h

                    # Desenhar bbox
                    x1 = int(x_center - box_w / 2)
                    y1 = int(y_center - box_h / 2)
                    x2 = int(x_center + box_w / 2)
                    y2 = int(y_center + box_h / 2)

                    # Clampar
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(w, x2), min(h, y2)

                    # Desenhar com cor brilhante
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(img, f"Robot {class_id}", (x1, max(20, y1 - 10)),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Informações no topo
        info_text = f"[{self.current_idx+1}/{len(self.images)}] {img_path.name} | {robot_count} robôs"
        cv2.putText(img, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.putText(img, "← → para navegar | Q para sair",
                   (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

        cv2.imshow(self.window_name, img)

    def show_current(self):
        self.draw_image()

    def run(self):
        print("\n📊 GUI aberta!")
        print("Controles:")
        print("  → Próxima imagem")
        print("  ← Imagem anterior")
        print("  Q Sair")
        print("\n✅ Verifique se os bboxes verdes estão alinhados com os robôs brancos")

        while True:
            key = cv2.waitKey(0) & 0xFF

            if key == ord('q') or key == ord('Q'):
                print("\n✅ Verificação encerrada")
                break
            elif key == 83 or key == ord('d'):  # RIGHT arrow ou 'D'
                self.current_idx = (self.current_idx + 1) % len(self.images)
                self.show_current()
            elif key == 81 or key == ord('a'):  # LEFT arrow ou 'A'
                self.current_idx = (self.current_idx - 1) % len(self.images)
                self.show_current()

        cv2.destroyAllWindows()

if __name__ == "__main__":
    viewer = BBoxViewer()
    viewer.run()
