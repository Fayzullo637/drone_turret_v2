# Drone Detection — YOLOv8 Training Notebook
# ==========================================
# Откройте этот файл на Google Colab: https://colab.research.google.com/
# Runtime → Change runtime type → GPU (T4)
#
# После обучения скачайте best.pt и скопируйте в:
#   drone_turret_v2/models/drone_best.pt

# ============================================
# CELL 1: Установка зависимостей
# ============================================
# !pip install -q ultralytics roboflow

# ============================================
# CELL 2: Проверка GPU
# ============================================
# import torch
# print(f"GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU ONLY'}")

# ============================================
# CELL 3: Скачивание датасета с Roboflow
# ============================================
# Зарегистрируйтесь бесплатно на https://roboflow.com/
# Найдите датасет "drone detection" → Export → YOLOv8 → Get API Key
#
# from roboflow import Roboflow
# rf = Roboflow(api_key="YOUR_API_KEY")  # Замените на свой ключ
# project = rf.workspace("drone-tracking-lrlvi").project("uav-detection-and-tracking-sdyxb")
# dataset = project.version(1).download("yolov8")
# data_yaml = f"{dataset.location}/data.yaml"

# ВАРИАНТ Б: VisDrone (встроен в Ultralytics)
# data_yaml = "VisDrone.yaml"

# ============================================
# CELL 4: Обучение модели
# ============================================
# from ultralytics import YOLO
# model = YOLO('yolov8n.pt')
# results = model.train(
#     data=data_yaml,
#     epochs=100,
#     imgsz=640,
#     batch=16,
#     device=0,
#     augment=True,
#     patience=20,
#     save=True,
#     project='drone_det',
#     name='train_v1',
#     mosaic=1.0,
#     mixup=0.1,
#     copy_paste=0.1,
# )

# ============================================
# CELL 5: Валидация
# ============================================
# model = YOLO('drone_det/train_v1/weights/best.pt')
# metrics = model.val(data=data_yaml)
# print(f"mAP50: {metrics.box.map50:.4f}")
# print(f"mAP50-95: {metrics.box.map:.4f}")

# ============================================
# CELL 6: Экспорт и скачивание
# ============================================
# model.export(format='onnx', imgsz=640, simplify=True)
# from google.colab import files
# files.download('drone_det/train_v1/weights/best.pt')
#
# Потом: переименуйте best.pt -> drone_best.pt
# Скопируйте в drone_turret_v2/models/drone_best.pt
