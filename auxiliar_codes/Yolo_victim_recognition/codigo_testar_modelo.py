import cv2
from ultralytics import YOLO

model_path = r"./weights/best.pt"

model = YOLO(model_path)

image_path = r"C:\Users\nicol\Documents\yollo11test\yollo11test\rescue simulation 2025.v9i.yolov11\test\images\frame_0265_jpg.rf.46179eafbe5750c461a9df476613e924.jpg"  # Modifique para sua imagem

image = cv2.imread(image_path)


results = model(image)

class_names = model.names

for result in results:
    for box in result.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf[0])
        class_id = int(box.cls[0])
        class_name = class_names[class_id]

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        text = f"{class_name}: {conf:.2f}"
        cv2.putText(
            image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
        )

# Exibir a imagem
cv2.imshow("Detecção", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
