import cv2
import threading
import ast  # 用于安全解析字符串为列表
from qian_wen.run import qwen_vl_query

# 用于存储异步推理结果的全局变量
detected_boxes = []

def run_inference(frame, prompt):
    global detected_boxes
    result = qwen_vl_query(frame, prompt)
    print("模型回答：", result)

    try:
        # 将字符串转换为列表
        boxes = ast.literal_eval(result)
        if isinstance(boxes, list):
            detected_boxes = boxes  # 更新全局变量
    except Exception as e:
        print("解析模型结果失败：", e)

def main():
    global detected_boxes
    cap = cv2.VideoCapture('/dev/video0')

    if not cap.isOpened():
        print("无法打开摄像头 /dev/video0")
        return

    print("按下 'q' 退出，按下 'k' 推理垃圾桶位置")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取视频帧")
            break

        # 如果已有检测框，则绘制到当前帧
        for box in detected_boxes:
            if isinstance(box, (list, tuple)) and len(box) == 4:
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, '垃圾桶', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow('Camera - /dev/video0', frame)

        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            break
        elif k == ord('k'):
            frame_copy = frame.copy()
            prompt = "请在图像中找到所有的垃圾桶。只输出它们的边界框坐标，格式为 [(x_min, y_min, x_max, y_max)]，不要返回其他内容。"
            threading.Thread(target=run_inference, args=(frame_copy, prompt)).start()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

