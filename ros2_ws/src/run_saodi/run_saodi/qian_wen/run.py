import os
import cv2
import base64
from io import BytesIO
from PIL import Image
import numpy as np
from openai import OpenAI

def qwen_vl_query(image: np.ndarray, prompt: str) -> str:
    """
    将 OpenCV 图像和用户 prompt 一起发送给 Qwen-VL 模型，返回文本回答
    """
    # 1. BGR → RGB → PIL Image
    img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(img_rgb)

    # 2. 编码为 Base64
    buffered = BytesIO()
    pil_img.save(buffered, format="JPEG")
    img_bytes = buffered.getvalue()
    img_base64 = base64.b64encode(img_bytes).decode("utf-8")
    data_url = f"data:image/jpeg;base64,{img_base64}"

    # 3. 构建请求
    client = OpenAI(
        api_key=os.getenv("DASHSCOPE_API_KEY"),
        base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
    )

    completion = client.chat.completions.create(
        model="qwen-vl-max-latest",
        messages=[
            {"role": "system", "content": [{"type": "text", "text": "You are a helpful assistant."}]},
            {"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": data_url}},
                {"type": "text", "text": prompt}
            ]}
        ]
    )

    return completion.choices[0].message.content

