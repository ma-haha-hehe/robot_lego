#yoloworld+mobile sam camera
from transformers import AutoModelForMaskGeneration, AutoProcessor, pipeline
from ultralytics import YOLOWorld  # <--- 添加这一行
from tqdm import tqdm
import random
from dataclasses import dataclass
from typing import Any, List, Dict, Optional, Union, Tuple

import cv2
import torch
import requests
import numpy as np
from PIL import Image
import plotly.express as px
import matplotlib.pyplot as plt
import plotly.graph_objects as go
from transformers import AutoModelForMaskGeneration, AutoProcessor, pipeline


@dataclass
class BoundingBox:
    xmin: int
    ymin: int
    xmax: int
    ymax: int

    @property
    def xyxy(self) -> List[float]:
        return [self.xmin, self.ymin, self.xmax, self.ymax]


@dataclass
class DetectionResult:
    score: float
    label: str
    box: BoundingBox
    mask: Optional[np.array] = None

    @classmethod
    def from_dict(cls, detection_dict: Dict) -> 'DetectionResult':
        return cls(score=detection_dict['score'],
                   label=detection_dict['label'],
                   box=BoundingBox(xmin=detection_dict['box']['xmin'],
                                   ymin=detection_dict['box']['ymin'],
                                   xmax=detection_dict['box']['xmax'],
                                   ymax=detection_dict['box']['ymax']))


def annotate(image: Union[Image.Image, np.ndarray], detection_results: List[DetectionResult]) -> np.ndarray:
    # Convert PIL Image to OpenCV format
    image_cv2 = np.array(image) if isinstance(image, Image.Image) else image
    image_cv2 = cv2.cvtColor(image_cv2, cv2.COLOR_RGB2BGR)

    # Iterate over detections and add bounding boxes and masks
    for detection in detection_results:
        label = detection.label
        score = detection.score
        box = detection.box
        mask = detection.mask

        # Sample a random color for each detection
        color = np.random.randint(0, 256, size=3)

        # Draw bounding box
        cv2.rectangle(image_cv2, (box.xmin, box.ymin), (box.xmax, box.ymax), color.tolist(), 2)
        cv2.putText(image_cv2, f'{label}: {score:.2f}', (box.xmin, box.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    color.tolist(), 2)

        # If mask is available, apply it
        if mask is not None:
            # Convert mask to uint8
            mask_uint8 = (mask * 255).astype(np.uint8)
            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image_cv2, contours, -1, color.tolist(), 2)

    return cv2.cvtColor(image_cv2, cv2.COLOR_BGR2RGB)
def annotate_video_frame(frame_bgr: np.ndarray, detection_results: List[DetectionResult]) -> np.ndarray:
    """
    直接在OpenCV BGR帧上绘制标注，并返回BGR帧。
    """
    image_cv2 = frame_bgr  # 输入已经是 BGR 格式

    # 遍历检测结果并添加边界框和掩码
    for detection in detection_results:
        label = detection.label
        score = detection.score
        box = detection.box
        mask = detection.mask

        # 为每个检测随机采样一个颜色
        color = np.random.randint(0, 256, size=3)
        color_list = color.tolist()

        # 绘制边界框
        cv2.rectangle(image_cv2, (box.xmin, box.ymin), (box.xmax, box.ymax), color_list, 2)
        cv2.putText(image_cv2, f'{label}: {score:.2f}', (box.xmin, box.ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    color_list, 2)

        # 如果有掩码，则应用它
        if mask is not None:
            mask_uint8 = (mask * 255).astype(np.uint8)
            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image_cv2, contours, -1, color_list, 2)

    return image_cv2  # 直接返回 BGR 格式的图像

def plot_detections(
        image: Union[Image.Image, np.ndarray],
        detections: List[DetectionResult],
        save_name: Optional[str] = None
) -> None:
    annotated_image = annotate(image, detections)
    plt.imshow(annotated_image)
    plt.axis('off')
    if save_name:
        plt.savefig(save_name, bbox_inches='tight')
    plt.show()


def random_named_css_colors(num_colors: int) -> List[str]:
    """
    Returns a list of randomly selected named CSS colors.

    Args:
    - num_colors (int): Number of random colors to generate.

    Returns:
    - list: List of randomly selected named CSS colors.
    """
    # List of named CSS colors
    named_css_colors = [
        'aliceblue', 'antiquewhite', 'aqua', 'aquamarine', 'azure', 'beige', 'bisque', 'black', 'blanchedalmond',
        'blue', 'blueviolet', 'brown', 'burlywood', 'cadetblue', 'chartreuse', 'chocolate', 'coral', 'cornflowerblue',
        'cornsilk', 'crimson', 'cyan', 'darkblue', 'darkcyan', 'darkgoldenrod', 'darkgray', 'darkgreen', 'darkgrey',
        'darkkhaki', 'darkmagenta', 'darkolivegreen', 'darkorange', 'darkorchid', 'darkred', 'darksalmon',
        'darkseagreen',
        'darkslateblue', 'darkslategray', 'darkslategrey', 'darkturquoise', 'darkviolet', 'deeppink', 'deepskyblue',
        'dimgray', 'dimgrey', 'dodgerblue', 'firebrick', 'floralwhite', 'forestgreen', 'fuchsia', 'gainsboro',
        'ghostwhite',
        'gold', 'goldenrod', 'gray', 'green', 'greenyellow', 'grey', 'honeydew', 'hotpink', 'indianred', 'indigo',
        'ivory',
        'khaki', 'lavender', 'lavenderblush', 'lawngreen', 'lemonchiffon', 'lightblue', 'lightcoral', 'lightcyan',
        'lightgoldenrodyellow',
        'lightgray', 'lightgreen', 'lightgrey', 'lightpink', 'lightsalmon', 'lightseagreen', 'lightskyblue',
        'lightslategray',
        'lightslategrey', 'lightsteelblue', 'lightyellow', 'lime', 'limegreen', 'linen', 'magenta', 'maroon',
        'mediumaquamarine',
        'mediumblue', 'mediumorchid', 'mediumpurple', 'mediumseagreen', 'mediumslateblue', 'mediumspringgreen',
        'mediumturquoise',
        'mediumvioletred', 'midnightblue', 'mintcream', 'mistyrose', 'moccasin', 'navajowhite', 'navy', 'oldlace',
        'olive',
        'olivedrab', 'orange', 'orangered', 'orchid', 'palegoldenrod', 'palegreen', 'paleturquoise', 'palevioletred',
        'papayawhip',
        'peachpuff', 'peru', 'pink', 'plum', 'powderblue', 'purple', 'rebeccapurple', 'red', 'rosybrown', 'royalblue',
        'saddlebrown',
        'salmon', 'sandybrown', 'seagreen', 'seashell', 'sienna', 'silver', 'skyblue', 'slateblue', 'slategray',
        'slategrey',
        'snow', 'springgreen', 'steelblue', 'tan', 'teal', 'thistle', 'tomato', 'turquoise', 'violet', 'wheat', 'white',
        'whitesmoke', 'yellow', 'yellowgreen'
    ]

    # Sample random named CSS colors
    return random.sample(named_css_colors, min(num_colors, len(named_css_colors)))


def plot_detections_plotly(
        image: np.ndarray,
        detections: List[DetectionResult],
        class_colors: Optional[Dict[str, str]] = None
) -> None:
    # If class_colors is not provided, generate random colors for each class
    if class_colors is None:
        num_detections = len(detections)
        colors = random_named_css_colors(num_detections)
        class_colors = {}
        for i in range(num_detections):
            class_colors[i] = colors[i]

    fig = px.imshow(image)

    # Add bounding boxes
    shapes = []
    annotations = []
    for idx, detection in enumerate(detections):
        label = detection.label
        box = detection.box
        score = detection.score
        mask = detection.mask

        polygon = mask_to_polygon(mask)

        fig.add_trace(go.Scatter(
            x=[point[0] for point in polygon] + [polygon[0][0]],
            y=[point[1] for point in polygon] + [polygon[0][1]],
            mode='lines',
            line=dict(color=class_colors[idx], width=2),
            fill='toself',
            name=f"{label}: {score:.2f}"
        ))

        xmin, ymin, xmax, ymax = box.xyxy
        shape = [
            dict(
                type="rect",
                xref="x", yref="y",
                x0=xmin, y0=ymin,
                x1=xmax, y1=ymax,
                line=dict(color=class_colors[idx])
            )
        ]
        annotation = [
            dict(
                x=(xmin + xmax) // 2, y=(ymin + ymax) // 2,
                xref="x", yref="y",
                text=f"{label}: {score:.2f}",
            )
        ]

        shapes.append(shape)
        annotations.append(annotation)

    # Update layout
    button_shapes = [dict(label="None", method="relayout", args=["shapes", []])]
    button_shapes = button_shapes + [
        dict(label=f"Detection {idx + 1}", method="relayout", args=["shapes", shape]) for idx, shape in
        enumerate(shapes)
    ]
    button_shapes = button_shapes + [dict(label="All", method="relayout", args=["shapes", sum(shapes, [])])]

    fig.update_layout(
        xaxis=dict(visible=False),
        yaxis=dict(visible=False),
        # margin=dict(l=0, r=0, t=0, b=0),
        showlegend=True,
        updatemenus=[
            dict(
                type="buttons",
                direction="up",
                buttons=button_shapes
            )
        ],
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        )
    )

    # Show plot
    fig.show()


def mask_to_polygon(mask: np.ndarray) -> List[List[int]]:
    # Find contours in the binary mask
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the contour with the largest area
    largest_contour = max(contours, key=cv2.contourArea)

    # Extract the vertices of the contour
    polygon = largest_contour.reshape(-1, 2).tolist()

    return polygon


def polygon_to_mask(polygon: List[Tuple[int, int]], image_shape: Tuple[int, int]) -> np.ndarray:
    """
    Convert a polygon to a segmentation mask.

    Args:
    - polygon (list): List of (x, y) coordinates representing the vertices of the polygon.
    - image_shape (tuple): Shape of the image (height, width) for the mask.

    Returns:
    - np.ndarray: Segmentation mask with the polygon filled.
    """
    # Create an empty mask
    mask = np.zeros(image_shape, dtype=np.uint8)

    # Convert polygon to an array of points
    pts = np.array(polygon, dtype=np.int32)

    # Fill the polygon with white color (255)
    cv2.fillPoly(mask, [pts], color=(255,))

    return mask


def load_image(image_str: str) -> Image.Image:
    if image_str.startswith("http"):
        image = Image.open(requests.get(image_str, stream=True).raw).convert("RGB")
    else:
        image = Image.open(image_str).convert("RGB")

    return image


def get_boxes(results: DetectionResult) -> List[List[List[float]]]:
    boxes = []
    for result in results:
        xyxy = result.box.xyxy
        boxes.append(xyxy)

    return [boxes]


def refine_masks(masks: torch.BoolTensor, polygon_refinement: bool = False) -> List[np.ndarray]:
    masks = masks.cpu().float()
    masks = masks.permute(0, 2, 3, 1)
    masks = masks.mean(axis=-1)
    masks = (masks > 0).int()
    masks = masks.numpy().astype(np.uint8)
    masks = list(masks)

    if polygon_refinement:
        for idx, mask in enumerate(masks):
            shape = mask.shape
            polygon = mask_to_polygon(mask)
            mask = polygon_to_mask(polygon, shape)
            masks[idx] = mask

    return masks


def detect_video_yolo(
        image: Image.Image,
        detector_model: YOLOWorld,  # 注意类型提示已更改
        conf_threshold: float = 0.3
) -> List[DetectionResult]:
    """
    使用预加载的 YOLOWorld 模型在图像中进行零样本检测。
    """
    # YOLOWorld (ultralytics) 可以直接处理 PIL 图像
    # verbose=False 可以关闭控制台的啰嗦日志
    results = detector_model.predict(image, conf=conf_threshold, verbose=False)

    # 从结果中提取数据
    result = results[0]  # 获取第一张图的结果
    boxes_xyxy = result.boxes.xyxy.cpu().tolist()
    scores = result.boxes.conf.cpu().tolist()
    class_indices = result.boxes.cls.cpu().tolist()

    # 从模型中获取类别的文本标签
    class_names = [detector_model.names[int(i)] for i in class_indices]

    # 构建我们自定义的 DetectionResult 列表，供 segment_video 使用
    detection_results = []
    for box, score, label in zip(boxes_xyxy, scores, class_names):
        detection_results.append(
            DetectionResult(
                score=score,
                label=label,
                box=BoundingBox(
                    xmin=int(box[0]),
                    ymin=int(box[1]),
                    xmax=int(box[2]),
                    ymax=int(box[3])
                )
                # mask 在此阶段为 None
            )
        )
    return detection_results


def segment_video(
    image: Image.Image,
    detection_results: List[DetectionResult],
    segmentator,           # 预先加载好的 HF-SAM 模型
    processor,             # 预先加载好的 HF-SAM 处理器
    polygon_refinement: bool = False,
    device: Optional[str] = None
) -> List[DetectionResult]:
    """
    使用 HuggingFace 版 SAM 为 YOLO 检测到的 bbox 生成掩码（视频单帧）。
    - 处理了“本帧无检测结果”的情况（直接提前返回）
    - 处理了 Apple MPS 的 float64 -> float32 问题
    - 变量名与传参一致（segmentator）
    """
    if device is None:
        device = "cuda" if torch.cuda.is_available() else ("mps" if torch.backends.mps.is_available() else "cpu")

    # 1) 没有检测结果，直接返回原结果（不做 SAM）
    if not detection_results:
        # print("⚠️ 本帧未检测到目标，跳过 SAM 分割")
        return detection_results

    # 2) 按 HF-SAM 的格式准备 boxes：[[[x1,y1,x2,y2], ...]]
    boxes = get_boxes(detection_results)  # 期望返回 [[[...], [...], ...]]
    if not boxes or len(boxes[0]) == 0:
        # print("⚠️ 本帧检测到的 boxes 为空，跳过 SAM 分割")
        return detection_results

    # 3) 送入 processor
    inputs = processor(images=image, input_boxes=boxes, return_tensors="pt")

    # 4) —— 统一把所有 float64 的张量转成 float32（不仅仅是 input_boxes）
    for k, v in list(inputs.items()):
        if isinstance(v, torch.Tensor) and v.dtype == torch.float64:
            inputs[k] = v.to(torch.float32)

    # 如果你想双保险，可以保留这行（但已不再必须）
    # if 'input_boxes' in inputs and inputs['input_boxes'].dtype == torch.float64:
    #     inputs['input_boxes'] = inputs['input_boxes'].to(torch.float32)

    # 5) 再把能 to(device) 的都搬到 MPS/CUDA/CPU
    for k, v in list(inputs.items()):
        if hasattr(v, "to"):
            inputs[k] = v.to(device)

    # 6) SAM 前向
    with torch.no_grad():
        outputs = segmentator(**inputs)

    # 7) 后处理，得到二值 mask（H,W）
    masks = processor.post_process_masks(
        masks=outputs.pred_masks,
        original_sizes=inputs["original_sizes"],
        reshaped_input_sizes=inputs["reshaped_input_sizes"]
    )[0]

    masks = refine_masks(masks, polygon_refinement)

    # 8) 把 mask 回填进 detection_results
    for det, m in zip(detection_results, masks):
        det.mask = m

    return detection_results

def grounded_segmentation(
    image: Union[Image.Image, str],
    labels: List[str],
    threshold: float = 0.3,
    polygon_refinement: bool = False,
    detector_id: Optional[str] = None,
    segmenter_id: Optional[str] = None
) -> Tuple[np.ndarray, List[DetectionResult]]:
    if isinstance(image, str):
        image = load_image(image)

    detections = detect(image, labels, threshold, detector_id)
    detections = segment(image, detections, polygon_refinement, segmenter_id)

    return np.array(image), detections


if __name__ == "__main__":

    # 1. --- 设置参数 ---
    WEBCAM_ID = 0  # 0 是你的默认摄像头
    LABELS = ["person", "cell phone", "cup", "laptop", "book", "bottle"]  # <--- YOLO-World 需要设置标签
    CONF_THRESHOLD = 0.2  # YOLO-World 很快，可以适当调低阈值
    POLYGON_REFINEMENT = False  # 保持 False 以获得最高速度

    # --- 新的模型 ID ---
    DETECTOR_MODEL_NAME = "weights/yolov8s-world.pt"  # s=Small, m=Medium, l=Large
    SEGMENTER_ID = "facebook/sam-vit-base"  # <--- 这就是 MobileSAM！

    # 自动检测设备
    if torch.cuda.is_available():
        DEVICE = "cuda"
    elif torch.backends.mps.is_available():
        DEVICE = "mps"
    else:
        DEVICE = "cpu"

    # 2. --- 在循环外加载模型 ---
    print(f"正在加载模型到 {DEVICE} ...")

    # (新) 加载 YOLO-World detector
    # ultralytics 会自动下载模型
    detector_model = YOLOWorld(DETECTOR_MODEL_NAME)

    # (!!!) 关键: 为 YOLO-World 设置你要检测的类别
    detector_model.set_classes(LABELS)

    # (新) 将模型移动到 GPU (如果可用)
    # 注意: YOLO-World 的 .to() 方法是 ultralytics 包装过的
    detector_model.to(DEVICE)
    print(f"YOLO-World ({DETECTOR_MODEL_NAME}) 加载完毕。")

    # (新) 加载 MobileSAM segmenter (和以前一样，只是 ID 变了)
    sam_model = AutoModelForMaskGeneration.from_pretrained(SEGMENTER_ID).to(DEVICE)
    sam_processor = AutoProcessor.from_pretrained(SEGMENTER_ID)
    print(f"MobileSAM ({SEGMENTER_ID}) 加载完毕。")

    # 3. --- 打开摄像头 ---
    cap = cv2.VideoCapture(WEBCAM_ID)
    if not cap.isOpened():
        print(f"错误: 无法打开摄像头 {WEBCAM_ID}")
        exit()

    print(f"开始实时处理摄像头: {WEBCAM_ID} (按 'q' 键退出)")

    # 5. --- 逐帧处理视频 (while 循环) ---
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("错误：无法读取摄像头画面。")
            break

        frame = cv2.flip(frame, 1)  # 镜像翻转

        # 将 OpenCV 帧 (BGR) 转换为 PIL 图像 (RGB)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_pil = Image.fromarray(frame_rgb)

        # 步骤 A: 检测 (使用新的 YOLO-World 函数)
        detections = detect_video_yolo(
            image=image_pil,
            detector_model=detector_model,  # 传入加载好的 YOLO-World 模型
            conf_threshold=CONF_THRESHOLD
        )

        # 步骤 B: 分割 (使用完全相同的 segment_video 函数!)
        detections_with_masks = segment_video(
            image=image_pil,
            detection_results=detections,
            segmentator=sam_model,  # 传入 MobileSAM
            processor=sam_processor,  # 传入 MobileSAM 的处理器
            polygon_refinement=POLYGON_REFINEMENT,
            device=DEVICE
        )

        # 步骤 C: 标注 (使用完全相同的 annotate_video_frame 函数)
        annotated_frame = annotate_video_frame(frame, detections_with_masks)

        # 步骤 D: 显示画面
        cv2.imshow("实时分割 (YOLO-World + MobileSAM)", annotated_frame)

        # 步骤 E: 检查退出键
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 6. --- 释放资源 ---
    cap.release()
    cv2.destroyAllWindows()
    print("处理完成！")